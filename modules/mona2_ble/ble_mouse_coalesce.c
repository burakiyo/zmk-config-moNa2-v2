/*
 * BLE マウスレポートを「並べる」代わりに「まとめる」送信処理 (ZMK v0.3 用)
 *
 * ZMK 標準の送信処理は、マウスのレポートを 1 件ずつ待ち行列
 * (zmk_hog_mouse_msgq) と Bluetooth スタックのバッファに積んでいく。
 * Mac が受け取るペースがセンサーの出すペースに追いつかないと行列が伸び続け、
 * カーソルが古い動きを時間差でなぞる (= 遅れてついてくる)。
 *
 * ここでは zmk_hog_send_mouse_report() をリンカの --wrap で差し替え、
 * Bluetooth スタックに渡すのを最大 MAX_IN_FLIGHT 件に抑える。送れない間に
 * 来た動きは、まだ送っていない最新のレポートに足し込む。動きは捨てず、
 * 古い動きの後ろに並ぶこともない。
 *
 * さらに CONFIG_MONA2_BLE_MOUSE_PACE では、Mac の受け取り (接続イベント)
 * 1 回につき 1 件だけ、受け取りの少し前 (PACE_LEAD_US) に、それまでの動きを
 * まとめて送る。届く間隔が接続間隔ちょうどに揃うので、macOS の加速の効き方が
 * 安定する (Logitech のマウスと同じやり方)。
 *
 * 受け取りのタイミングは、送信完了の通知から割り出す。Mac は受け取りの最初に
 * 前回の受信確認を返すので、通知が届く時刻は「受け取りの時刻 + 少し」になる。
 * そこから接続間隔だけ先が次の受け取りなので、その PACE_LEAD_US 前に送る。
 *
 * クリック (ボタン状態の変化) を含むレポートには後から来た動きを足さず、
 * 歩調も待たずにすぐ送る。足すとクリック位置がずれ、待つとクリックが遅れるため。
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/ble.h>
#include <zmk/hid.h>
#include <zmk/hog.h>

LOG_MODULE_REGISTER(ble_mouse_coalesce, CONFIG_ZMK_LOG_LEVEL);

/* zmk/app/src/hog.c で static なしに定義されているもの */
extern struct k_work_q hog_work_q;
extern const struct bt_gatt_service_static hog_svc;

/* hog.c がマウスの通知に使っているのと同じ属性 (characteristic 宣言) */
#define MOUSE_CHRC_INDEX 13
/* その characteristic の Report Reference ディスクリプタ (宣言, 値, CCC, 参照) */
#define MOUSE_REPORT_REF_INDEX (MOUSE_CHRC_INDEX + 3)

#define SLOTS CONFIG_MONA2_BLE_MOUSE_COALESCE_SLOTS
#define MAX_IN_FLIGHT CONFIG_MONA2_BLE_MOUSE_COALESCE_MAX_IN_FLIGHT
#define STALL_MS CONFIG_MONA2_BLE_MOUSE_COALESCE_STALL_MS

#if IS_ENABLED(CONFIG_MONA2_BLE_MOUSE_PACE)
#define PACE_LEAD_US CONFIG_MONA2_BLE_MOUSE_PACE_LEAD_US
#define PACE_IDLE_EVENTS CONFIG_MONA2_BLE_MOUSE_PACE_IDLE_EVENTS
#else
#define PACE_LEAD_US 0
#define PACE_IDLE_EVENTS 0
#endif

/*
 * ZMK 標準の送信処理。属性の並びが想定と違うときだけ使う。
 * この参照があるので、--wrap が効いていないとリンクエラーになる
 * (差し替えが黙って無効になることはない)。
 */
int __real_zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *body);

struct pending_report {
    struct zmk_hid_mouse_report_body body;
    /* 直前のレポートからボタン状態が変わった (クリックの押し・離し) */
    bool button_edge;
};

static struct k_spinlock lock;
static struct pending_report ring[SLOTS];
static uint8_t ring_head;
static uint8_t ring_count;
static uint8_t edges_in_ring;
static zmk_mouse_button_flags_t last_buttons;

/* Bluetooth スタックに渡して、まだ送信完了の通知が来ていない件数 */
static uint8_t in_flight;
/* 接続が切れたら進める。古い送信完了の通知で in_flight を減らさないため */
static uint32_t generation;
static int64_t last_progress_ms;
static const struct bt_conn *flight_conn;

/* 歩調合わせ (CONFIG_MONA2_BLE_MOUSE_PACE) */
static bool pacing;            /* 受け取りの歩調に合わせて送っている */
static bool phase_locked;      /* 送信完了の通知から歩調を割り出せている */
static int64_t flush_target_us; /* 次にまとめて送る予定の時刻 */
static int64_t last_completion_us;
static uint32_t interval_us;   /* 接続間隔 */
static uint16_t idle_flushes;  /* 送るものがなかった回数 (連続) */

/* 0 = 未確認, 1 = 想定どおり, -1 = 想定外 (標準の送信処理を使う) */
static int8_t attr_state;

static void notify_sent(struct bt_conn *conn, void *user_data);
static void send_work_handler(struct k_work *work);
static void flush_work_handler(struct k_work *work);
static K_WORK_DEFINE(send_work, send_work_handler);
static K_WORK_DELAYABLE_DEFINE(flush_work, flush_work_handler);

static inline int64_t now_us(void) { return k_ticks_to_us_floor64(k_uptime_ticks()); }

static bool mouse_attr_ok(void) {
    if (attr_state == 0) {
        const struct bt_gatt_attr *attrs = hog_svc.attrs;
        bool ok = hog_svc.attr_count > MOUSE_REPORT_REF_INDEX &&
                  bt_uuid_cmp(attrs[MOUSE_CHRC_INDEX].uuid, BT_UUID_GATT_CHRC) == 0 &&
                  bt_uuid_cmp(attrs[MOUSE_REPORT_REF_INDEX].uuid, BT_UUID_HIDS_REPORT_REF) == 0 &&
                  attrs[MOUSE_REPORT_REF_INDEX].user_data != NULL &&
                  *(const uint8_t *)attrs[MOUSE_REPORT_REF_INDEX].user_data ==
                      ZMK_HID_REPORT_ID_MOUSE;

        attr_state = ok ? 1 : -1;
        if (!ok) {
            LOG_ERR("Unexpected HOG attribute layout, using the stock mouse sender");
        }
    }

    return attr_state == 1;
}

/* 接続間隔が短すぎると「受け取りの少し前」を取れないので、歩調合わせはしない */
static inline bool pace_enabled(void) {
    return IS_ENABLED(CONFIG_MONA2_BLE_MOUSE_PACE) && interval_us > PACE_LEAD_US + 1000;
}

static inline int16_t sat_add16(int16_t a, int16_t b) {
    return (int16_t)CLAMP((int32_t)a + (int32_t)b, INT16_MIN, INT16_MAX);
}

static void merge_into(struct zmk_hid_mouse_report_body *dst,
                       const struct zmk_hid_mouse_report_body *src) {
    dst->buttons = src->buttons;
    dst->d_x = sat_add16(dst->d_x, src->d_x);
    dst->d_y = sat_add16(dst->d_y, src->d_y);
    dst->d_scroll_y = sat_add16(dst->d_scroll_y, src->d_scroll_y);
    dst->d_scroll_x = sat_add16(dst->d_scroll_x, src->d_scroll_x);
}

static void pop_head_locked(struct zmk_hid_mouse_report_body *out) {
    *out = ring[ring_head].body;
    if (ring[ring_head].button_edge) {
        edges_in_ring--;
    }
    ring_head = (ring_head + 1) % SLOTS;
    ring_count--;
}

static void drop_pending_locked(void) {
    ring_head = 0;
    ring_count = 0;
    edges_in_ring = 0;
}

static void reset_flight_locked(const struct bt_conn *conn, int64_t now_ms) {
    in_flight = 0;
    generation++;
    flight_conn = conn;
    last_progress_ms = now_ms;
    /* 歩調も取り直す。予約済みのまとめ送りは pacing を見て何もせず終わる */
    pacing = false;
    phase_locked = false;
}

/* 歩調合わせを始める。最初の予定は「いまから接続間隔 1 回分あと」 */
static void start_pacing_locked(void) {
    pacing = true;
    phase_locked = false;
    idle_flushes = 0;
    flush_target_us = now_us() + interval_us;
    k_work_reschedule_for_queue(&hog_work_q, &flush_work, K_USEC(interval_us));
}

/*
 * 送信先の接続を取り、接続先の変化や送信の詰まりを片付ける。
 * 未接続なら溜まった分を捨てて NULL を返す (つながり直したときに古い動きを
 * 再生しないため)。返した接続は bt_conn_unref() すること。
 */
static struct bt_conn *get_conn(void) {
    struct bt_conn *conn = zmk_ble_active_profile_conn();
    int64_t now_ms = k_uptime_get();
    struct bt_conn_info info;
    bool have_info = conn != NULL && bt_conn_get_info(conn, &info) == 0;

    k_spinlock_key_t key = k_spin_lock(&lock);

    if (conn == NULL) {
        drop_pending_locked();
        reset_flight_locked(NULL, now_ms);
    } else if (conn != flight_conn) {
        /* 接続先 (プロファイル) が変わった */
        reset_flight_locked(conn, now_ms);
    } else if (in_flight >= MAX_IN_FLIGHT && now_ms - last_progress_ms > STALL_MS) {
        /* 送信完了の通知が来ないまま止まっている。待ち続けない */
        LOG_WRN("Mouse notifications stalled, resetting");
        reset_flight_locked(conn, now_ms);
    }

    if (have_info && info.type == BT_CONN_TYPE_LE) {
        interval_us = (uint32_t)info.le.interval * 1250U;
    }

    k_spin_unlock(&lock, key);
    return conn;
}

/* 行列の先頭を 1 件送る。行列が空か、渡せる件数の上限なら false */
static bool try_send_head(struct bt_conn *conn) {
    struct zmk_hid_mouse_report_body report;

    k_spinlock_key_t key = k_spin_lock(&lock);
    if (ring_count == 0 || in_flight >= MAX_IN_FLIGHT) {
        k_spin_unlock(&lock, key);
        return false;
    }
    pop_head_locked(&report);
    in_flight++;
    last_progress_ms = k_uptime_get();
    uint32_t gen = generation;
    k_spin_unlock(&lock, key);

    struct bt_gatt_notify_params params = {
        .attr = &hog_svc.attrs[MOUSE_CHRC_INDEX],
        .data = &report,
        .len = sizeof(report),
        .func = notify_sent,
        .user_data = UINT_TO_POINTER(gen),
    };

    int err = bt_gatt_notify_cb(conn, &params);
    if (err) {
        key = k_spin_lock(&lock);
        if (gen == generation && in_flight > 0) {
            in_flight--;
        }
        k_spin_unlock(&lock, key);

        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else {
            LOG_DBG("Error notifying %d", err);
        }
    }

    /* 送れなかった分は捨てる (ZMK 標準と同じ)。行列は先に進める */
    return true;
}

int __wrap_zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *body) {
    if (!mouse_attr_ok()) {
        return __real_zmk_hog_send_mouse_report(body);
    }

    k_spinlock_key_t key = k_spin_lock(&lock);

    bool edge = body->buttons != last_buttons;
    last_buttons = body->buttons;

    struct pending_report *tail =
        ring_count > 0 ? &ring[(ring_head + ring_count - 1) % SLOTS] : NULL;

    if (!edge && tail != NULL && (!tail->button_edge || ring_count == SLOTS)) {
        /* 動きだけのレポートは、未送信の最新レポートに足し込む */
        merge_into(&tail->body, body);
    } else {
        if (ring_count == SLOTS) {
            /* クリックが溜まりすぎた (回線が長く止まっている)。最古を捨てる */
            if (ring[ring_head].button_edge) {
                edges_in_ring--;
            }
            ring_head = (ring_head + 1) % SLOTS;
            ring_count--;
        }
        struct pending_report *slot = &ring[(ring_head + ring_count) % SLOTS];
        slot->body = *body;
        slot->button_edge = edge;
        ring_count++;
        if (edge) {
            edges_in_ring++;
        }
    }

    /* 歩調に合わせて送っている間、動きだけなら予定の時刻に任せる */
    bool kick = edge || !pacing || !pace_enabled();

    k_spin_unlock(&lock, key);

    if (kick) {
        k_work_submit_to_queue(&hog_work_q, &send_work);
    }
    return 0;
}

/* 送信完了の通知 (システムワークキューで呼ばれる) */
static void notify_sent(struct bt_conn *conn, void *user_data) {
    uint32_t gen = POINTER_TO_UINT(user_data);
    int64_t now = now_us();

    k_spinlock_key_t key = k_spin_lock(&lock);

    if (gen == generation && in_flight > 0) {
        in_flight--;
    }
    last_progress_ms = k_uptime_get();

    /*
     * 同じ受け取りの中で複数の通知が来たら、最初のものだけを歩調の目安にする
     * (後のものほど受け取りの時刻から遠い)。
     */
    bool first_in_event = now - last_completion_us > interval_us / 2;
    last_completion_us = now;

    if (gen == generation && pacing && first_in_event && pace_enabled()) {
        int64_t candidate = now + interval_us - PACE_LEAD_US;
        int64_t diff = candidate - flush_target_us;

        if (!phase_locked || diff < -(int64_t)interval_us / 2 ||
            diff > (int64_t)interval_us / 2) {
            /* 初回か、大きくずれた: そのまま合わせる */
            flush_target_us = candidate;
            phase_locked = true;
        } else if (diff < 0) {
            /* 早めに来た: 受け取りの時刻により近いので、そちらに合わせる */
            flush_target_us = candidate;
        } else {
            /* 遅めに来た: 通知の遅れのばらつきかもしれないので、少しだけ寄せる */
            flush_target_us += diff / 8;
        }

        k_work_reschedule_for_queue(&hog_work_q, &flush_work,
                                    K_USEC(MAX(flush_target_us - now, 0)));
    }

    k_spin_unlock(&lock, key);

    k_work_submit_to_queue(&hog_work_q, &send_work);
}

/* すぐ送る分: 歩調合わせをしないとき全部、するときはクリックと動き出しの 1 件 */
static void send_work_handler(struct k_work *work) {
    struct bt_conn *conn = get_conn();
    if (conn == NULL) {
        return;
    }

    if (!pace_enabled()) {
        while (try_send_head(conn)) {
        }
        bt_conn_unref(conn);
        return;
    }

    while (true) {
        k_spinlock_key_t key = k_spin_lock(&lock);
        bool edge = edges_in_ring > 0;
        bool start = !edge && ring_count > 0 && !pacing;
        bool send_now = edge || (start && in_flight == 0);
        if (start) {
            /* 動き出し: 最初の 1 件はすぐ送り、そこから歩調を取り始める */
            start_pacing_locked();
        }
        k_spin_unlock(&lock, key);

        if (!send_now || !try_send_head(conn)) {
            break;
        }
        if (start) {
            break;
        }
    }

    bt_conn_unref(conn);
}

/* 歩調に合わせたまとめ送り: 受け取りの少し前に、それまでの動きを 1 件で送る */
static void flush_work_handler(struct k_work *work) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    bool active = pacing;
    k_spin_unlock(&lock, key);
    if (!active) {
        return;
    }

    struct bt_conn *conn = get_conn();
    if (conn == NULL) {
        return;
    }

    if (!pace_enabled()) {
        /* 接続間隔が短くなった: 歩調合わせをやめて、すぐ送る */
        key = k_spin_lock(&lock);
        pacing = false;
        k_spin_unlock(&lock, key);
        while (try_send_head(conn)) {
        }
        bt_conn_unref(conn);
        return;
    }

    key = k_spin_lock(&lock);

    if (!pacing) {
        /* get_conn() で接続先の変化などを検出して取り直しになった */
        k_spin_unlock(&lock, key);
        bt_conn_unref(conn);
        return;
    }

    bool has_data = ring_count > 0;
    if (has_data) {
        idle_flushes = 0;
    } else {
        idle_flushes++;
    }

    if (idle_flushes > PACE_IDLE_EVENTS) {
        /* しばらく動きがない: 歩調合わせをやめる。次の動き出しで取り直す */
        pacing = false;
        phase_locked = false;
    } else {
        /* 次の受け取りの分を予約する。送信完了の通知が来れば、そこで微調整する */
        int64_t now = now_us();
        int64_t next = flush_target_us + interval_us;
        if (next <= now) {
            next = now + interval_us;
        }
        flush_target_us = next;
        k_work_reschedule_for_queue(&hog_work_q, &flush_work, K_USEC(next - now));
    }

    k_spin_unlock(&lock, key);

    if (has_data) {
        while (try_send_head(conn)) {
        }
    }

    bt_conn_unref(conn);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    if (conn == flight_conn) {
        drop_pending_locked();
        reset_flight_locked(NULL, k_uptime_get());
    }
    k_spin_unlock(&lock, key);
}

BT_CONN_CB_DEFINE(ble_mouse_coalesce_conn_cb) = {
    .disconnected = on_disconnected,
};
