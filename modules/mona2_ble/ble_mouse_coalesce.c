/*
 * BLE マウスレポートを「並べる」代わりに「まとめる」送信処理 (ZMK v0.3 用)
 *
 * ZMK 標準の送信処理は、マウスのレポートを 1 件ずつ待ち行列
 * (zmk_hog_mouse_msgq) と Bluetooth スタックのバッファに積んでいく。
 * Mac が受け取るペース (macOS では接続間隔 15ms) がセンサーの出すペース
 * (8ms) に追いつかないと行列が伸び続け、カーソルが古い動きを時間差で
 * なぞる (= 遅れてついてくる)。
 *
 * ここでは zmk_hog_send_mouse_report() をリンカの --wrap で差し替え、
 * Bluetooth スタックに渡すのを最大 MAX_IN_FLIGHT 件に抑える。送れない間に
 * 来た動きは、まだ送っていない最新のレポートに足し込む。動きは捨てず、
 * 古い動きの後ろに並ぶこともない。回線が遅いときは、届く回数が減る代わりに
 * 1 回の移動量が大きくなる。
 *
 * クリック (ボタン状態の変化) を含むレポートには後から来た動きを足さない。
 * 足すとクリック位置がずれるため。
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
static zmk_mouse_button_flags_t last_buttons;

/* Bluetooth スタックに渡して、まだ送信完了の通知が来ていない件数 */
static uint8_t in_flight;
/* 接続が切れたら進める。古い送信完了の通知で in_flight を減らさないため */
static uint32_t generation;
static int64_t last_progress_ms;
static const struct bt_conn *flight_conn;

/* 0 = 未確認, 1 = 想定どおり, -1 = 想定外 (標準の送信処理を使う) */
static int8_t attr_state;

static void send_work_handler(struct k_work *work);
static K_WORK_DEFINE(send_work, send_work_handler);

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

static void drop_pending_locked(void) {
    ring_head = 0;
    ring_count = 0;
}

static void reset_flight_locked(const struct bt_conn *conn, int64_t now) {
    in_flight = 0;
    generation++;
    flight_conn = conn;
    last_progress_ms = now;
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
            ring_head = (ring_head + 1) % SLOTS;
            ring_count--;
        }
        struct pending_report *slot = &ring[(ring_head + ring_count) % SLOTS];
        slot->body = *body;
        slot->button_edge = edge;
        ring_count++;
    }

    k_spin_unlock(&lock, key);

    k_work_submit_to_queue(&hog_work_q, &send_work);
    return 0;
}

static void notify_sent(struct bt_conn *conn, void *user_data) {
    uint32_t gen = POINTER_TO_UINT(user_data);

    k_spinlock_key_t key = k_spin_lock(&lock);
    if (gen == generation && in_flight > 0) {
        in_flight--;
    }
    last_progress_ms = k_uptime_get();
    k_spin_unlock(&lock, key);

    k_work_submit_to_queue(&hog_work_q, &send_work);
}

static void send_work_handler(struct k_work *work) {
    while (true) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        int64_t now = k_uptime_get();
        k_spinlock_key_t key = k_spin_lock(&lock);

        if (conn == NULL) {
            /* 未接続。つながり直したときに古い動きを再生しないよう捨てる */
            drop_pending_locked();
            reset_flight_locked(NULL, now);
            k_spin_unlock(&lock, key);
            return;
        }

        if (conn != flight_conn) {
            /* 接続先 (プロファイル) が変わった */
            reset_flight_locked(conn, now);
        } else if (in_flight >= MAX_IN_FLIGHT && now - last_progress_ms > STALL_MS) {
            /* 送信完了の通知が来ないまま止まっている。待ち続けない */
            LOG_WRN("Mouse notifications stalled, resetting");
            reset_flight_locked(conn, now);
        }

        if (ring_count == 0 || in_flight >= MAX_IN_FLIGHT) {
            k_spin_unlock(&lock, key);
            bt_conn_unref(conn);
            return;
        }

        struct zmk_hid_mouse_report_body report = ring[ring_head].body;
        ring_head = (ring_head + 1) % SLOTS;
        ring_count--;
        in_flight++;
        last_progress_ms = now;
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

        bt_conn_unref(conn);
    }
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
