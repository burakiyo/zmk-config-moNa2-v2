/*
 * 広告データの Appearance を CONFIG_BT_DEVICE_APPEARANCE に合わせる (ZMK v0.3 用)
 *
 * ZMK v0.3 は広告データの Appearance を 0x03C1 (キーボード) に固定している
 * (app/src/ble.c の zmk_ble_ad)。CONFIG_BT_DEVICE_APPEARANCE を変えても、
 * 接続後に読まれる GAP の値しか変わらない。macOS は Appearance を見て
 * 低遅延の接続にするかを決める (bluetoothd の isLowLatencyBLEHID) ので、
 * 広告を始めるときに Appearance だけ設定値に差し替える。
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/util.h>

int __real_bt_le_adv_start(const struct bt_le_adv_param *param, const struct bt_data *ad,
                           size_t ad_len, const struct bt_data *sd, size_t sd_len);

static const uint8_t appearance[] = {
    CONFIG_BT_DEVICE_APPEARANCE & 0xff,
    (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff,
};

static struct bt_data patched_ad[8];

int __wrap_bt_le_adv_start(const struct bt_le_adv_param *param, const struct bt_data *ad,
                           size_t ad_len, const struct bt_data *sd, size_t sd_len) {
    if (ad == NULL || ad_len > ARRAY_SIZE(patched_ad)) {
        return __real_bt_le_adv_start(param, ad, ad_len, sd, sd_len);
    }

    for (size_t i = 0; i < ad_len; i++) {
        patched_ad[i] = ad[i];
        if (ad[i].type == BT_DATA_GAP_APPEARANCE && ad[i].data_len == sizeof(appearance)) {
            patched_ad[i].data = appearance;
        }
    }

    return __real_bt_le_adv_start(param, patched_ad, ad_len, sd, sd_len);
}
