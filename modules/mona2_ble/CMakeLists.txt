# moNa2 用の BLE まわりの差し替え (ZMK v0.3 用)
# どちらもリンカの --wrap で ZMK / Zephyr の関数の呼び出しを __wrap_ 版に
# 差し替える。差し替えが効いていないと __real_ が未定義でリンクエラーになる。
if(CONFIG_MONA2_BLE_MOUSE_COALESCE OR CONFIG_MONA2_BLE_ADV_APPEARANCE)
  zephyr_library()
  zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
endif()

# BLE マウスレポートを「並べる」代わりに「まとめる」
if(CONFIG_MONA2_BLE_MOUSE_COALESCE)
  zephyr_library_sources(ble_mouse_coalesce.c)
  zephyr_link_libraries(-Wl,--wrap=zmk_hog_send_mouse_report)
endif()

# 広告データの Appearance を CONFIG_BT_DEVICE_APPEARANCE に合わせる
if(CONFIG_MONA2_BLE_ADV_APPEARANCE)
  zephyr_library_sources(ble_adv_appearance.c)
  zephyr_link_libraries(-Wl,--wrap=bt_le_adv_start)
endif()
