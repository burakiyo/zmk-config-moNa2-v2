# BLE マウスレポートを「並べる」代わりに「まとめる」送信処理 (ZMK v0.3 用)
if(CONFIG_MONA2_BLE_MOUSE_COALESCE)
  zephyr_library()
  zephyr_library_sources(ble_mouse_coalesce.c)
  zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)

  # ZMK の zmk_hog_send_mouse_report() の呼び出しを __wrap_ 版に差し替える
  zephyr_link_libraries(-Wl,--wrap=zmk_hog_send_mouse_report)
endif()
