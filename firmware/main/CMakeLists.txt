idf_component_register(SRCS "main.c"
                       REQUIRES esp_phy esp_wifi nvs_flash esp_timer driver esp_event)
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,-T${CMAKE_CURRENT_LIST_DIR}/rf_bank_guard.ld")
