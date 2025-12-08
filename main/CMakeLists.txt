idf_component_register(SRCS "app_task.c" "app_main.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include"
                       INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components/oled/include"
                       INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components/gSensor/include")
