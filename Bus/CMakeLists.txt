idf_component_register(SRCS "I2C/I2c_ctrl.c" Ble/ble_gatt.c  # 根据实际源文件名调整
                       INCLUDE_DIRS "include" 
                       INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/utils/include"
                       REQUIRES driver
                       nvs_flash
                       bt)