idf_component_register(SRCS "drv_oled.c"  # 根据实际源文件名调整
                       INCLUDE_DIRS "include"
                       INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/Bus/include"
                       INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/utils/include"
                       REQUIRES driver
                       PRIV_REQUIRES Bus)