
set(COM_RQUIRES mcommon mwifi mlink mdebug light_driver light_handle mesh_utils aliyun_sdk bh1750 unity WS2182)

set(COM_SRCS "EnergysESP32Firmware.c" "./EnergysESP32Firmware.c")

idf_component_register(SRCS ${COM_SRCS}
                    INCLUDE_DIRS ""
                    REQUIRES ${COM_REQUIRES}
)
