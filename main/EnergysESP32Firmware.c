
/**
 * @file light_example.c
 * @author Samuel Mok
 * @brief Firmware for ESP32 IC, PIR senor AS312, LUX meter BH1750
 *        function: wifi mesh, wifi, ble, http web server
 * @version 1.0
 * @date 2021-09-01
 * 
 * @copyright Energys Group HK Copyright (c) 2021
 * 
 */

//for the mdf library
#include "mdf_common.h"
#include "mwifi.h"
#include "mlink.h"
#include "mupgrade.h"
//for the mdf config
#include "mespnow.h"
#include "mconfig_blufi.h"
#include "mconfig_chain.h"
#include "mdebug_console.h"
#include "mdebug_log.h"
//for the light driver and handler
#include "light_driver.h"
#include "light_handle.h"
//for the basic function of the mesh wifi, time.....
#include "mesh_utils.h"
//for the mqtt handler
#include "mesh_mqtt_handle.h"
//for the bh1750
#include "esp_log.h"
#include "driver/i2c.h"
#include "bh1750.h"
#include "unity.h"
//for the sdkconfig definiation value
#include "sdkconfig.h"
//for the AS312
#include "driver/gpio.h"
//for the WS2182
#include "led_strip.h"
#include "driver/rmt.h"
//for the button
#include "button.h"

#define LIGHT_TID                     (1)
#define LIGHT_NAME                    "light"
#define LIGHT_RESTART_COUNT_FALLBACK  CONFIG_LIGHT_RESTART_COUNT_FALLBACK
#define LIGHT_RESTART_COUNT_RESET     CONFIG_LIGHT_RESTART_COUNT_RESET


/**
 * @brief for bh1750 definiation
 */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static bh1750_handle_t bh1750 = NULL;
static TaskHandle_t BH1750_task_handle = NULL;
static void bh1750_task(void *arg);
/**
 * @brief for AS312 definiation 
 * 
 */
static TaskHandle_t AS312_task_handle = NULL;
void AS312_task(void *arg);

/**
 * @brief for sensor mqtt task definiation 
 * 
 */
static TaskHandle_t sensor_mqtt_task_handle = NULL;
void sensor_mqtt_update_task(void *arg); 

/**
 * @brief label for the LOG TAG 
 * 
 */
static const char *TAG                       = "EnergysESP32FirmwareV1";
static const char *TAG_BH1750                = "BH1750";
static const char *TAG_AS312                 = "AS312";
static const char *TAG_WS2182                = "WS2182";
static const char *TAG_button                = "button";

esp_netif_t *sta_netif;
static TaskHandle_t g_root_write_task_handle = NULL;
static TaskHandle_t g_root_read_task_handle  = NULL;
static bool g_config_from_blufi_flag         = false;

/**
 * @brief controller flag for the onbroad led 
 * 
 */
static bool led_flag = true;

/**
 * @brief controller flag for the sensor 
 * 
 */
static bool sensor_flag = true;

/**
 * @brief for the led strip 
 * 
 */
static led_strip_t *strip;

/**
 * @brief for the button control: true= AS312 task will send on off message. default is true
 * 
 */
static bool control_flag =true;

#define NAV_PROTO_BYTES 100
#define NAV_PROTO_JSON 101

/**
 * @brief for passing value between task
 * 
 */
QueueHandle_t queue;

/**
 * @brief Read data from mesh network, forward data to extern IP network by http or udp.
 */
static void root_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data    = NULL;
    size_t size   = 0;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t mwifi_type      = {0};

    MDF_LOGI("root_write_task is running");

    while (esp_mesh_get_layer() == MESH_ROOT) {
        
        ret = mwifi_root_read(src_addr, &mwifi_type, &data, &size, portMAX_DELAY);
        MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mwifi_root_read", mdf_err_to_name(ret));

        //for writing msg to mqtt
        //example:
        //{"addr":"94b97eada080","type":"json","data":{"type":"heartbeat", "self": "94b97eada080", "parent":"94b97eacc649","layer":2}}
        //{"addr":"94b97eada080","type":"json","data":{"scr_mac" :94:b9:7e:ad:a0:80, "type" :"remote_sensor","data" :264.166656}}
        //

        //cJSON_Object();
        cJSON *name_json = cJSON_GetObjectItem(data, "type");
        if (name_json != NULL){
            MDF_LOGD("this is not null");
        }

        ret = mesh_mqtt_write(src_addr, data, size, MESH_MQTT_ENDNODE_READ);
        MDF_LOGI("the src_addr in root write task is %02x%02x%02x%02x%02x%02x",MAC2STR(src_addr));
        MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mesh_mqtt_publish", mdf_err_to_name(ret));
        
        if (mwifi_type.upgrade) {
            ret = mupgrade_root_handle(src_addr, data, size);
            MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mupgrade_handle", mdf_err_to_name(ret));
            goto FREE_MEM;
        }

        MDF_LOGD("Root receive, addr: " MACSTR ", size: %d, data: %.*s",
                 MAC2STR(src_addr), size, size, data);
        
        //char *json_string = "{\"scr_mac\" :94:b9:7e:ac:c6:48, \"type\" :\"remote_sensor\",\"data\" :340.833344}";
        //str[82] = "{\"scr_mac\" :94:b9:7e:ac:c6:48, \"type\" :\"remote_sensor\",\"data\" :340.833344}";

        // cJSON *json_root = cJSON_Parse(str);

        // if (data == NULL){
        //     MDF_LOGD("data is null.");
        // }

        // if (json_root == NULL){
        //     MDF_LOGD("parse fail. lala");
        // }



        //MDF_ERROR_CONTINUE(!json_root, "cJSON_Parse, data format error"); 
        //don't use this

        
        // if(cJSON_GetObjectItem(json_root, "type")){
        //     MDF_LOGD("lalalala this is remote sensor data");
        //      uint8_t *sta_mac = cJSON_GetObjectItem(json_root, "scr_mac");
        //      float *bh1750_data = cJSON_GetObjectItem(json_root,"data");

        //      MDF_LOGD("sta_mac %d, bh1750 data %f", *sta_mac, *bh1750_data);
        // }

        switch (mwifi_type.protocol) {
            case MLINK_PROTO_HTTPD: { // use http protocol
                mlink_httpd_t httpd_data  = {
                    .size       = size,
                    .data       = data,
                    .addrs_num  = 1,
                    .addrs_list = src_addr,
                };
                memcpy(&httpd_data.type, &mwifi_type.custom, sizeof(httpd_data.type));

                ret = mlink_httpd_write(&httpd_data, portMAX_DELAY);
                MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_httpd_write", mdf_err_to_name(ret));

                break;
            }

            case MLINK_PROTO_NOTICE: { // use udp protocol
                ret = mlink_notice_write(data, size, src_addr);
                MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_httpd_write", mdf_err_to_name(ret));
                break;
            }

            // case NAV_PROTO_BYTES:{
            //     ret = mesh_mqtt_write(src_addr, data, size, MESH_MQTT_DATA_BYTES);
            //     break;
            // }

            // case NAV_PROTO_JSON:{
            //     MDF_LOGD("laalalal I am here NAV_PROTO_JSON");
            //     ret = mesh_mqtt_write(src_addr, data, size, MESH_MQTT_DATA_JSON);
            //     break;
            // }

            default:
                MDF_LOGW("Does not support the protocol: %d", mwifi_type.protocol);
                break;
        }

FREE_MEM:
        MDF_FREE(data);
    }

    MDF_LOGW("root_write_task is exit");

    MDF_FREE(data);
    g_root_write_task_handle = NULL;
    //for stopping the mqtt task
    mesh_mqtt_stop();
    vTaskDelete(NULL);
}

/**
 * @brief Read data from extern IP network, forward data to destination device.
 */
static void root_read_task(void *arg)
{
    mdf_err_t ret               = MDF_OK;
    mlink_httpd_t *httpd_data   = NULL;
    mwifi_data_type_t mwifi_type = {
        .compression = true,
        .communicate = MWIFI_COMMUNICATE_MULTICAST,
    };

    MDF_LOGI("root_read_task is running");

    while (esp_mesh_get_layer() == MESH_ROOT) {
        ret = mlink_httpd_read(&httpd_data, portMAX_DELAY);
        MDF_ERROR_GOTO(ret != MDF_OK || !httpd_data, FREE_MEM, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        MDF_LOGD("Root send, addrs_num: %d, addrs_list: " MACSTR ", size: %d, data: %.*s",
                 httpd_data->addrs_num, MAC2STR(httpd_data->addrs_list),
                 httpd_data->size, httpd_data->size, httpd_data->data);

        mwifi_type.group = httpd_data->group;
        memcpy(&mwifi_type.custom, &httpd_data->type, sizeof(mlink_httpd_type_t));

        ret = mwifi_root_write(httpd_data->addrs_list, httpd_data->addrs_num,
                               &mwifi_type, httpd_data->data, httpd_data->size, true);
        MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mwifi_root_write", mdf_err_to_name(ret));
        
        //for the mqtt recv data

        mesh_mqtt_data_t *request = NULL;
        mwifi_data_type_t data_type = { 0x0 };

        /**
         * @brief Recv data from mqtt data queue, and forward to special device.
         */
        ret = mesh_mqtt_read(&request, pdMS_TO_TICKS(500));

        if (ret != MDF_OK) {
            continue;
        }

        ret = mwifi_root_write(request->addrs_list, request->addrs_num, &data_type, request->data, request->size, true);
        MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mwifi_root_write", mdf_err_to_name(ret));


FREE_MEM:

        if (httpd_data) {
            MDF_FREE(httpd_data->addrs_list);
            MDF_FREE(httpd_data->data);
            MDF_FREE(httpd_data);
        }
        //for freeing the MQTT mem
        if (request){
            MDF_FREE(request->addrs_list);
            MDF_FREE(request->data);
            MDF_FREE(request);
        }
    }

    MDF_LOGW("root_read_task is exit");

    if (httpd_data) {
        MDF_FREE(httpd_data->addrs_list);
        MDF_FREE(httpd_data->data);
        MDF_FREE(httpd_data);
    }

    g_root_read_task_handle = NULL;
    //stop the mqtt
    mesh_mqtt_stop();
    vTaskDelete(NULL);
}

static void delay_func_timer_cb(void *timer)
{
    char *data = (char *)pvTimerGetTimerID(timer);

    mlink_handle_data_t handle_data = {
        .req_data    = data,
        .req_size    = strlen(data),
        .req_fromat  = MLINK_HTTPD_FORMAT_JSON,
        .resp_data   = NULL,
        .resp_size   = 0,
        .resp_fromat = MLINK_HTTPD_FORMAT_JSON,
    };

    if (mlink_handle_request(&handle_data) != MDF_OK) {
        MDF_LOGW("Call the handler in the request list");
    }

    MDF_FREE(handle_data.resp_data);
    MDF_FREE(data);
    xTimerStop(timer, 0);
    xTimerDelete(timer, 0);
}

static mdf_err_t delay_func_call(char *data, TickType_t wait_ticks)
{
    MDF_PARAM_CHECK(data);

    MDF_LOGD("tsf_time: %lld, wait_ticks: %d, delay_data: %s", esp_mesh_get_tsf_time(), wait_ticks, data);
    TimerHandle_t timer = xTimerCreate("delay_func", wait_ticks, false, data, delay_func_timer_cb);
    xTimerStart(timer, 0);

    return MDF_OK;
}

/**
 * @brief Handling data between wifi mesh devices.
 */
 #include <string.h>
void node_handle_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    uint8_t *data = NULL;
    size_t size   = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t mwifi_type     = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mlink_httpd_type_t *header_info  = NULL;

    while (true) {
        ret = mwifi_read(src_addr, &mwifi_type, &data, &size, portMAX_DELAY);
        MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> Receive a packet targeted to self over the mesh network",
                       mdf_err_to_name(ret));

        if (mwifi_type.upgrade) { // This mesh package contains upgrade data.
            ret = mupgrade_handle(src_addr, data, size);
            MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mupgrade_handle", mdf_err_to_name(ret));
            goto FREE_MEM;
        }

        //print out the data received
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
        MDF_LOGI("Node receive, addr: " MACSTR ", size: %d, data: %.*s", MAC2STR(src_addr), size, size, data);


        /*< Header information for http data */
        header_info = (mlink_httpd_type_t *)&mwifi_type.custom;
        MDF_ERROR_GOTO(header_info->format != MLINK_HTTPD_FORMAT_JSON, FREE_MEM,
                       "The current version only supports the json protocol");

        //for the button request this one is only for the controlling the task
        MDF_LOGI("checking recevied message");
        char value[1] ={0x0}; 
        //int control_value =0;
        if (mlink_json_parse((char *)data, "control", value)==MDF_OK){
        //if (strcmp(&data, "{\"request\": \"set_status\",\"control\":\"button\",\"characteristics\": [{\"cid\": 0,\"value\": 1}]}") == 0) {
            if (strcmp(value, "1" )==0){
                    MDF_LOGI("bh1750 and AS312 task resume %s", value);
                    //check if the sensor is attached first
                    if (sensor_flag == true){
#ifdef CONFIG_BH1750_ENABLE
                        vTaskResume(BH1750_task_handle);
                        //xTaskCreate(bh1750_task, "bh1750_task", 4 * 1024, 
                        //    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, &BH1750_task_handle);
#endif
#ifdef CONFIG_AS312_ENABLE
                        vTaskResume(AS312_task_handle);
                        //xTaskCreate(AS312_task, "AS312_task", 4 * 1024, 
                        //    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, &AS312_task_handle);            
#endif
                    } else {
                        MDF_LOGI("No sensor detected");
                    }
            }
            else if (strcmp(value, "0")==0){
            //else if (strcmp(&data, "{\"request\": \"set_status\",\"control\":\"button\",\"characteristics\": [{\"cid\": 0,\"value\": 0}]}") == 0) {

                    MDF_LOGI("bh1750 and AS312 task suspend %s", value);
                    //check if the sensor is attached first
                    if (sensor_flag == true){
#ifdef CONFIG_BH1750_ENABLE
                        //vTaskDelete(BH1750_task_handle);
                        vTaskSuspend(BH1750_task_handle);
#endif
#ifdef CONFIG_AS312_ENABLE
                        //vTaskDelete (AS312_task_handle);
                        vTaskSuspend(AS312_task_handle);
#endif
                    } else {
                        MDF_LOGI("No sensor detected");
                    }

            }
        } else {
            MDF_LOGI("Cannot determine the message");
        }


        /**
         * @brief Delayed call to achieve synchronous execution
         */
        char tsf_time_str[16] = {0x0};
        int64_t delay_ticks   = 0;

        if (mlink_json_parse((char *)data, "tsf_time", tsf_time_str) == MDF_OK) {
            int64_t tsf_time_us = 0;
            sscanf(tsf_time_str, "%llu", &tsf_time_us);
            delay_ticks = pdMS_TO_TICKS((tsf_time_us - esp_mesh_get_tsf_time()) / 1000);
            MDF_LOGD("delay_ticks: %lld ms", delay_ticks);
        }

        if (delay_ticks > 0 && delay_ticks < 60 * 1000) {
            char *delay_data = MDF_CALLOC(1, size + 1);
            memcpy(delay_data, data, size);
            delay_func_call(delay_data, delay_ticks);
            goto FREE_MEM;
        }

        /**
         * @brief Processing request commands, generating response data
         *
         * @note  Handling only the body part of http, the header
         *        of http is handled by mlink_httpd
         */
        mlink_handle_data_t handle_data = {
            .req_data    = (char *)data,
            .req_size    = size,
            .req_fromat  = MLINK_HTTPD_FORMAT_JSON,
            .resp_data   = NULL,
            .resp_size   = 0,
            .resp_fromat = MLINK_HTTPD_FORMAT_JSON,
        };
        ret = mlink_handle_request(&handle_data);
        MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mlink_handle", mdf_err_to_name(ret));

        if (handle_data.resp_fromat == MLINK_HTTPD_FORMAT_JSON) {
            mlink_json_pack(&handle_data.resp_data, "status_msg", mdf_err_to_name(ret));
            handle_data.resp_size = mlink_json_pack(&handle_data.resp_data, "status_code", -ret);
        }

        /**
         * @brief If this packet comes from a device on the mesh network,
         *  it will notify the App that the device's status has changed.
         */
        if (header_info->from == MLINK_HTTPD_FROM_DEVICE && mwifi_get_root_status()) {
            mwifi_type.protocol = MLINK_PROTO_NOTICE;
            ret = mwifi_write(NULL, &mwifi_type, "status", strlen("status"), true);
            MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mlink_handle", mdf_err_to_name(ret));
        }

        /**
         * @brief Send the response data to the source device
         */
        if (header_info->resp) {
            uint8_t *dest_addr = (header_info->from == MLINK_HTTPD_FROM_SERVER) ? NULL : src_addr;
            /*< Populate the header information of http */
            header_info->format = handle_data.resp_fromat;
            header_info->from   = MLINK_HTTPD_FROM_DEVICE;

            mwifi_type.protocol = MLINK_PROTO_HTTPD;
            mwifi_type.compression = true;
            ret = mwifi_write(dest_addr, &mwifi_type, handle_data.resp_data, handle_data.resp_size, true);

            if (handle_data.resp_fromat == MLINK_HTTPD_FORMAT_HEX) {
                MDF_LOGI("Node send, size: %d, data: ", handle_data.resp_size);
                ESP_LOG_BUFFER_HEX(TAG, handle_data.resp_data, handle_data.resp_size);
            } else {
                MDF_LOGI("Node send, size: %d, data: %.*s", handle_data.resp_size,
                         handle_data.resp_size, handle_data.resp_data);
            }
        }

        MDF_FREE(handle_data.resp_data);
        MDF_ERROR_GOTO(ret != ESP_OK, FREE_MEM, "<%s> mdf_write", mdf_err_to_name(ret));
        
FREE_MEM:
        MDF_FREE(data);
    }

    MDF_FREE(data);
    vTaskDelete(NULL);
}

/**
 * @brief MQTT node write task to server
 */


static void node_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    size_t size = 0;
    char *data = NULL;
    mwifi_data_type_t data_type = { 0x0 };
    uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
    mesh_addr_t parent_mac = { 0 };

    MDF_LOGI("Node task is running");

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    for (;;) {
        if (!mwifi_is_connected() || !mwifi_get_root_status()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        /**
         * @brief Send device information to mqtt server throught root node.
         */
        esp_mesh_get_parent_bssid(&parent_mac);
        size = asprintf(&data, "{\"type\":\"heartbeat\", \"self\": \"%02x%02x%02x%02x%02x%02x\", \"parent\":\"%02x%02x%02x%02x%02x%02x\",\"layer\":%d}",
                        MAC2STR(sta_mac), MAC2STR(parent_mac.addr), esp_mesh_get_layer());

        MDF_LOGD("Node send, size: %d, data: %s", size, data);
        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_FREE(data);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));

        //the heartbeat will send every 6s
        vTaskDelay(3000 / portTICK_RATE_MS);
    }

    MDF_LOGW("Node task is exit");
    vTaskDelete(NULL);
}



/**
 * @brief Initialize espnow_to_mwifi_task for forward esp-now data to the wifi mesh network.
 */
static void espnow_to_mwifi_task(void *arg)
{
    mdf_err_t ret       = MDF_OK;
    uint8_t *data       = NULL;
    uint8_t *addrs_list = NULL;
    size_t addrs_num    = 0;
    size_t size         = 0;
    uint32_t type       = 0;

    mwifi_data_type_t mwifi_type = {
        .protocol = MLINK_PROTO_HTTPD,
    };

    mlink_httpd_type_t header_info = {
        .format = MLINK_HTTPD_FORMAT_JSON,
        .from   = MLINK_HTTPD_FROM_DEVICE,
        .resp   = false,
    };

    memcpy(&mwifi_type.custom, &header_info, sizeof(mlink_httpd_type_t));

    while (mlink_espnow_read(&addrs_list, &addrs_num, &data, &size, &type, portMAX_DELAY) == MDF_OK) {
        /*< Send to yourself if the destination address is empty */
        if (MWIFI_ADDR_IS_EMPTY(addrs_list) && addrs_num == 1) {
            esp_wifi_get_mac(ESP_IF_WIFI_STA, addrs_list);
        }

        mwifi_type.group = (type == MLINK_ESPNOW_COMMUNICATE_GROUP) ? true : false;
        MDF_LOGI("Mlink espnow read data: %.*s", size, data);

        for (int i = 0; i < addrs_num; ++i) {
            ret = mwifi_write(addrs_list  + 6 * i, &mwifi_type, data, size, true);
            MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
        }

        MDF_FREE(data);
        MDF_FREE(addrs_list);
    }

    MDF_LOGW("espnow_to_mwifi_task is exit");
    vTaskDelete(NULL);
}

#ifdef CONFIG_LIGHT_BLE_GATEWAY
mdf_err_t mlink_ble_write(void *data, size_t size)
{
    mdf_err_t ret         = MDF_OK;
    char *header          = NULL;
    char *body            = NULL;
    char **addr_list_json = NULL;
    uint32_t addr_list_num  = 0;
    mwifi_data_type_t mwifi_type = {
        .protocol = MLINK_PROTO_HTTPD,
    };
    mlink_httpd_type_t header_info = {
        .format = MLINK_HTTPD_FORMAT_JSON,
        .from   = MLINK_HTTPD_FROM_DEVICE,
        .resp   = false,
    };

    memcpy(&mwifi_type.custom, &header_info, sizeof(mlink_httpd_type_t));

    ret = mlink_json_parse(data, "header", &header);
    MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "Parse the json formatted string: header");
    ret = mlink_json_parse(data, "body", &body);
    MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "Parse the json formatted string: header");

    if (mlink_json_parse(header, "group", &addr_list_num) == MDF_OK && addr_list_num > 0) {
        mwifi_type.group = true;
        addr_list_json = MDF_CALLOC(addr_list_num, sizeof(char *));
        ret = mlink_json_parse(header, "group", addr_list_json);
        MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "Parse the json formatted string: group");
    } else if (mlink_json_parse(header, "addr", &addr_list_num) == MDF_OK && addr_list_num > 0) {
        addr_list_json = MDF_CALLOC(addr_list_num, sizeof(char *));
        ret = mlink_json_parse(header, "addr", addr_list_json);
        MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "Parse the json formatted string: addr");
    } else {
        MDF_LOGW("Data format error");
        ret = MDF_ERR_NOT_SUPPORTED;
        goto EXIT;
    }

    MDF_LOGI("addr_num: %d, headr: %s, body: %s", addr_list_num, header, body);

    for (int i = 0; i < addr_list_num; ++i) {
        uint8_t addr[6] = {0x0};
        mlink_mac_str2hex(addr_list_json[i], addr);
        MDF_FREE(addr_list_json[i]);

        ret = mwifi_write(addr, &mwifi_type, body, strlen(body), true);
        MDF_ERROR_GOTO(ret != MDF_OK, EXIT, "<%s> mwifi_write", mdf_err_to_name(ret));
    }

EXIT:

    MDF_FREE(header);
    MDF_FREE(body);
    MDF_FREE(addr_list_json);
    return MDF_OK;
}
#endif /**< CONFIG_LIGHT_BLE_GATEWAY */

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: 0x%x", event);
    mdf_err_t ret = MDF_OK;

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");

            if (esp_mesh_is_root()) {
                esp_netif_dhcpc_start(sta_netif);
            }

            light_driver_breath_stop();

#ifdef CONFIG_LIGHT_NETWORKING_TIME_OPTIMIZE_ENABLE

            if (esp_mesh_is_root_fixed()) {
                /**
                 * TODO：Fix the problem that esp_mesh does not update at the bottom,
                 *       IE does not update. This is a temporary solution. This code
                 *       needs to be deleted after esp-idf is fixed.
                 */
                extern mesh_assoc_t g_mesh_ie;
                g_mesh_ie.rc_rssi = mwifi_get_parent_rssi();

                esp_mesh_fix_root(false);
                ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, false));
            }

            mwifi_node_type_t mesh_type = MWIFI_MESH_IDLE;

            if (esp_mesh_is_root()) {
                mesh_type = MWIFI_MESH_ROOT;
            } else {
                mesh_type = MWIFI_MESH_IDLE;
            }

            mwifi_config_t ap_config  = {0x0};
            mdf_info_load("ap_config", &ap_config, sizeof(mwifi_config_t));

            if (esp_mesh_get_type() != MESH_LEAF && esp_mesh_get_layer() == CONFIG_MWIFI_MAX_LAYER) {
                ESP_ERROR_CHECK(esp_mesh_set_type(MESH_LEAF));
            }

            if (ap_config.mesh_type != mesh_type) {
                ap_config.mesh_type = mesh_type;
                mdf_info_save("ap_config", &ap_config, sizeof(mwifi_config_t));
            }

#endif /**< CONFIG_LIGHT_NETWORKING_TIME_OPTIMIZE_ENABLE */

#ifdef CONFIG_LIGHT_BLE_GATEWAY

            if (!esp_mesh_is_root()) {
                mlink_ble_config_t config = {
                    .company_id = MCOMMON_ESPRESSIF_ID,
                    /**
                     * @brief  This custom_data is for iBeacon definitions. https://developer.apple.com/ibeacon/
                     *         Espressif WeChat official account can be found using WeChat "Yao Yi Yao Zhou Bian"
                     */
                    .custom_data = {
                        0x02, 0x01, 0x06, 0x1A, 0xFF, 0x4C, 0x00, 0x02,
                        0x15, 0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F,
                        0xB1, 0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78,
                        0x25, 0x27, 0xB7, 0xF2, 0x06, 0xC5
                    },
                    .custom_size = 30,
                };

                memcpy(config.name, mlink_device_get_name(), sizeof(config.name) - 1);
                MDF_ERROR_ASSERT(mlink_ble_init(&config));
                mlink_ble_set_cb(NULL, mlink_ble_write);
            } else {
                MDF_ERROR_ASSERT(mlink_ble_deinit());
            }

#endif /**< CONFIG_LIGHT_BLE_GATEWAY */

            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");

            if (esp_mesh_is_root()) {
                ret = mwifi_post_root_status(false);
                MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mwifi_post_root_status", mdf_err_to_name(ret));
                mesh_mqtt_stop();
            }

            break;

        case MDF_EVENT_MWIFI_FIND_NETWORK: {
            MDF_LOGI("the root connects to another router with the same SSID");
            mwifi_config_t ap_config  = {0x0};
            wifi_second_chan_t second = 0;

            mdf_info_load("ap_config", &ap_config, sizeof(mwifi_config_t));
            esp_wifi_get_channel(&ap_config.channel, &second);
            esp_mesh_get_parent_bssid((mesh_addr_t *)ap_config.router_bssid);
            mwifi_set_config(&ap_config);
            mdf_info_save("ap_config", &ap_config, sizeof(mwifi_config_t));
            break;
        }

        case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
        case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE: {
            MDF_LOGI("total_num: %d", esp_mesh_get_total_node_num());

            if (esp_mesh_is_root()) {
                uint8_t sta_mac[MWIFI_ADDR_LEN] = {0x0};
                MDF_ERROR_ASSERT(esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac));

                ret = mlink_notice_write("http", strlen("http"), sta_mac);
                MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_httpd_write", mdf_err_to_name(ret));
            }
            
            // for update topo and publish to mqtt
            if (esp_mesh_is_root() && mwifi_get_root_status()) {
                mdf_err_t err = mesh_mqtt_update_topo();

                if (err != MDF_OK) {
                    MDF_LOGE("Update topo failed");
                }
            }

            break;
        }


        case MDF_EVENT_MWIFI_ROOT_GOT_IP: {
            MDF_LOGI("Root obtains the IP address");

            /**
             * @brief Initialization mlink notice for inform the mobile phone that there is a mesh root device
             */
            ret = mlink_notice_init();
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_notice_init", mdf_err_to_name(ret));

            uint8_t sta_mac[MWIFI_ADDR_LEN] = {0x0};
            MDF_ERROR_ASSERT(esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac));

            ret = mlink_notice_write("http", strlen("http"), sta_mac);
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_httpd_write", mdf_err_to_name(ret));

            /**
             * @brief start mlink http server for handle data between device and moblie phone.
             */
            ret = mlink_httpd_start();
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_httpd_start", mdf_err_to_name(ret));
            
            //for starting the mqtt
            mesh_mqtt_start(CONFIG_MQTT_URL);            
            // MDF_PARAM_CHECK(CONFIG_MQTT_URL);
            // MDF_ERROR_CHECK(g_mesh_mqtt.client != NULL, MDF_ERR_INVALID_STATE, "MQTT client is already running");

            // esp_mqtt_client_config_t mqtt_cfg = {
            //     .uri = CONFIG_MQTT_URL,
            //     .event_handle = mqtt_event_handler,
            //     //.cert_pem = (const char *)mqtt_energysmeter_com_pem_start,
            //     //.client_cert_pem = (const char *)client_cert_pem_start,
            //     //.client_key_pem = (const char *)client_key_pem_start,
            // };
            // MDF_ERROR_ASSERT(esp_read_mac(g_mesh_mqtt.addr, ESP_MAC_WIFI_STA));
            // snprintf(g_mesh_mqtt.publish_topic, sizeof(g_mesh_mqtt.publish_topic), publish_topic_template, MAC2STR(g_mesh_mqtt.addr));
            // snprintf(g_mesh_mqtt.topo_topic, sizeof(g_mesh_mqtt.topo_topic), topo_topic_template, MAC2STR(g_mesh_mqtt.addr));
            // g_mesh_mqtt.queue = xQueueCreate(3, sizeof(mesh_mqtt_data_t *));
            // g_mesh_mqtt.client = esp_mqtt_client_init(&mqtt_cfg);
            // MDF_ERROR_ASSERT(esp_mqtt_client_start(g_mesh_mqtt.client));


            /**
             * @brief start root read/write task for hand data between mesh network and extern ip network.
             */
            if (!g_root_write_task_handle) {
                xTaskCreate(root_write_task, "root_write", 6 * 1024,
                            NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, &g_root_write_task_handle);
            }

            if (!g_root_read_task_handle) {
                xTaskCreate(root_read_task, "root_read", 4 * 1024,
                            NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, &g_root_read_task_handle);
            }

            ret = mwifi_post_root_status(true);
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mwifi_post_root_status", mdf_err_to_name(ret));

            break;
        }

        case MDF_EVENT_MWIFI_ROUTER_SWITCH:
        case MDF_EVENT_MWIFI_ROOT_LOST_IP:
            /** When the root node switches, sometimes no disconnected packets are received */
            ret = mlink_notice_deinit();
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_notice_deinit", mdf_err_to_name(ret));

            ret = mlink_httpd_stop();
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_httpd_stop", mdf_err_to_name(ret));
            break;

        case MDF_EVENT_MCONFIG_BLUFI_STA_DISCONNECTED:
            light_driver_breath_start(128, 128, 0); /**< yellow blink */
            break;

        case MDF_EVENT_MCONFIG_BLUFI_STA_CONNECTED:
            light_driver_breath_start(255, 128, 0); /**< orange blink */
            break;

        case MDF_EVENT_MCONFIG_BLUFI_FINISH:
            g_config_from_blufi_flag = true;
            __attribute__((fallthrough));

        case MDF_EVENT_MCONFIG_CHAIN_FINISH:
            light_driver_breath_start(0, 255, 0); /**< green blink */
            break;

        case MDF_EVENT_MUPGRADE_STARTED:
            MDF_LOGI("Enter upgrade mode");
            light_driver_breath_start(0, 0, 128); /**< blue blink */
            vTaskDelay(pdMS_TO_TICKS(3000));
            light_driver_breath_stop();
            break;

        case MDF_EVENT_MUPGRADE_STATUS: {
            MDF_LOGI("The upgrade progress is: %d%%", (int)ctx);
            mwifi_data_type_t mwifi_type = {
                .protocol = MLINK_PROTO_NOTICE,
            };
            ret = mwifi_write(NULL, &mwifi_type, "ota_status", strlen("ota_status"), true);
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
            break;
        }

        case MDF_EVENT_MUPGRADE_FINISH:
            MDF_LOGI("Upgrade completed waiting for restart");
            light_driver_breath_start(0, 0, 255); /**< blue blink */
            break;

        case MDF_EVENT_MLINK_SYSTEM_RESET:
            MDF_LOGW("Erase information saved in flash and system restart");

            ret = mdf_info_erase(MDF_SPACE_NAME);
            MDF_ERROR_BREAK(ret != 0, "Erase the information");

            esp_restart();
            break;

        case MDF_EVENT_MLINK_SYSTEM_REBOOT:
            MDF_LOGW("Restart PRO and APP CPUs");
            esp_restart();
            break;

        case MDF_EVENT_MLINK_SET_STATUS:
            /**
             * @brief Waiting for adjacent packets to be processed, avoiding loops
             */
            vTaskDelay(pdMS_TO_TICKS(50));

            /**
             * @brief Trigger handler
             */
            ret = mlink_trigger_handle(MLINK_COMMUNICATE_MESH);
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mlink_trigger_handle", mdf_err_to_name(ret));

            break;

        case MDF_EVENT_MLINK_BUFFER_FULL: {
            MDF_LOGI("Receive data from sniffer");

            /**
             * @brief Notify the APP to actively request the sniffer data received by the device
             */
            mwifi_data_type_t mwifi_type = {
                .protocol = MLINK_PROTO_NOTICE,
            };
            ret = mwifi_write(NULL, &mwifi_type, "sniffer", strlen("sniffer"), true);
            MDF_ERROR_BREAK(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
            break;
        }

        case MDF_EVENT_CUSTOM_MQTT_CONNECTED:{
            MDF_LOGI("MQTT connect");
            mdf_err_t err = mesh_mqtt_subscribe();
            if (err != MDF_OK) {
                MDF_LOGE("Subscribe failed");
            }
            err = mesh_mqtt_update_topo();
            if (err != MDF_OK) {
                MDF_LOGE("Update topo failed");
            }
            
            mwifi_post_root_status(true);
            break;
        }

        case MDF_EVENT_CUSTOM_MQTT_DISCONNECTED:{
            MDF_LOGI("MQTT disconnected");
            mwifi_post_root_status(false);
            break;
        }

        default:
            break;
    }

    return MDF_OK;
}

// static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
// {
//     int msg_id;

//     switch (event->event_id)
//     {
//     case MQTT_EVENT_CONNECTED:
//         MDF_LOGD("MQTT_EVENT_CONNECTED");
//         g_mesh_mqtt.is_connected = true;
//         mdf_event_loop_send(MDF_EVENT_CUSTOM_MQTT_CONNECTED, NULL);

//         msg_id = esp_mqtt_client_publish(g_mesh_mqtt.client, "Energys/CNMD/", "connect ok", 0, 1, 0);
//         ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

//         msg_id = esp_mqtt_client_subscribe(g_mesh_mqtt.client, "Energys/CNMD/", 0);
//         ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        
//         msg_id = esp_mqtt_client_publish(g_mesh_mqtt.client, "Energys/STAT/", "connect ok", 0, 1, 0);
//         ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

//         msg_id = esp_mqtt_client_subscribe(g_mesh_mqtt.client, "Energys/STAT/", 0);
//         ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        
//         msg_id = esp_mqtt_client_publish(g_mesh_mqtt.client, "Energys/SENS/", "connect ok", 0, 1, 0);
//         ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

//         msg_id = esp_mqtt_client_subscribe(g_mesh_mqtt.client, "Energys/SENS/", 0);
//         ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
//         break;

//     case MQTT_EVENT_DISCONNECTED:
//         MDF_LOGD("MQTT_EVENT_DISCONNECTED");
//         g_mesh_mqtt.is_connected = false;
//         mdf_event_loop_send(MDF_EVENT_CUSTOM_MQTT_DISCONNECTED, NULL);
//         break;

//     case MQTT_EVENT_SUBSCRIBED:
//         MDF_LOGD("MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
//         break;

//     case MQTT_EVENT_UNSUBSCRIBED:
//         MDF_LOGD("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
//         break;

//     case MQTT_EVENT_PUBLISHED:
//         MDF_LOGD("MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
//         break;

//     case MQTT_EVENT_DATA:
//     {
//         MDF_LOGD("MQTT_EVENT_DATA, topic: %.*s, data: %.*s",
//                  event->topic_len, event->topic, event->data_len, event->data);

//         mesh_mqtt_data_t *item = mesh_mqtt_parse_data(event->topic, event->topic_len, event->data, event->data_len);

//         if (item == NULL)
//         {
//             break;
//         }

//         if (xQueueSend(g_mesh_mqtt.queue, &item, 0) != pdPASS)
//         {
//             MDF_LOGD("Send receive queue failed");
//             MDF_FREE(item->addrs_list);
//             MDF_FREE(item->data);
//             MDF_FREE(item);
//         }

//         break;
//     }

//     case MQTT_EVENT_ERROR:
//         MDF_LOGD("MQTT_EVENT_ERROR");
//         break;

//     default:
//         MDF_LOGD("Other event id:%d", event->event_id);
//         break;
//     }

//     return ESP_OK;
// }

/**
 * @brief WS2812 initialization 
 * 
 */



void wh2812_init()
{

   //config WS2812 and led pins as GPIO pins
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_RMT_TX_GPIO, CONFIG_RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    // install ws2812 driver
    led_strip_config_t  strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }

}


/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

void bh1750_init()
{
    i2c_bus_init();
    bh1750 = bh1750_create(I2C_MASTER_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    TEST_ASSERT_NOT_NULL_MESSAGE(bh1750, "BH1750 create returned NULL");
}

/**
 * @brief Task for the bh1750 light sensor.
 *        Reading light value in lux.
 * 
 * @param arg 
 */
static void bh1750_task(void *arg)
{

    static float bh1750_data;
    esp_err_t ret;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000;

    /**
     * @brief counter_mqtt for lower the message send to mqtt 
     * 
     */
    int counter_mqtt =0;

    xLastWakeTime = xTaskGetTickCount();

    while(true){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
                //TODO: need to disable the led
        if (strip->clear(strip, 100) == ESP_OK){
            
            strip->set_pixel(strip, 0, 255, 0, 0);
            strip->refresh(strip, 100);}
        MDF_LOGI("bh1750_task is running");

        // run every 1000ms
		//vTaskDelay(1000 / portTICK_PERIOD_MS);

        ret = bh1750_get_data(bh1750, &bh1750_data);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG_BH1750, "bh1750 val(CONTINUE mode): %f", bh1750_data);

        //send to mqtt server
        //TODO: need to add sdkconfig
        //if (counter_mqtt>60){
            // char *data = NULL;
            // mwifi_data_type_t data_type = { 0x0 };
            // size_t size   = 0;

            // size = asprintf(&data, "{\"type\":\"bh1750 data\", \"value\":%f}",
            //         bh1750_data);

            // MDF_LOGD("Node send, size: %d, data: %s", size, data);
            // ret = mwifi_write(NULL, &data_type, data, size, true);
            // MDF_FREE(data);
            // MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));

            if (esp_mesh_is_root()){
                MDF_LOGI("is root: update to mqtt directly");
                //mesh_mqtt_update_bh1750_data(&bh1750_data);
                
                //mesh_root_update_bh1750_data();
                //mesh_mqtt_write();
            }else{
                MDF_LOGI("not root: send message to root");
                uint8_t sta_mac[MWIFI_ADDR_LEN] = {0x0};
                MDF_ERROR_ASSERT(esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac));

                MDF_LOGI("my mac address is:" MACSTR, MAC2STR(sta_mac));

                //TODO: need to move to mesh_root_update_bh1750_data
                //mesh_root_update_bh1750_data(&bh1750_data, &sta_mac);
                mesh_root_update_bh1750_data(&bh1750_data);
                // char *data = NULL;
                // //mwifi_data_type_t data_type = { 0x0 };
                // //data_type.protocol = NAV_PROTO_JSON;
                // mwifi_data_type_t data_type = {
                //     .protocol = NAV_PROTO_JSON,
                //     .communicate = MWIFI_COMMUNICATE_MULTICAST,
                // };
                
                // size_t size   = 0;
                // size = asprintf(&data, "{\"scr_mac\" :" MACSTR ", \"type\" :\"BH1750_sensor\",\"data\" :%f}",
                //      MAC2STR(sta_mac), bh1750_data);

                // MDF_LOGD("Node send, size: %d, data: %s", size, data);
                // //mwifi_write(NULL, &data_type, data, size, true);
                // MDF_FREE(data);
                // MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));                


                // cJSON *obj = cJSON_CreateObject();

                // cJSON_AddNumberToObject(obj, "scr_mac", (double) sta_mac);
                // cJSON_AddNumberToObject(obj, "data", (double) bh1750_data);

                // char *data = NULL;
                // mwifi_data_type_t data_type = { 0x0 };
                // const char *send_data = cJSON_PrintUnformatted(obj);
                // MDF_LOGD("Node send, size: %d, data: %s", strlen(send_data), send_data);
                // mwifi_write(NULL, &data_type, send_data, strlen(send_data), true);
                // MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                // MDF_FREE(send_data);
                //cJSON_Delete(obj);

                //mesh_mqtt_update_bh1750_data(&bh1750_data);
            }

        //    counter_mqtt=0;
        //}
        
        //counter_mqtt++;
        xQueueSend(queue, &bh1750_data, portMAX_DELAY);

    }

    // clean-up
    ret = bh1750_delete(bh1750);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    //MDFFREE(bh1750_data);
    MDF_LOGI("bh1750_task is stop. driver disabled.");
    vTaskDelete(NULL);

}


/**
 * @brief AS312 PIR senor task for detecting IR signal
 * 
 * @param arg 
 */

void AS312_task(void *arg){

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000;

    xLastWakeTime = xTaskGetTickCount();

    /**
     * @brief counter for time out
     * 
     */
    int counter=0;
    // Initialize fade service.
    //ledc_fade_func_install(0);
    mdf_err_t ret = MDF_OK;

    /**
     * @brief counter_mqtt for lower the message send to mqtt 
     * 
     */
    int counter_mqtt =0;

    while(true){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        //TODO: need to disable the led

        //ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 255, 0, 0));
        //ESP_ERROR_CHECK(strip->refresh(strip, 100));
        if (control_flag == true)
        {
            if (gpio_get_level(CONFIG_AS312_DATA) == 1)
            {
                ESP_LOGI(TAG_AS312, "presence detected, light on.");
                light_driver_set_switch(true);
                light_driver_fade_brightness(100);
                counter =0;

                //for open the green led
                if (strip->clear(strip, 100) == ESP_OK){
                
                    strip->set_pixel(strip, 0, 0, 255, 0);
                    strip->refresh(strip, 100);}
                

                //publish light on message to all node

                    char *data = NULL;
                    size_t size = 0;
                    //const uint8_t dest_addr[6] = {0x94, 0xb9, 0x7e, 0xa8, 0xf2, 0x3c};
                    const uint8_t dest_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                    //const uint8_t dest_addr[6] = {0x30, 0x23, 0x03, 0xD7, 0xD3, 0x18};
                    MDF_LOGI("Format control message (light on). dest addr:" MACSTR, MAC2STR(dest_addr));
                    size = asprintf(&data, "{\"request\": \"set_status\",\"characteristics\": [{\"cid\": 0,\"value\": 1}]}\n");

                    mwifi_data_type_t mwifi_type = {
                        .protocol = MLINK_PROTO_HTTPD,
                        .communicate = MWIFI_COMMUNICATE_MULTICAST,
                    };

                    mlink_httpd_type_t header_info = {
                        .format = MLINK_HTTPD_FORMAT_JSON,
                        .from = MLINK_HTTPD_FROM_DEVICE,
                        .resp = false,
                    };

                    memcpy(&mwifi_type.custom, &header_info, sizeof(mlink_httpd_type_t));
                    //disable mwifi_write
                    //ret = mwifi_write(dest_addr, &mwifi_type, data, size, true);
                    MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                    MDF_FREE(data);
                    //end publish light on message to all node

                // //check if root and publish message to mqtt
                // if (esp_mesh_is_root()){
                //     mesh_mqtt_update_AS312_data("Present Detected");}
                // else {

                // }

            }else{
                ESP_LOGI(TAG_AS312, "the input is 0. counter value:%d", counter);
                counter++;

                //for close the green led
                if (strip->clear(strip, 100) == ESP_OK){
                
                    strip->set_pixel(strip, 0, 0, 0, 0);
                    strip->refresh(strip, 100);}
            }

            
            if (counter > (CONFIG_LIGHT_AUTO_OFF/1000)){
                ESP_LOGI(TAG_AS312, "No presence detected, light off. counter value:%d", counter);
                light_driver_set_switch(false);
                counter = 0;

                //publish message to all node

                    char *data = NULL;
                    //mwifi_data_type_t data_type = {0x0};
                    size_t size = 0;
                    //const uint8_t dest_addr[6] = {0x94, 0xb9, 0x7e, 0xa8, 0xf2, 0x3c};
                    const uint8_t dest_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                    //const uint8_t dest_addr[6] = {0x30, 0x23, 0x03, 0xD7, 0xD3, 0x18};

                    MDF_LOGI("Format control message (light off). dest addr:" MACSTR, MAC2STR(dest_addr));
                    size = asprintf(&data, "{\"request\": \"set_status\",\"characteristics\": [{\"cid\": 0,\"value\": 0}]}\n");

                    mwifi_data_type_t mwifi_type = {
                        .protocol = MLINK_PROTO_HTTPD,
                        .communicate = MWIFI_COMMUNICATE_MULTICAST,
                    };

                    mlink_httpd_type_t header_info = {
                        .format = MLINK_HTTPD_FORMAT_JSON,
                        .from = MLINK_HTTPD_FROM_DEVICE,
                        .resp = false,
                    };

                    memcpy(&mwifi_type.custom, &header_info, sizeof(mlink_httpd_type_t));
                    //disable mwifi_write
                    //ret = mwifi_write(dest_addr, &mwifi_type, data, size, true);
                    MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                    MDF_FREE(data);

                //end publish message to all node

                // //publish data to mqtt
                //                 //check if root and publish message to mqtt
                // if (esp_mesh_is_root()){
                // mesh_mqtt_update_AS312_data("No Present Detected");
                // }else{

                // }
                // //end publish

            } 
            else if (counter>(CONFIG_LIGHT_AUTO_OFF/1000/2) && counter < (CONFIG_LIGHT_AUTO_OFF/1000)){
                ESP_LOGI(TAG_AS312, "No presence detected, light fade down 50. counter value:%d", counter);
                light_driver_fade_brightness(50);
            }

            //send status to mqtt server
            if (counter_mqtt > 60){
                //send to mqtt server
                //char *data = NULL;
                //mwifi_data_type_t data_type = { 0x0 };
                //size_t size   = 0;

                if (gpio_get_level(CONFIG_AS312_DATA) == 1){
                    //size = asprintf(&data, "{\"type\":\"AS312 data\", \"value\":\"1\" }");
                    if (esp_mesh_is_root()){
                        mesh_mqtt_update_AS312_data("No Present Detected");
                        
                    }else{
                        
                    }
                } else {
                    //size = asprintf(&data, "{\"type\":\"AS312 data\", \"value\":\"0\"}");
                    if (esp_mesh_is_root()){
                        mesh_mqtt_update_AS312_data("Present Detected");
                        
                    } else {
                        
                    }
                }

                //MDF_LOGD("Node send, size: %d, data: %s", size, data);
                //ret = mwifi_write(NULL, &data_type, data, size, true);
                //MDF_FREE(data);
                //MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                counter_mqtt = 0;
            }
            counter_mqtt++;
        }
    }

    MDF_LOGI("AS312 Task Stopped");
    vTaskDelete(NULL);
}


// static struct mesh_mqtt
// {
//     xQueueHandle queue;              /**< mqtt receive data queue */
//     esp_mqtt_client_handle_t client; /**< mqtt client */
//     bool is_connected;
//     uint8_t addr[MWIFI_ADDR_LEN];
//     char publish_topic[220];
//     char topo_topic[32];
// } g_mesh_mqtt;

void sensor_mqtt_status_task(void *arg){

    while(true){
        mesh_mqtt_update_node_status();
    }

}

/**
 * @brief sensor mqtt update task
 * 
 * @param arg 
 */
void sensor_mqtt_update_task(void *arg){

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000;

    xLastWakeTime = xTaskGetTickCount();

    while(true){

        MDF_LOGI("sensor_mqtt_update_task is running");
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };

        MDF_ERROR_ASSERT(esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac));

        float bh1750_data;
        char* as312_data = "Demo";

        //cJSON *root, *bh1750, *as312;

        //root = cJSON_CreateObject();
        //bh1750 = cJSON_CreateObject();
        //as312 = cJSON_CreateObject();

        // char mac_str[17] ={0};
        // mlink_mac_hex2str(sta_mac, mac_str);
        // cJSON_AddStringToObject(root, "scr_mac", mac_str );
        // bh1750 = cJSON_AddObjectToObject(root, "BH1750_sensor");
        // //cJSON_AddNumberToObject(bh1750, "Lux", (double) *bh1750_data);
        // cJSON_AddNumberToObject(bh1750, "Lux", 2.00);
        // as312 = cJSON_AddObjectToObject(root, "AS312_sensor");
        // //cJSON_AddStringToObject(as312, "Present", as312_data);
        // cJSON_AddStringToObject(as312, "Present", "Present detected");

        // MDF_LOGD("the new JSON format as %s", cJSON_PrintUnformatted(root));

        //size_t size   = 0;
        //char *data = NULL;
        //string message = {"scr_mac":"94b97eacc648","BH1750_sensor":{"Lux":2},"AS312_sensor":{"Present":"Present detected"}}
        //size = asprintf(&data, "{\"scr_mac\":\"94b97eacc648\",\"BH1750_sensor\":{\"Lux\":2.00},\"AS312_sensor\":{\"Present\":\"Present detected\"}}");

        //MDF_LOGD("the new JSON format as %s", data);
//        cJSON_Delete(root);
//        cJSON_Delete(bh1750);
//        cJSON_Delete(as312);
        //MDF_ERROR_GOTO(str == NULL, _no_mem, "Print JSON failed");
        
        //getting as312 data
        // if (gpio_get_level(CONFIG_AS312_DATA) == 1){
        //     //size = asprintf(&data, "{\"type\":\"AS312 data\", \"value\":\"1\" }");
        //     if (esp_mesh_is_root()){
        //         //mesh_mqtt_update_AS312_data("No Present Detected");
        //         as312_data = "No Present Detected";
        //     }else{
                
        //     }
        // } else {
        //     //size = asprintf(&data, "{\"type\":\"AS312 data\", \"value\":\"0\"}");
        //     if (esp_mesh_is_root()){
        //         //mesh_mqtt_update_AS312_data("Present Detected");
        //         as312_data = "Present Detected";
        //     } else {
                
        //     }
        // }
        xQueueReceive(queue, &bh1750_data, portMAX_DELAY);

        
        mesh_mqtt_update_sensor_data( &bh1750_data, as312_data);
        //esp_mqtt_client_publish(g_mesh_mqtt.client, "Energys/SENS/", "{\"scr_mac\":\"94b97eacc648\",\"BH1750_sensor\":{\"Lux\":2.00},\"AS312_sensor\":{\"Present\":\"Present detected\"}}", 0, 1, 0);
    }

    MDF_LOGI("Sensor MQTT update task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Button task
 * 
 */
 static   button_event_t ev;
 static   QueueHandle_t button_events;
 int counter;
void button_task (void *arg){

    mdf_err_t ret = MDF_OK;
    MDF_LOGI("button Task Start");
    
    while (true) {


        if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
            if ((ev.pin == GPIO_NUM_39) && (ev.event == BUTTON_DOWN)) {

                ESP_LOGI(TAG_button, "button down. Issue on light message.");

                
                //suspend the sensor task to prevent enable the light
                //MDF_LOGI("bh1750 and AS312 task resume");
// #ifdef CONFIG_BH1750_ENABLE
//                 vTaskResume(BH1750_task_handle);
// #endif
// #ifdef CONFIG_AS312_ENABLE
//                 vTaskResume(AS312_task_handle);
// #endif

                //issue command to open light
                //publish message to all node
                char *data = NULL;
                //mwifi_data_type_t data_type = {0x0};
                size_t size = 0;
                //const uint8_t dest_addr[6] = {0x94, 0xb9, 0x7e, 0xa8, 0xf2, 0x3c};
                const uint8_t dest_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                //const uint8_t dest_addr[6] = {0x30, 0x23, 0x03, 0xD7, 0xD3, 0x18};
                MDF_LOGI("Format control message (light on). dest addr:" MACSTR, MAC2STR(dest_addr));
                size = asprintf(&data, "{\"request\": \"set_status\",\"control\":\"1\",\"characteristics\": [{\"cid\": 0,\"value\": 1}]}\n");
                //size = asprintf(&data, "{\"request\": \"set_status\",\"characteristics\": [{\"cid\": 0,\"value\": 1}]}\n");

                mwifi_data_type_t mwifi_type = {
                    .protocol = MLINK_PROTO_HTTPD,
                    .communicate = MWIFI_COMMUNICATE_MULTICAST,
                };

                mlink_httpd_type_t header_info = {
                    .format = MLINK_HTTPD_FORMAT_JSON,
                    .from = MLINK_HTTPD_FROM_DEVICE,
                    .resp = false,
                };

                memcpy(&mwifi_type.custom, &header_info, sizeof(mlink_httpd_type_t));

                ret = mwifi_write(dest_addr, &mwifi_type, data, size, true);
                MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                MDF_FREE(data);
                

                control_flag = true;


                                                        // //publish message to all node
                                                        // char *data0 = NULL;
                                                        // //mwifi_data_type_t data_type = {0x0};
                                                        // size_t size0 = 0;
                                                        // //const uint8_t dest_addr[6] = {0x94, 0xb9, 0x7e, 0xa8, 0xf2, 0x3c};
                                                        // const uint8_t dest_addr0[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                                                        // //const uint8_t dest_addr[6] = {0x30, 0x23, 0x03, 0xD7, 0xD3, 0x18};
                                                        // MDF_LOGI("Format control message (light on). dest addr:" MACSTR, MAC2STR(dest_addr0));
                                                        // size = asprintf(&data0, "{\"button_request\": 1}\n");

                                                        // mwifi_data_type_t mwifi_type0 = {
                                                        //     .protocol = MLINK_PROTO_HTTPD,
                                                        // };

                                                        // mlink_httpd_type_t header_info0 = {
                                                        //     .format = MLINK_HTTPD_FORMAT_JSON,
                                                        //     .from = MLINK_HTTPD_FROM_DEVICE,
                                                        //     .resp = false,
                                                        // };

                                                        // memcpy(&mwifi_type.custom, &header_info0, sizeof(mlink_httpd_type_t));

                                                        // ret = mwifi_write(dest_addr0, &mwifi_type0, data0, size0, true);
                                                        // MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                                                        // MDF_FREE(data0);                

                //end publish message to all node

                //led_flag = false;
                //esp_restart();
                //strip->del(strip);
            }
            if ((ev.pin == GPIO_NUM_39) && (ev.event == BUTTON_UP)) {
                
                ESP_LOGI(TAG_button, "button up lalallaa");
                //no use this event will appear after button down
                //led_flag = false;
                //esp_restart();
                //strip->del(strip);
            }
            if ((ev.pin == GPIO_NUM_39) && (ev.event == BUTTON_HELD)) {
                
                ESP_LOGI(TAG_button, "button held. Issue off light message. %d", counter);
                counter++;
                //suspend the sensor task to prevent enable the light
                //MDF_LOGI("bh1750 and AS312 task suspend");
// #ifdef CONFIG_BH1750_ENABLE
//                 vTaskSuspend(BH1750_task_handle);
// #endif
// #ifdef CONFIG_AS312_ENABLE
//                 vTaskSuspend(AS312_task_handle);
// #endif

                if (counter>10){
                //issue command to close light
                //publish message to all node
                char *data = NULL;
                //mwifi_data_type_t data_type = {0x0};
                size_t size = 0;
                //const uint8_t dest_addr[6] = {0x94, 0xb9, 0x7e, 0xa8, 0xf2, 0x3c};
                const uint8_t dest_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                //const uint8_t dest_addr[6] = {0x30, 0x23, 0x03, 0xD7, 0xD3, 0x18};
                MDF_LOGI("Format control message (light off). dest addr:" MACSTR, MAC2STR(dest_addr));
                size = asprintf(&data, "{\"request\": \"set_status\",\"control\":\"0\",\"characteristics\": [{\"cid\": 0,\"value\": 0}]}\n");
                //size = asprintf(&data, "{\"request\": \"set_status\",\"characteristics\": [{\"cid\": 0,\"value\": 0}]}\n");

                mwifi_data_type_t mwifi_type = {
                    .protocol = MLINK_PROTO_HTTPD,
                    .communicate = MWIFI_COMMUNICATE_MULTICAST,
                };

                mlink_httpd_type_t header_info = {
                    .format = MLINK_HTTPD_FORMAT_JSON,
                    .from = MLINK_HTTPD_FROM_DEVICE,
                    .resp = false,
                };

                memcpy(&mwifi_type.custom, &header_info, sizeof(mlink_httpd_type_t));

                ret = mwifi_write(dest_addr, &mwifi_type, data, size, true);
                MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                MDF_FREE(data);
                counter=0;

                control_flag = false;

                }

                                                    // //publish message to all node
                                                    // char *data1 = NULL;
                                                    // //mwifi_data_type_t data_type = {0x0};
                                                    // size_t size1 = 0;
                                                    // //const uint8_t dest_addr[6] = {0x94, 0xb9, 0x7e, 0xa8, 0xf2, 0x3c};
                                                    // const uint8_t dest_addr1[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                                                    // //const uint8_t dest_addr[6] = {0x30, 0x23, 0x03, 0xD7, 0xD3, 0x18};
                                                    // MDF_LOGI("Format control message (light off). dest addr:" MACSTR, MAC2STR(dest_addr1));
                                                    // size1 = asprintf(&data1, "{\"button_request\": 0}\n");

                                                    // mwifi_data_type_t mwifi_type1 = {
                                                    //     .protocol = MLINK_PROTO_HTTPD,
                                                    // };

                                                    // mlink_httpd_type_t header_info1 = {
                                                    //     .format = MLINK_HTTPD_FORMAT_JSON,
                                                    //     .from = MLINK_HTTPD_FROM_DEVICE,
                                                    //     .resp = false,
                                                    // };

                                                    // memcpy(&mwifi_type1.custom, &header_info1, sizeof(mlink_httpd_type_t));

                                                    // ret = mwifi_write(dest_addr1, &mwifi_type1, data1, size1, true);
                                                    // MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                                                    // MDF_FREE(data1);     

                //end publish message to all node

                //led_flag = false;
                //esp_restart();
                //strip->del(strip);
            }
        }
    }

    MDF_LOGI("button Task Stopped");
    vTaskDelete(NULL);

}

void app_main()
{
    /**
     * @brief create queue for the variable tranfer between task 
     * 
     */
    queue = xQueueCreate(6, sizeof(float));

    /**
     * @brief Create WS2812 onbroad led and light up for indicate power on
     * 
     */

    wh2812_init();
 
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    ESP_LOGI(TAG_WS2182, "WS2812 is Running");

    ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 16, 90, 219));
    ESP_ERROR_CHECK(strip->refresh(strip, 100));

    //Setup button 
    button_events = button_init(PIN_BIT(GPIO_NUM_39));
    xTaskCreate(button_task, "button_task", 1024 * 2,  NULL, 1, NULL);


    char name[32]                   = {0};
    uint8_t sta_mac[6]              = {0};
    mwifi_config_t ap_config        = {0x0};
    mwifi_init_config_t init_config = MWIFI_INIT_CONFIG_DEFAULT();
    mdebug_log_config_t log_config = {
        .log_uart_enable = true,
        .log_espnow_enable = true,
    };

    /**
     * NOTE:
     *  If the module has SPI flash, GPIOs 6-11 are connected to the module’s integrated SPI flash and PSRAM.
     *  If the module has PSRAM, GPIOs 16 and 17 are connected to the module’s integrated PSRAM.
     */
    light_driver_config_t driver_config = {
        .gpio_red        = CONFIG_LIGHT_GPIO_RED,
        .gpio_green      = CONFIG_LIGHT_GPIO_GREEN,
        .gpio_blue       = CONFIG_LIGHT_GPIO_BLUE,
        .gpio_cold       = CONFIG_LIGHT_GPIO_COLD,
        .gpio_warm       = CONFIG_LIGHT_GPIO_WARM,
        .fade_period_ms  = CONFIG_LIGHT_FADE_PERIOD_MS,
        .blink_period_ms = CONFIG_LIGHT_BLINK_PERIOD_MS,
    };

    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /**
     * @brief Continuous power off and restart more than three times to reset the device
     */
    if (restart_count_get() >= LIGHT_RESTART_COUNT_FALLBACK) {
        mupgrade_version_fallback();
        MDF_LOGW("Fall back to the previous version");
    } else if (restart_count_get() >= LIGHT_RESTART_COUNT_RESET) {
        MDF_LOGW("Erase information saved in flash");
        mdf_info_erase(MDF_SPACE_NAME);
    }

    /**
     * @brief Light driver initialization
     */
    MDF_ERROR_ASSERT(light_driver_init(&driver_config));
    light_driver_set_switch(true);

    if (mdf_info_load("ap_config", &ap_config, sizeof(mwifi_config_t)) == MDF_OK) {
        if (restart_is_exception()) {
            light_driver_set_rgb(255, 0, 0); /**< red */
        } else {
            light_driver_set_switch(true);   /**< turn on */
        }
    } else {
        light_driver_breath_start(255, 255, 0); /**< yellow blink */
    }

    /**
     * @brief   1.Initialize event loop, receive event
     *          2.Initialize wifi with station mode
     *          3.Initialize espnow(ESP-NOW is a kind of connectionless WiFi communication protocol)
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default());
    MDF_ERROR_ASSERT(esp_netif_create_default_wifi_mesh_netifs(&sta_netif, NULL));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mespnow_init());

    /**
     * @brief Add debug function, you can use serial command and wireless debugging.
     *      1. Initialize console module
     */
    MDF_ERROR_ASSERT(mdebug_console_init());
    MDF_ERROR_ASSERT(mdebug_log_set_config(&log_config));
    mdebug_cmd_register_common();

    /**
     * @brief   1.Get Mwifi initialization configuration information and Mwifi AP configuration information from nvs flash.
     *          2.If there is no network configuration information in the nvs flash,
     *              obtain the network configuration information through the blufi or mconfig chain.
     *          3.Indicate the status of the device by means of a light
     */
    if (mdf_info_load("ap_config", &ap_config, sizeof(mwifi_config_t)) != MDF_OK) {
        MDF_ERROR_ASSERT(get_network_config(&init_config, &ap_config, LIGHT_TID, LIGHT_NAME));
        MDF_LOGI("mconfig, ssid: %s, password: %s, mesh_id: " MACSTR,
                 ap_config.router_ssid, ap_config.router_password,
                 MAC2STR(ap_config.mesh_id));

#ifdef CONFIG_LIGHT_NETWORKING_TIME_OPTIMIZE_ENABLE

        if (g_config_from_blufi_flag) {
            ap_config.mesh_type = MESH_ROOT;
        }

#endif /**< CONFIG_LIGHT_NETWORKING_TIME_OPTIMIZE_ENABLE */

        /**
         * @brief Save configuration information to nvs flash.
         */
        mdf_info_save("ap_config", &ap_config, sizeof(mwifi_config_t));
    }

    /**
     * @brief Note that once BT controller memory is released, the process cannot be reversed.
     *        It means you can not use the bluetooth mode which you have released by this function.
     *        it can release the .bss, .data and other section to heap
     */
#ifdef CONFIG_IDF_TARGET_ESP32
    esp_bt_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif

    /**< When using a BLE gateway, you must ensure that the BLE host stack is not released */
#ifndef CONFIG_LIGHT_BLE_GATEWAY
    esp_bt_mem_release(ESP_BT_MODE_BLE);
#endif /**< CONFIG_LIGHT_BLE_GATEWAY */

    /**
     * @brief Configure MLink (LAN communication module)
     *          1.add device
     *          2.add characteristic of device
     *          3.add characteristic handle for get/set value of characteristic.
     */
    MDF_ERROR_ASSERT(esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac));
    snprintf(name, sizeof(name), LIGHT_NAME "_%02x%02x", sta_mac[4], sta_mac[5]);
    MDF_ERROR_ASSERT(mlink_add_device(LIGHT_TID, name, CONFIG_LIGHT_VERSION));
    MDF_ERROR_ASSERT(mlink_add_characteristic(LIGHT_CID_STATUS, "on", CHARACTERISTIC_FORMAT_INT, CHARACTERISTIC_PERMS_RWT, 0, 3, 1));
    MDF_ERROR_ASSERT(mlink_add_characteristic(LIGHT_CID_HUE, "hue", CHARACTERISTIC_FORMAT_INT, CHARACTERISTIC_PERMS_RWT, 0, 360, 1));
    MDF_ERROR_ASSERT(mlink_add_characteristic(LIGHT_CID_SATURATION, "saturation", CHARACTERISTIC_FORMAT_INT, CHARACTERISTIC_PERMS_RWT, 0, 100, 1));
    MDF_ERROR_ASSERT(mlink_add_characteristic(LIGHT_CID_VALUE, "value", CHARACTERISTIC_FORMAT_INT, CHARACTERISTIC_PERMS_RWT, 0, 100, 1));
    MDF_ERROR_ASSERT(mlink_add_characteristic(LIGHT_CID_COLOR_TEMPERATURE, "color_temperature", CHARACTERISTIC_FORMAT_INT, CHARACTERISTIC_PERMS_RWT, 0, 100, 1));
    MDF_ERROR_ASSERT(mlink_add_characteristic(LIGHT_CID_BRIGHTNESS, "brightness", CHARACTERISTIC_FORMAT_INT, CHARACTERISTIC_PERMS_RWT, 0, 100, 1));
    MDF_ERROR_ASSERT(mlink_add_characteristic(LIGHT_CID_MODE, "mode", CHARACTERISTIC_FORMAT_INT, CHARACTERISTIC_PERMS_RW, 1, 3, 1));
    MDF_ERROR_ASSERT(mlink_add_characteristic_handle(mlink_get_value, mlink_set_value));

    /**
     * @brief Initialize trigger handler
     *          while characteristic of device reaching conditions， will trigger the corresponding action.
     */
    MDF_ERROR_ASSERT(mlink_trigger_init());

    /**
     * @brief Initialize espnow_to_mwifi_task for forward esp-now data to the wifi mesh network.
     * esp-now data from button or other device.
     */
    xTaskCreate(espnow_to_mwifi_task, "espnow_to_mwifi", 1024 * 3,  NULL, 1, NULL);

    /**
     * @brief Add a request handler, handling request for devices on the LAN.
     */
    MDF_ERROR_ASSERT(mlink_set_handle("show_layer", light_show_layer));
    MDF_ERROR_ASSERT(mlink_set_handle("get_tsf_time", light_get_tsf_time));

#ifndef CONFIG_LIGHT_BLE_GATEWAY
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
#endif

    /**
     * @brief Initialize and start esp-mesh network according to network configuration information.
     */
    MDF_ERROR_ASSERT(mwifi_init(&init_config));
    MDF_ERROR_ASSERT(mwifi_set_config(&ap_config));
    MDF_ERROR_ASSERT(mwifi_start());

    mwifi_print_config();

    /**
     * @brief Add a default group for the meshkit_button to control all devices
     */
    const uint8_t default_group_id[6] = {LIGHT_TID};
    esp_mesh_set_group_id((mesh_addr_t *)default_group_id, 1);

    /**
     * @brief Handling data between wifi mesh devices.
     */
    xTaskCreate(node_handle_task, "node_handle", 4 * 1024,
               NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

    /**
     * @brief Create node handler
     */
    xTaskCreate(node_write_task, "node_write_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    //xTaskCreate(node_read_task, "node_read_task", 4 * 1024,
    //            NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);


    //Set LED to purple after network config ok
    ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 150, 0, 255));
    ESP_ERROR_CHECK(strip->refresh(strip, 100));


    /**
     *  @brief Create bh1750 sensor task
     */

    esp_err_t ret;
    bh1750_measure_mode_t cmd_measure;
    MDF_LOGI("bh1750(setting) is running");
    bh1750_init();
    ret = bh1750_power_on(bh1750);
    if (ret == ESP_OK){
        TEST_ASSERT_EQUAL(ESP_OK, ret);
#ifdef CONFIG_BH1750_CONTINUE_1LX_RES
        cmd_measure = BH1750_CONTINUE_1LX_RES;
#elif CONFIG_BH1750_CONTINUE_HALFLX_RES
        cmd_measure = BH1750_CONTINUE_HALFLX_RES;
#elif CONFIG_BH1750_CONTINUE_4LX_RES
        cmd_measure = BH1750_CONTINUE_4LX_RES;
#endif
        ret = bh1750_set_measure_mode(bh1750, cmd_measure);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        vTaskDelay(30 / portTICK_RATE_MS);
#ifdef CONFIG_BH1750_ENABLE
        xTaskCreate(bh1750_task, "bh1750_task", 4 * 1024, 
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, &BH1750_task_handle);
#endif
        
        /**
         *  @brief Create AS312 sensor task only enable if the bh1750 sensor is present
         */
        // configure button and led pins as GPIO pins

        MDF_LOGI("AS312(setting) is running");
        gpio_pad_select_gpio(CONFIG_AS312_DATA);
        //gpio_pad_select_gpio(CONFIG_LED_PIN);

        // set the correct direction
        gpio_set_direction(CONFIG_AS312_DATA, GPIO_MODE_INPUT);
        //gpio_set_direction(CONFIG_LED_PIN, GPIO_MODE_OUTPUT)
#ifdef CONFIG_AS312_ENABLE
        xTaskCreate(AS312_task, "AS312_task", 4 * 1024, 
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, &AS312_task_handle);
#endif


        //create task for the sensor
        xTaskCreate(sensor_mqtt_update_task, "sensor_mqtt_update_task", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, &sensor_mqtt_task_handle);
        sensor_flag = true;
    } else {
        sensor_flag = false;
    }




    /**
     * @brief Periodically print system information.
     */
    TimerHandle_t timer = xTimerCreate("show_system_info", pdMS_TO_TICKS(10000),
                                       true, NULL, show_system_info_timercb);
    xTimerStart(timer, 0);
    

}
