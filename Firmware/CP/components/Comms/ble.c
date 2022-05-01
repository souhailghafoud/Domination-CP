/****************************************** Header includes *******************************************/

/* std Lib */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

/* BLE */
#include "ble.h"



/********************************************* Constants **********************************************/

#define BLE_SCAN_RESULT_QUEUE_LEN	            20			// BLE scan result Queue length

#define BLE_FAIL_BIT                            BIT0        // BLE fail EventBit
#define BLE_SCAN_PARAM_SET_COMPLETE_BIT         BIT1        // BLE scan parameter setup complete EventBit
#define BLE_SCAN_START_SUCCESS_BIT              BIT2        // BLE scan start success EventBit
#define BLE_SCAN_STOP_SUCCESS_BIT               BIT3        // BLE scan stop success EventBit
#define BLE_ADV_DATA_RAW_SET_COMPLETE_BIT       BIT4        // BLE adv raw data setup complete EventBit
#define BLE_ADV_START_SUCCESS_BIT               BIT5        // BLE adv start success EventBit
#define BLE_ADV_STOP_SUCCESS_BIT                BIT6        // BLE adv stop success EventBit



/***************************************** Static Variables *******************************************/

static QueueHandle_t s_ble_scan_result_queue = NULL;	// BLE scan result Queue handle

static EventGroupHandle_t s_ble_event_group = NULL;     // BLE EvenGroup handle




/***************************************** Private Functions ******************************************/

/*!
 * @brief This private function is used for BLE radio events
 */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err = ESP_OK;

    /* Parse BLE event */
    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;            
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    /* Store device address and RSSI value */
					ble_scan_result_t ble_scan_result = {0};
                    memcpy(ble_scan_result.address, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                    ble_scan_result.rssi = scan_result->scan_rst.rssi;
                    /* Enqueue scan result */
                    xQueueSend(s_ble_scan_result_queue, &ble_scan_result, 0);
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            /* Scan start complete event to indicate scan start successfully or failed */
            if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                /* Set fail EventBit */
                xEventGroupSetBits(s_ble_event_group, BLE_FAIL_BIT);
            }
            else {
                /* Set scan start success EventBit */
                xEventGroupClearBits(s_ble_event_group, BLE_FAIL_BIT);
                xEventGroupClearBits(s_ble_event_group, BLE_SCAN_STOP_SUCCESS_BIT);
                xEventGroupSetBits(s_ble_event_group, BLE_SCAN_START_SUCCESS_BIT);
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            /* Scan stop complete event to indicate scan stop successfully or failed */
            if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                /* Set fail EventBit */
                xEventGroupSetBits(s_ble_event_group, BLE_FAIL_BIT);
            }
            else {
                /* Set scan stop success EventBit */
                xEventGroupClearBits(s_ble_event_group, BLE_FAIL_BIT);
                xEventGroupClearBits(s_ble_event_group, BLE_SCAN_START_SUCCESS_BIT);
                xEventGroupSetBits(s_ble_event_group, BLE_SCAN_STOP_SUCCESS_BIT);
            }
            break;
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            /* Set scan parameter setup complete EventBit */
            xEventGroupClearBits(s_ble_event_group, BLE_FAIL_BIT);
            xEventGroupSetBits(s_ble_event_group, BLE_SCAN_PARAM_SET_COMPLETE_BIT);
            break;
        }
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            /* Set adv raw data setup complete EventBit */
            xEventGroupClearBits(s_ble_event_group, BLE_FAIL_BIT);
            xEventGroupSetBits(s_ble_event_group, BLE_ADV_DATA_RAW_SET_COMPLETE_BIT);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* Adv start complete event to indicate adv start successfully or failed */
            if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                /* Set fail EventBit */
                xEventGroupSetBits(s_ble_event_group, BLE_FAIL_BIT);
            }
            else {
                /* Set adv start success EventBit */
                xEventGroupClearBits(s_ble_event_group, BLE_FAIL_BIT);
                xEventGroupClearBits(s_ble_event_group, BLE_ADV_STOP_SUCCESS_BIT);
                xEventGroupSetBits(s_ble_event_group, BLE_ADV_START_SUCCESS_BIT);
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            /* Adv stop complete event to indicate adv stop successfully or failed */
            if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                /* Set fail EventBit */
                xEventGroupSetBits(s_ble_event_group, BLE_FAIL_BIT);
            }
            else {
                /* Set adv stop success EventBit */
                xEventGroupClearBits(s_ble_event_group, BLE_FAIL_BIT);
                xEventGroupClearBits(s_ble_event_group, BLE_ADV_START_SUCCESS_BIT);
                xEventGroupSetBits(s_ble_event_group, BLE_ADV_STOP_SUCCESS_BIT);
            }
            break;
        default:
            break;
    }
}



/***************************************** Public Functions *******************************************/

/*!
 * @brief This public function is used to initialize the BLE radio.
 */
esp_err_t ble_init(esp_ble_scan_params_t scan_params, uint8_t *raw_adv_data)
{    
    esp_err_t esp_status = ESP_OK;
    
    /* Create an EventGroup for BLE events */
    s_ble_event_group = xEventGroupCreate();

    /* Create scan result queue */
    s_ble_scan_result_queue = xQueueCreate(BLE_SCAN_RESULT_QUEUE_LEN, sizeof(ble_scan_result_t));
		
    /* Free unused BT Classic memory */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* BT controller configuration */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);            //Initialize BT controller
    esp_bt_controller_enable(ESP_BT_MODE_BLE);  // Enable BT controller
    
    esp_bluedroid_init();   // Init and alloc the resource for bluetooth
    esp_bluedroid_enable(); // Enable bluetooth

    /* Register the scan callback function to the gap module */
    esp_status = esp_ble_gap_register_callback(esp_gap_cb);
    if (esp_status != ESP_OK) {
		return esp_status;
    }
    
    /* Set advertising data */
    esp_status = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (esp_status != ESP_OK) {
		return esp_status;
    }
    
    /* Set scan parameters */
    esp_ble_gap_set_scan_params(&scan_params);
    
    /* Wait until the scan parameter setup is complete (BLE_SCAN_PARAM_SET_COMPLETE_BIT) 
     * and the advertising raw data setup is complete (BLE_ADV_DATA_RAW_SET_COMPLETE_BIT)
     * */
    EventBits_t bits = xEventGroupWaitBits(s_ble_event_group,
                                           BLE_SCAN_PARAM_SET_COMPLETE_BIT | 
                                           BLE_ADV_DATA_RAW_SET_COMPLETE_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    
    /* Check bits status */
    if (bits & (BLE_SCAN_PARAM_SET_COMPLETE_BIT | 
                BLE_ADV_DATA_RAW_SET_COMPLETE_BIT)) {
        return ESP_OK;
    }
    else {
        return ESP_FAIL;
    }
}


/*!
 * @brief This public function is used to start scanning for other
 *        BLE devices for a given duration in second.
 */
esp_err_t ble_start_scanning(uint32_t duration_sec)
{
    /* BLE eventGroup not created */
    if (NULL == s_ble_event_group) {return ESP_FAIL;}

	esp_ble_gap_start_scanning(duration_sec);   // Start scanning

    /* Wait until either the scan started 
     * successfully (BLE_SCAN_PARAM_SET_COMPLETE_BIT) 
     * or failed (BLE_FAIL_BIT)
     * */
    EventBits_t bits = xEventGroupWaitBits(s_ble_event_group,
                                           BLE_SCAN_START_SUCCESS_BIT | 
                                           BLE_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           portMAX_DELAY);

    /* Check bits status */
    if (bits & BLE_FAIL_BIT) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


/*!
 * @brief This public function is used to stop scanning for other
 *        BLE devices.
 */
esp_err_t ble_stop_scanning(void)
{
    /* BLE eventGroup not created */
    if (NULL == s_ble_event_group) {return ESP_FAIL;}

	esp_ble_gap_stop_scanning();    // Stop scanning

    /* Wait until either the scan stopped 
     * successfully (BLE_SCAN_STOP_SUCCESS_BIT) 
     * or failed (BLE_FAIL_BIT)
     * */
    EventBits_t bits = xEventGroupWaitBits(s_ble_event_group,
                                           BLE_SCAN_STOP_SUCCESS_BIT | 
                                           BLE_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           portMAX_DELAY);

    /* Check bits status */
    if (bits & BLE_FAIL_BIT) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


/*!
 * @brief This public function is used to start advertising.
 */
esp_err_t ble_start_advertising(esp_ble_adv_params_t adv_params)
{
    /* BLE eventGroup not created */
    if (NULL == s_ble_event_group) {return ESP_FAIL;}

	esp_ble_gap_start_advertising(&adv_params); // Start advertising

    /* Wait until either the advertising started 
     * successfully (BLE_ADV_START_SUCCESS_BIT) 
     * or failed (BLE_FAIL_BIT)
     * */
    EventBits_t bits = xEventGroupWaitBits(s_ble_event_group,
                                           BLE_ADV_START_SUCCESS_BIT | 
                                           BLE_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           portMAX_DELAY);

    /* Check bits status */
    if (bits & BLE_FAIL_BIT) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


/*!
 * @brief This public function is used to stop advertising.
 */
esp_err_t ble_stop_advertising(void)
{
    /* BLE eventGroup not created */
    if (NULL == s_ble_event_group) {return ESP_FAIL;}

	esp_ble_gap_stop_advertising(); // Stop advertising

    /* Wait until either the advertising stopped 
     * successfully (BLE_ADV_STOP_SUCCESS_BIT) 
     * or failed (BLE_FAIL_BIT)
     * */
    EventBits_t bits = xEventGroupWaitBits(s_ble_event_group,
                                           BLE_ADV_STOP_SUCCESS_BIT | 
                                           BLE_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           portMAX_DELAY);

    /* Check bits status */
    if (bits & BLE_FAIL_BIT) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


/*!
 * @brief This public function is used to get the scan result
 *        of BLE devices nearby.
 */
esp_err_t ble_get_scan_result(ble_scan_result_t *ble_scan_result)
{
    /* BLE scan result queue not created */
    if (NULL == s_ble_event_group) {return ESP_FAIL;}

    /* Dequeue scan result of BLE devices nearby */
	xQueueReceive(s_ble_scan_result_queue, ble_scan_result, portMAX_DELAY);

    return ESP_OK;
}



/******************************************************************************************************/