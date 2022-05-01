#ifndef BLE_H      /* Include guard */
#define BLE_H



/****************************************** Header includes *******************************************/

/* Bluetooth */
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"



/*************************************** Structure Definitions ****************************************/

/*!
 * @brief BLE scan result.
 */
typedef struct {
    esp_bd_addr_t address;
    int rssi;
} ble_scan_result_t;



/*********************************** Function Prototype Declarations **********************************/

/*!
 * @brief  This public function is used to initialize the BLE radio.
 * 
 * @param[in] scan_params   :Structure instance of esp_ble_scan_params_t.
 * @param[in] raw_adv_data  :Raw advertising data.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t ble_init(esp_ble_scan_params_t scan_params, uint8_t *raw_adv_data);


/*!
 * @brief  This public function is used to start scanning for other
 *         BLE devices for a given duration in second.
 * 
 * @param[in] duration_sec  :Scan duration in seconds.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t ble_start_scanning(uint32_t duration_sec);


/*!
 * @brief  This public function is used to stop scanning for other
 *         BLE devices.
 * 
 * @param  :Nothing.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t ble_stop_scanning(void);


/*!
 * @brief  This public function is used to start advertising.
 * 
 * @param[in] vibration  :Structure instance of vibration_motor_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t ble_start_advertising(esp_ble_adv_params_t adv_params);


/*!
 * @brief  This public function is used to stop advertising.
 * 
 * @param  :Nothing.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t ble_stop_advertising(void);


/*!
 * @brief  This public function is used to get the scan result
 *         of BLE devices nearby.
 * 
 * @param[in] vibration  :Structure instance of vibration_motor_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t ble_get_scan_result(ble_scan_result_t *ble_scan_result);



#endif      /* End of include guard */

/******************************************************************************************************/