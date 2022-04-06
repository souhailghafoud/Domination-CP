/*******************************************************************************************************
**
** @brief     XstractiK Domination Project  -  CP-v0.1.0 Firmware
**
** @copyright Copyright © 2021 GHS. All rights reserved.
** 
** @file	  main.cpp
** @author    Souhail Ghafoud
** @date	  November 05, 2021
** @version	  0.1.0
**
*******************************************************************************************************/



/****************************************** Header includes *******************************************/

/* std Lib */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* GPIO */
#include "driver/gpio.h"

/* PWM */
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

/* Timer */
#include "esp_timer.h"

/* NVS */
#include "nvs_flash.h"

/* Bluetooth */
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

/* LOG */
#include "esp_log.h"

/* LoRa */
#include "lora.h"



/********************************************* Constants **********************************************/

#define FIRMWARE_VERSION            "0.1.0"                     // Firmware version

#define PRO_CORE                    0                           // ESP32 Core 0
#define APP_CORE                    1                           // ESP32 Core 1

#define LIMIT_SWITCH_TASK_STACK     (1024 * 3)                  // Stack size in bytes
#define LIMIT_SWITCH_TASK_PRIORITY  (tskIDLE_PRIORITY + 3)      // Priority level
#define LIMIT_SWITCH_TASK_CORE      APP_CORE                    // CPU core ID

#define BEACON_PARSER_TASK_STACK    (1024 * 3)                  // Stack size in bytes
#define BEACON_PARSER_TASK_PRIORITY (tskIDLE_PRIORITY + 1)      // Priority level
#define BEACON_PARSER_TASK_CORE     APP_CORE                    // CPU core ID

#define CP_CTRL_TASK_STACK          (1024 * 3)                  // Stack size in bytes
#define CP_CTRL_TASK_PRIORITY       (tskIDLE_PRIORITY + 2)      // Priority level
#define CP_CTRL_TASK_CORE           APP_CORE                    // CPU core ID

#define MAX_TEAMS                   2
#define MAX_UNITS                   5
#define MIN_UNITS                   0
#define TEAM_INFO_QUEUE_LEN         5                           // Team info Queue length

#define MAX_BEACONS                 10
#define MAX_RSSI_COUNT              10
#define RSSI_RADIUS                 -65                         // dBm
#define BEACON_QUEUE_LEN            MAX_BEACONS                 // Beacon devives Queue length

#define LORA_PACKET_LEN            32

#define HIGH			            1
#define LOW 			            0

/* SPI pins */
#define PIN_SPI_MOSI                GPIO_NUM_23
#define PIN_SPI_MISO                GPIO_NUM_19
#define PIN_SPI_CLK                 GPIO_NUM_18

/* LoRa pins */
#define PIN_LORA_DIO0               GPIO_NUM_27
#define PIN_LORA_RESET              GPIO_NUM_14

/* Limit switches pins */
#define PIN_LIMIT_SWITCH_1          GPIO_NUM_4
#define PIN_LIMIT_SWITCH_2          GPIO_NUM_2
#define PIN_LIMIT_SWITCH_3          GPIO_NUM_15

/* Stepper motor pins */
#define PIN_MOTOR_DIR               GPIO_NUM_32
#define PIN_MOTOR_STEP              GPIO_NUM_33
#define PIN_MOTOR_RESET             GPIO_NUM_25

/* Stepper motor directions (Clockwise & Conterclockwise) */
#define MOTOR_DIR_CW			    HIGH
#define MOTOR_DIR_CCW 			    LOW



/************************************** Enumeration Definitions ***************************************/

/*!
 * @brief LED colors.
 */
typedef enum {
    LED_OFF = 0,
    LED_GREEN,
    LED_RED
} led_color_t;


/*!
 * @brief Beacon proximity status.
 */
typedef enum {
    BEACON_OUT_RANGE = 0,
    BEACON_IN_RANGE,
    BEACON_ABSENT
} beacon_proximity_t;


/*!
 * @brief Team id.
 */
typedef enum {
    TEAM_A = 0,
    TEAM_B,
    TEAM_NONE
} team_id_t;


/*!
 * @brief Team units.
 */
typedef enum {
    UNITS_NONE = 0,
    UNITS_PLUS,
    UNITS_MINUS
} team_units_t;


/*!
 * @brief Flag positons.
 */
typedef enum {
    POSITION_A = 0,
    POSITION_B,
    POSITION_NONE,
    POSITION_START
} flag_position_t;


/*!
 * @brief CP id.
 */
typedef enum {
    CP_A = 0,
    CP_B,
    CP_C
} cp_id_t;


/*!
 * @brief CP status.
 */
typedef enum {
    CP_CAPTURED = 0,
    CP_CAPTURING
} cp_status_t;


/*!
 * @brief Stepper motor frequency in Hz.
 */
typedef enum {
    STEPPER_FREQ_00HZ = 0,
    STEPPER_FREQ_20HZ = 20,
    STEPPER_FREQ_30HZ = 30,
    STEPPER_FREQ_40HZ = 40,
    STEPPER_FREQ_50HZ = 50,
    STEPPER_FREQ_60HZ = 60
} stepper_freq_t;



/*************************************** Structure Definitions ****************************************/

/*!
 * @brief Beacon device.
 */
typedef struct {
    esp_bd_addr_t address;
    beacon_proximity_t proximity;
    int rssi;
    uint8_t rssi_counter;
} beacon_dev_t;


/*!
 * @brief Team info.
 */
typedef struct {
    team_id_t id;
    team_units_t units;
    uint8_t units_counter;
    uint8_t capture_speed;
    bool b_cp_captured;
} team_info_t;



/***************************************** Static Variables *******************************************/

static QueueHandle_t s_beacon_queue = NULL;             // Beacon devices Queue handle
static QueueHandle_t s_team_info_queue = NULL;          // Team info Queue handle
static QueueHandle_t s_limit_switch_queue = NULL;       // Limit switch Queue handle

static esp_timer_handle_t s_beacon_timer[MAX_BEACONS] = {0};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t raw_adv_data[20] = {
    /* Len:2, Type:1 (flags), Data:6 */
    0x02, 0x01, 0x06,
    /* Len:2, Type:a (tx power), Data:eb*/
    0x02, 0x0a, 0xeb,
    /* Len:3, Type:3 (Complete List of 16-bit Service Class UUIDs), Data: FF 00 */
    0x03, 0x03, 0xFF, 0xF1,
    /* Len:5, Type:9 (Complete Local Name) */
    0x05, 0x09, 'C', '-', '0', '1'
};


static stepper_freq_t stepper_freq[6] = {
    STEPPER_FREQ_00HZ,
    STEPPER_FREQ_20HZ,
    STEPPER_FREQ_30HZ,
    STEPPER_FREQ_40HZ,
    STEPPER_FREQ_50HZ,
    STEPPER_FREQ_60HZ
};



/*********************************************** ISRs *************************************************/

/*!
 * @brief This interrupt service routine is used to .
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void limit_switch_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // Higher priority task flag
    uint32_t limit_switch_num = (uint32_t) arg;

    xQueueSendFromISR(s_limit_switch_queue, &limit_switch_num, &xHigherPriorityTaskWoken);

    /* Wake up higher priority task immediately */
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



/***************************************** Private Functions ******************************************/

/*!
 * @brief This private function is used as a callback for timer alarms. 
 *
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void beacon_timer_cb(void *arg)
{
    beacon_dev_t beacon = {0};

    //printf("Timer beacon_addr[0] = %X\n", *(*(esp_bd_addr_t *)arg));

    /* Enqueue beacon device */
    memcpy(beacon.address, (esp_bd_addr_t *)arg, sizeof(esp_bd_addr_t));
    beacon.proximity = BEACON_ABSENT;
    xQueueSend(s_beacon_queue, &beacon, 0);
}


/*!
 * @brief This private function is used as a callback for BLE events. 
 *
 * @param[in] event      :Structure instance of esp_gap_ble_cb_event_t
 * @param[in,out] param  :Structure instance of esp_ble_gap_cb_param_t
 * 
 * @return Nothing.
 */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err = ESP_OK;

    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    /* Store beacon device address and RSSI value */
                    beacon_dev_t beacon = {0};
                    memcpy(beacon.address, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                    beacon.rssi = scan_result->scan_rst.rssi;
                    /* Enqueue beacon device */
                    xQueueSend(s_beacon_queue, &beacon, 0);
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            //scan start complete event to indicate scan start successfully or failed
            if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                printf("Scan start failed: %s\n\n", esp_err_to_name(err));
            }
            else {
                printf("Start scan successfully\n\n");
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            //scan stop complete event to indicate scan stop successfully or failed
            if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                printf("Scan stop failed: %s\n\n", esp_err_to_name(err));
            }
            else {
                printf("Stop scan successfully\n\n");
            }
            break;
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            //the unit of the duration in seconds, 0 means scan permanently
            uint32_t duration_s = 0;
            esp_ble_gap_start_scanning(duration_s);
            break;
        }
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&ble_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //adv start complete event to indicate adv start successfully or failed
            if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                printf("Adv start failed: %s\n\n", esp_err_to_name(err));
            }
            else {
                printf("start adv successfully\n\n");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            //adv stop complete event to indicate adv stop successfully or failed
            if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                printf("Adv stop failed: %s\n\n", esp_err_to_name(err));
            }
            else {
                printf("Stop adv successfully\n\n");
            }
            break;
        default:
            break;
    }
}



/***************************************** Public Functions *******************************************/

/*!
 * @brief This public function is used to delay a task for a period
 *        of time in milliseconds.
 */
void delay_ms(uint32_t period_ms)
{
    vTaskDelay((period_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}


/*!
 * @brief This public function is used to initialize the stepper motor.
 *
 * @param  :Nothing.
 *
 * @return Nothing.
 */
void stepper_init(void)
{
    gpio_set_direction(PIN_MOTOR_DIR, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_MOTOR_RESET, GPIO_MODE_OUTPUT);
    
    gpio_set_level(PIN_MOTOR_DIR, HIGH);
    gpio_set_level(PIN_MOTOR_RESET, HIGH);

    /* mcpwm gpio initialization */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_MOTOR_STEP);
    
    /* Initial mcpwm configuration */
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 100;
    pwm_config.cmpr_a = 50;     // Duty cycle of PWMxA = 50
    pwm_config.cmpr_b = 0;      // Duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    /* Configure PWM0A & PWM0B with above settings */
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    /* Stop PWM signal */
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
}


/*!
 * @brief This public function is used to .
 *
 * @param[in,out] team_dest  :Structure instance of team_info_t
 * @param[in] team_source    :Structure instance of team_info_t.
 *
 * @return Nothing.
 */
void team_info_update(team_info_t *team_dest, team_info_t team_source)
{
    /* Check if there is more units or less units */
    if (UNITS_PLUS == team_source.units) {
        /* Make sure units counter doesn't exceed the max limit */
        if (MAX_UNITS > team_dest[team_source.id].units_counter) {
            /* Increase units counter */
            team_dest[team_source.id].units_counter++;
            team_dest[team_source.id].capture_speed++;
        }
    }
    else if (UNITS_MINUS == team_source.units) {
        /* Make sure units counter doesn't exceed the min limit */
        if (MIN_UNITS < team_dest[team_source.id].units_counter) {
            /* Decrease units counter */
            team_dest[team_source.id].units_counter--;
            team_dest[team_source.id].capture_speed--;
        }
    }
}



/****************************************** App Core Tasks ********************************************/

/*!
 * @brief This internal task is used to..
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void beacon_parser_task(void *arg)
{
    esp_bd_addr_t beacon_addr_list[MAX_BEACONS] = {{0x24, 0x0A, 0xC4, 0xFA, 0x4A, 0xCA},
                                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                   {0x08, 0x3A, 0xF2, 0xAC, 0x68, 0xBA},
                                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
    beacon_dev_t beacon[MAX_BEACONS] = {0};
    beacon_dev_t beacon_temp = {0};
    uint8_t beacon_index = 0;
    team_info_t team = {0};
    
    esp_timer_create_args_t beacon_timer_args = {
        .callback = &beacon_timer_cb
    };

    for (beacon_index = 0; beacon_index < MAX_BEACONS; beacon_index++) {
        beacon_timer_args.arg = (void *)beacon_addr_list[beacon_index];
        esp_timer_create(&beacon_timer_args, &s_beacon_timer[beacon_index]);
    }

    /* Create beacon devices queue */
    s_beacon_queue = xQueueCreate(BEACON_QUEUE_LEN, sizeof(beacon_dev_t));
    
    while (1) {
        /* Dequeue beacon device */
        xQueueReceive(s_beacon_queue, &beacon_temp, portMAX_DELAY);

        /* DEBUG */
        //printf("RSSI: %d dbm\n", beacon_temp.rssi);
        //printf("Distance: %.2f m\n\n", pow(10, (((-55.0f) - beacon_temp.rssi) / 20.0f)));

        /* Search through all beacons devices */
        for (beacon_index = 0; beacon_index < MAX_BEACONS; beacon_index++) {
            /* Find the corresponding beacon within the address list */
            if (0 == memcmp(beacon_addr_list[beacon_index], beacon_temp.address, ESP_BD_ADDR_LEN)) {
                /* Check if beacon still advertising */
                if (BEACON_ABSENT == beacon_temp.proximity) {
                    printf("**************************\n");
                    printf("Beacon[%d] absent\n\n", beacon_index);
                    beacon[beacon_index].proximity = BEACON_ABSENT;
                    /* Reset RSSI value and counter */
                    beacon[beacon_index].rssi = 0;
                    beacon[beacon_index].rssi_counter = 0;
                    /* Enqueue team info */
                    team.id = beacon_index < 5 ? TEAM_A : TEAM_B;
                    team.units = UNITS_MINUS;
                    xQueueSend(s_team_info_queue, &team, 0);
                }
                else if (BEACON_ABSENT != beacon_temp.proximity) {
                    /* Stop timer of this beacon since we know it's still advertising */
                    esp_timer_stop(s_beacon_timer[beacon_index]);

                    /* Sum RSSI value and increase counter */
                    beacon[beacon_index].rssi += beacon_temp.rssi;
                    beacon[beacon_index].rssi_counter++;

                    /* RSSI values ready to be averaged */
                    if (MAX_RSSI_COUNT == beacon[beacon_index].rssi_counter) {
                        /* Average RSSI values */
                        beacon[beacon_index].rssi = (float)beacon[beacon_index].rssi / (float)beacon[beacon_index].rssi_counter;

                        /* Determine if the beacon is in range or not */
                        if (RSSI_RADIUS <= beacon[beacon_index].rssi) {
                            /* Beacon in range.
                             * Make sure beacon is not already in range so this    
                             * is run only on a proximity status change */
                            if (BEACON_IN_RANGE != beacon[beacon_index].proximity) {
                                printf("**************************\n");
                                printf("beacon[%d] in range\n\n", beacon_index);
                                beacon[beacon_index].proximity = BEACON_IN_RANGE;
                                /* Enqueue team info */
                                team.id = beacon_index < 5 ? TEAM_A : TEAM_B;
                                team.units = UNITS_PLUS;
                                xQueueSend(s_team_info_queue, &team, 0);
                            }
                        }
                        else {
                            /* Beacon out of range.
                             * Make sure beacon is not already out of range so this    
                             * is run only on a proximity status change */
                            if (BEACON_OUT_RANGE != beacon[beacon_index].proximity) {
                                printf("**************************\n");
                                printf("beacon[%d] out of range\n\n", beacon_index);
                                beacon[beacon_index].proximity = BEACON_OUT_RANGE;
                                /* Enqueue team info */
                                team.id = beacon_index < 5 ? TEAM_A : TEAM_B;
                                team.units = UNITS_MINUS;
                                xQueueSend(s_team_info_queue, &team, 0);
                            }
                        }
                        /* Reset RSSI value and counter */
                        beacon[beacon_index].rssi = 0;
                        beacon[beacon_index].rssi_counter = 0;
                    }
                    /* Set 1 second alarm for this beacon to know if it's still advertising */
                    esp_timer_start_once(s_beacon_timer[beacon_index], 1000000);
                }
                break;  // Beacon device found, break from for() loop
            }
        }
    }
}


/*!
 * @brief This internal task is used to control the position of the flags.
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void cp_ctrl_task(void *arg)
{
    team_id_t current_team_id;
    team_id_t opposing_team_id;
    team_info_t team[MAX_TEAMS] = {0};
    team_info_t team_temp = {0};
    flag_position_t flag_pos = POSITION_START;
    char lora_packet[LORA_PACKET_LEN] = {0};

    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();
    
    stepper_init();

    /* Put flag to starting position (Quick rotation) */
    // TODO

    /* Create Team info queue */
    s_team_info_queue = xQueueCreate(TEAM_INFO_QUEUE_LEN, sizeof(team_info_t));

    while (1) {
        /* Wait for new Team info then dequeue */
        xQueueReceive(s_team_info_queue, &team_temp, portMAX_DELAY);

        /* Update Team info */
        team_info_update(team, team_temp);
        
        /* Store Team ids */
        current_team_id = team_temp.id;
        opposing_team_id = !team_temp.id;

        if (true == team_temp.b_cp_captured && 0 == team[opposing_team_id].units_counter) {
            /* The current team has captured the CP (Save CP capture status) */
            team[current_team_id].b_cp_captured = team_temp.b_cp_captured;
            team[opposing_team_id].b_cp_captured = !team_temp.b_cp_captured;
            
            /* Save new flag position */
            flag_pos = (current_team_id == TEAM_A ? POSITION_A : POSITION_B);

            sprintf(lora_packet, "cp_id:%d, cp_status:%d, dominant_team:%d", CP_A, CP_CAPTURED, current_team_id);
            lora_send_packet((uint8_t *)lora_packet, sizeof(lora_packet));
        }
        else {
            /* Before raising current team flag, make sure :
             *  - There are still current team units in range 
             *  - There are no opposing team units in range
             *  - The flag position is not already on current team flag_pos
             * */
            if (0 < team[current_team_id].units_counter &&
                0 == team[opposing_team_id].units_counter &&
                (flag_position_t)current_team_id != flag_pos) {
                
                /* Raise flag */
                gpio_set_level(PIN_MOTOR_DIR,
                              (current_team_id == TEAM_A ? MOTOR_DIR_CW : MOTOR_DIR_CCW));
                mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0,
                                    stepper_freq[team[current_team_id].capture_speed]);
                mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

                /* The opposing team is loosing the CP (Save CP new capture status) */
                team[opposing_team_id].b_cp_captured = false;

                /* Save new flag position (in motion so NONE)*/
                flag_pos = POSITION_NONE;

                sprintf(lora_packet, "cp_id:%d, cp_status:%d, dominant_team:%d", CP_A, CP_CAPTURING, current_team_id);
                lora_send_packet((uint8_t *)lora_packet, sizeof(lora_packet));
            }
            else {
                /* Resume opposing team flag rotation if :
                 *  - There are still opposing team units in range  
                 *  - There are no current team units in range
                 *  - The flag position is not already on opposing team flag_pos 
                 * */
                if (0 < team[opposing_team_id].units_counter &&
                    0 == team[current_team_id].units_counter &&
                    (flag_position_t)opposing_team_id != flag_pos) {

                    /* Raise flag */
                    gpio_set_level(PIN_MOTOR_DIR,
                                  (opposing_team_id == TEAM_B ? MOTOR_DIR_CCW : MOTOR_DIR_CW));
                    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0,
                                        stepper_freq[team[opposing_team_id].capture_speed]);
                    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

                    /* The current team is loosing the CP (Save CP new capture status) */
                    team[current_team_id].b_cp_captured = false;

                    /* Save new flag position (in motion so NONE)*/
                    flag_pos = POSITION_NONE;

                    sprintf(lora_packet, "cp_id:%d, cp_status:%d, dominant_team:%d", CP_A, CP_CAPTURING, opposing_team_id);
                    lora_send_packet((uint8_t *)lora_packet, sizeof(lora_packet));
                }
                else {
                    /* Stop flag rotation (Both team units are near the CP) */
                    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                }
            }
        }


        /* Debug */
        printf("TEAM-A Current info:\n");
        printf("  Units counter = %d\n", team[TEAM_A].units_counter);
        printf("  Capture speed = %d\n", team[TEAM_A].capture_speed);
        printf("  CP captured ? = %s\n\n", team[TEAM_A].b_cp_captured == true ? "YES" : "NO");
        printf("TEAM-B Current info:\n");
        printf("  Units counter = %d\n", team[TEAM_B].units_counter);
        printf("  Capture speed = %d\n", team[TEAM_B].capture_speed);
        printf("  CP captured ? = %s\n\n", team[TEAM_B].b_cp_captured == true ? "YES" : "NO");
        printf("**************************\n\n");
    }
}



/*!
 * @brief This internal task is used to .
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void limit_switch_task(void *arg)
{
    uint32_t limit_switch_num = 0;
    team_info_t team = {0};
    
    /* Create queue for limit switches events */
    s_limit_switch_queue = xQueueCreate(1, sizeof(uint32_t));

    /* Set inpout mode for limit switch Pins */
    gpio_set_direction(PIN_LIMIT_SWITCH_1, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_LIMIT_SWITCH_2, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_LIMIT_SWITCH_3, GPIO_MODE_INPUT);
    
    /* Set rising edge interrupt type for limit switch Pins */
    gpio_set_intr_type(PIN_LIMIT_SWITCH_1, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(PIN_LIMIT_SWITCH_2, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(PIN_LIMIT_SWITCH_3, GPIO_INTR_POSEDGE);

    /* Setup ISR for limit switch Pins */
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(PIN_LIMIT_SWITCH_1, limit_switch_isr, (void*) PIN_LIMIT_SWITCH_1);
    gpio_isr_handler_add(PIN_LIMIT_SWITCH_2, limit_switch_isr, (void*) PIN_LIMIT_SWITCH_2);
    gpio_isr_handler_add(PIN_LIMIT_SWITCH_3, limit_switch_isr, (void*) PIN_LIMIT_SWITCH_3);

    while (1) {
        /* Wait until a limit switch is reached */
        xQueueReceive(s_limit_switch_queue, &limit_switch_num, portMAX_DELAY);
        
        /* Stop flag rotation */
        mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

        switch (limit_switch_num) {
            case PIN_LIMIT_SWITCH_1:
                team.id = TEAM_A;
                printf("Limit switch TEAM-A\n\n");
                break;
            case PIN_LIMIT_SWITCH_2:
                team.id = TEAM_NONE;
                printf("Limit switch START\n\n");
                break;
            case PIN_LIMIT_SWITCH_3:
                team.id = TEAM_B;
                printf("Limit switch TEAM-B\n\n");
                break;
            default:
                break;
        }
        
        team.b_cp_captured = true;

        xQueueSend(s_team_info_queue, &team, 0);
    }
}



/****************************************** Pro Core Tasks ********************************************/

/*!
 * @brief The app_main task is used to..
 * 
 * @param  :Nothing.
 * 
 * @return Nothing.
 */
void app_main(void)
{
    printf("\n\nXstractiK Domination Project  -  CP-v%s\n\n", FIRMWARE_VERSION);

    /* Create a task for limit switches */
    xTaskCreatePinnedToCore(&limit_switch_task,
                            "Limit Switch Task",
                            LIMIT_SWITCH_TASK_STACK,
                            NULL,
                            LIMIT_SWITCH_TASK_PRIORITY,
                            NULL,
                            LIMIT_SWITCH_TASK_CORE);

    /* Create a task for flag control */
    xTaskCreatePinnedToCore(&cp_ctrl_task,
                            "CP ctrl task",
                            CP_CTRL_TASK_STACK,
                            NULL,
                            CP_CTRL_TASK_PRIORITY,
                            NULL,
                            CP_CTRL_TASK_CORE);

    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    
    esp_bluedroid_init();
    esp_bluedroid_enable();
    
    /* Create a task for.. */
    xTaskCreatePinnedToCore(&beacon_parser_task,
                            "Beacon parser task",
                            BEACON_PARSER_TASK_STACK,
                            NULL,
                            BEACON_PARSER_TASK_PRIORITY,
                            NULL,
                            BEACON_PARSER_TASK_CORE);
    
    esp_err_t status;

    //register the scan callback function to the gap module
    status = esp_ble_gap_register_callback(esp_gap_cb);
    if (status != ESP_OK) {
        printf("\ngap register error: %s\n", esp_err_to_name(status));
    }
    
    status = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (status != ESP_OK){
        printf("\nConfig adv data failed: %s\n", esp_err_to_name(status));
    }
    
    /* set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);
}


/******************************************************************************************************/