/*******************************************************************************************************
**
** @brief     XstractiK Domination Project  -  CP-v0.3.0 Firmware
**
** @copyright Copyright © 2021 GHS-Tech. All rights reserved.
** 
** @file	  main.cpp
** @author    Souhail Ghafoud
** @date	  April 29, 2022
** @version	  0.3.0
**
*******************************************************************************************************/



/****************************************** Header includes *******************************************/

/* std Lib */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* FreeRTOS */
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

/* ESP32 */
#include "driver/gpio.h"        // GPIO
#include "driver/mcpwm.h"       // PWM
#include "soc/mcpwm_periph.h"   // PWM

/* Timer */
#include "esp_timer.h"

/* NVS */
#include "nvs_flash.h"

/* BLE */
#include "ble.h"

/* LoRa */
#include "lora.h"



/********************************************* Constants **********************************************/

#define FIRMWARE_VERSION                "0.3.0"                 // Firmware version

#define PRO_CORE                        0                       // ESP32 Core 0
#define APP_CORE                        1                       // ESP32 Core 1

#define OPTICAL_SWITCH_TASK_STACK       (1024 * 3)              // Stack size in bytes
#define OPTICAL_SWITCH_TASK_PRIORITY    (tskIDLE_PRIORITY + 3)  // Priority level
#define OPTICAL_SWITCH_TASK_CORE        APP_CORE                // CPU core ID

#define PLAYER_PARSER_TASK_STACK        (1024 * 3)              // Stack size in bytes
#define PLAYER_PARSER_TASK_PRIORITY     (tskIDLE_PRIORITY + 2)  // Priority level
#define PLAYER_PARSER_TASK_CORE         APP_CORE                // CPU core ID

#define CP_CTRL_TASK_STACK              (1024 * 3)              // Stack size in bytes
#define CP_CTRL_TASK_PRIORITY           (tskIDLE_PRIORITY + 1)  // Priority level
#define CP_CTRL_TASK_CORE               APP_CORE                // CPU core ID

#define CP_ID                           0

#define MAX_RSSI_COUNT                  10
#define RSSI_RADIUS                     -65                     // dBm

#define MAX_PLAYERS                     10

#define MAX_TEAMS                       2
#define MAX_UNITS                       5
#define MIN_UNITS                       0

#define MAX_CAPTURE_SPEED               6

#define PLAYER_ABSENCE_QUEUE_LEN        MAX_PLAYERS             // Player absence Queue length
#define TEAM_INFO_QUEUE_LEN             10                      // Team info Queue length

#define SYS_INIT_EVENT_BIT              BIT0                    // System init EventBit

#define LORA_PACKET_LEN                 20

#define HIGH			                1
#define LOW 			                0

/* SPI pins */
#define PIN_SPI_MOSI                    GPIO_NUM_23
#define PIN_SPI_MISO                    GPIO_NUM_19
#define PIN_SPI_CLK                     GPIO_NUM_18

/* LoRa pins */
#define PIN_LORA_DIO0                   GPIO_NUM_27
#define PIN_LORA_RESET                  GPIO_NUM_14

/* Optical switch pins */
#define PIN_OPTICAL_SWITCH_1            GPIO_NUM_26             // Flag position Team_A
#define PIN_OPTICAL_SWITCH_2            GPIO_NUM_35             // Flag position Start
#define PIN_OPTICAL_SWITCH_3            GPIO_NUM_15             // Flag position Team_B

/* Stepper motor pins */
#define PIN_MOTOR_DIR                   GPIO_NUM_32
#define PIN_MOTOR_STEP                  GPIO_NUM_33
#define PIN_MOTOR_RESET                 GPIO_NUM_25

/* Stepper motor directions (Clockwise & Conterclockwise) */
#define MOTOR_DIR_CW			        HIGH
#define MOTOR_DIR_CCW 			        LOW



/************************************** Enumeration Definitions ***************************************/

/*!
 * @brief Player proximity state.
 */
typedef enum {
    PLAYER_ABSENT = 0,
    PLAYER_OUT_RANGE,
    PLAYER_IN_RANGE
} player_proximity_t;


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
    POSITION_START
} flag_position_t;


/*!
 * @brief CP capture state.
 */
typedef enum {
    CP_CAPTURED = 0,
    CP_CAPTURING
} cp_capture_state_t;


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
 * @brief Player information.
 */
typedef struct {
    esp_bd_addr_t address;
    player_proximity_t proximity;
    int rssi;
    uint8_t rssi_counter;
} player_info_t;


/*!
 * @brief Team information.
 */
typedef struct {
    team_id_t id;
    team_units_t units;
    uint8_t units_counter;
    uint8_t capture_speed;
    bool b_cp_captured;
} team_info_t;



/***************************************** Static Variables *******************************************/

static QueueHandle_t s_player_absence_queue = NULL;                 // Player absent Queue handle
static QueueHandle_t s_team_info_queue = NULL;                      // Team info Queue handle
static QueueHandle_t s_optical_switch_queue = NULL;                 // Optical switch Queue handle


static EventGroupHandle_t s_sys_init_event_group = NULL;            // System init EventGroup handle


static esp_timer_handle_t s_player_adv_timer[MAX_PLAYERS] = {0};


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


static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};


esp_bd_addr_t player_addr_list[MAX_PLAYERS] = {{0x24, 0x0A, 0xC4, 0xFA, 0x4A, 0xCA},
                                               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                               {0x08, 0x3A, 0xF2, 0xAC, 0x68, 0xBA},
                                               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};


static stepper_freq_t stepper_freq_hz[MAX_CAPTURE_SPEED] = {
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
 * @param[in] arg  :Optical switch GPIO number.
 * 
 * @return Nothing.
 */
static void optical_switch_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;      // Higher priority task flag
    gpio_num_t optical_switch_num = (gpio_num_t) arg;

    xQueueSendFromISR(s_optical_switch_queue, &optical_switch_num, &xHigherPriorityTaskWoken);

    /* Wake up higher priority task immediately */
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



/***************************************** Private Functions ******************************************/

/*!
 * @brief This private function is used as a callback for timer alarms. 
 *
 * @param[in] arg  :Structure instance of player_info_t.
 * 
 * @return Nothing.
 */
static void player_absence_timer_cb(void *arg)
{
    esp_bd_addr_t player_addr = {0};

    //printf("Timer player_addr[0] = %X\n", *(*(esp_bd_addr_t *)arg));

    /* Enqueue player's address for player_parser_task */
    memcpy(player_addr, (esp_bd_addr_t *)arg, sizeof(esp_bd_addr_t));
    xQueueSend(s_player_absence_queue, &player_addr, 0);
}


/*!
 * @brief This private function is used to delay a task for a period
 *        of time in milliseconds.
 */
static void delay_ms(uint32_t period_ms)
{
    vTaskDelay((period_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}



/***************************************** Public Functions *******************************************/

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
 * @param[in,out] team_dest  :Structure instance of team_info_t.
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
static void player_parser_task(void *arg)
{
    ble_scan_result_t ble_scan_result = {0};
    player_info_t player[MAX_PLAYERS] = {0};
    esp_bd_addr_t player_addr = {0};
    uint8_t player_index = 0;
    bool b_player_found = false;
    team_info_t team = {0};
    
    /* Player advertising timer callback */
    esp_timer_create_args_t player_absence_timer_args = {
        .callback = &player_absence_timer_cb
    };

    /* Create a timer for each player */
    for (uint8_t player_index = 0; player_index < MAX_PLAYERS; player_index++) {
        player_absence_timer_args.arg = (void *)player_addr_list[player_index];
        esp_timer_create(&player_absence_timer_args, &s_player_adv_timer[player_index]);
    }

    /* Create a queue for player absence event */
    s_player_absence_queue = xQueueCreate(PLAYER_ABSENCE_QUEUE_LEN, sizeof(esp_bd_addr_t));
    
    /* Wait until system init is done */
    xEventGroupWaitBits(s_sys_init_event_group, SYS_INIT_EVENT_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* Start scanning continiously for nearby players */
    if (ESP_OK != ble_start_scanning(0)) {
        printf("ERROR : BLE start scan failed\n\n");
    }
    
    while (1) {
        /********************************* Absent Players *********************************/

        /* Check if a player adv timer timed out (not receiving advertising data anymore) */
        if (xQueueReceive(s_player_absence_queue, player_addr, 0))
        {
            /* Search through all players */
            for (player_index = 0; 
                 b_player_found != true && player_index < MAX_PLAYERS;
                 player_index++) 
            {                
                /* Find the corresponding player within the address list */
                if (0 == memcmp(player_addr_list[player_index],
                                player_addr,
                                ESP_BD_ADDR_LEN))
                {
                    /* Update player's poximity status */
                    player[player_index].proximity = PLAYER_ABSENT;
                    /* Reset RSSI and counter values */
                    player[player_index].rssi = 0;
                    player[player_index].rssi_counter = 0;
                    /* Enqueue team info for cp_ctrl_task */
                    team.id = player_index < 5 ? TEAM_A : TEAM_B;
                    team.units = UNITS_MINUS;
                    xQueueSend(s_team_info_queue, &team, 0);
                    /* Set flag to exit for loop */
                    b_player_found = true;
                }
            }
            /* Reset flag */
            b_player_found = false;
        }
        

        /********************************* Nearby Players *********************************/

        /* Get latest scan of nearby players */
        ble_get_scan_result(&ble_scan_result);

        /* Search through all players */
        for (player_index = 0;
             b_player_found != true && player_index < MAX_PLAYERS;
             player_index++)
        {
            /* Find the corresponding player within the address list */
            if (0 == memcmp(player_addr_list[player_index],
                            ble_scan_result.address,
                            ESP_BD_ADDR_LEN))
            {
                /* Stop timer of this player since we know he's still advertising */
                esp_timer_stop(s_player_adv_timer[player_index]);

                /* Sum RSSI value and increase counter */
                player[player_index].rssi += ble_scan_result.rssi;
                player[player_index].rssi_counter++;

                /* RSSI values ready to be averaged */
                if (MAX_RSSI_COUNT == player[player_index].rssi_counter)
                {
                    /* Average RSSI values */
                    player[player_index].rssi = (float)player[player_index].rssi /
                                                (float)player[player_index].rssi_counter;

                    /* Determine if the player is in range or not */
                    if (RSSI_RADIUS <= player[player_index].rssi) {
                        /* Player in range.
                         * Make sure player is not already in range so 
                         * this is run only on a proximity status change.
                         * */
                        if (PLAYER_IN_RANGE != player[player_index].proximity) {
                            /* Update player's poximity status */
                            player[player_index].proximity = PLAYER_IN_RANGE;
                            /* Enqueue team info for cp_ctrl_task */
                            team.id = player_index < 5 ? TEAM_A : TEAM_B;
                            team.units = UNITS_PLUS;
                            xQueueSend(s_team_info_queue, &team, 0);
                        }
                    }
                    else {
                        /* Player out of range.
                         * Make sure player is not already out of range so   
                         * this is run only on a proximity status change.
                         * */
                        if (PLAYER_OUT_RANGE != player[player_index].proximity) {
                            /* Update player's poximity status */
                            player[player_index].proximity = PLAYER_OUT_RANGE;
                            /* Enqueue team info for cp_ctrl_task */
                            team.id = player_index < 5 ? TEAM_A : TEAM_B;
                            team.units = UNITS_MINUS;
                            xQueueSend(s_team_info_queue, &team, 0);
                        }
                    }
                            
                    /* DEBUG *//*
                    printf("RSSI: %d dbm\n", player[player_index].rssi);
                    printf("Distance: %.2f m\n\n",
                            pow(10, (((-55.0f) - player[player_index].rssi) / 20.0f)));*/

                    /* Reset RSSI value and counter */
                    player[player_index].rssi = 0;
                    player[player_index].rssi_counter = 0;
                }
                /* Set 1 second alarm for this player to monitor his advertising status */
                esp_timer_start_once(s_player_adv_timer[player_index], 1000000);

                /* Set flag to exit for loop */
                b_player_found = true;
            }
        }        
        b_player_found = false;     // Reset flag
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
    
    /* Create Team info queue */
    s_team_info_queue = xQueueCreate(TEAM_INFO_QUEUE_LEN, sizeof(team_info_t));

    /* Init LoRa module */
    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();

    /* Flag position should be at starting position */
    if (LOW != gpio_get_level(PIN_OPTICAL_SWITCH_2)){
        /* Start flag rotation */
        gpio_set_level(PIN_MOTOR_DIR, MOTOR_DIR_CW);
        mcpwm_set_frequency(MCPWM_UNIT_0,
                            MCPWM_TIMER_0,
                            stepper_freq_hz[MAX_CAPTURE_SPEED-1]);
        mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

        /* Wait until flag reaches starting position */
        while (TEAM_NONE != team_temp.id) {
            xQueueReceive(s_team_info_queue, &team_temp, portMAX_DELAY);
        }

        /* Stop flag rotation */
        mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    }
    
    /* Set SYS_INIT_EVENT_BIT */
    xEventGroupSetBits(s_sys_init_event_group, SYS_INIT_EVENT_BIT);

    while (1) {
        /* Wait for new Team info then dequeue */
        xQueueReceive(s_team_info_queue, &team_temp, portMAX_DELAY);

        /* Update Team info */
        team_info_update(team, team_temp);
        
        /* Store Team IDs */
        current_team_id = team_temp.id;
        opposing_team_id = !team_temp.id;

        /* Process only TEAM_A and TEAM_B */
        if (TEAM_NONE != team_temp.id) {
            /* The current team has captured the CP */
            if (true == team_temp.b_cp_captured && 
                0 == team[opposing_team_id].units_counter &&
                false == team[current_team_id].b_cp_captured)
            {
                /* Save CP capture status */
                team[current_team_id].b_cp_captured = team_temp.b_cp_captured;
                team[opposing_team_id].b_cp_captured = !team_temp.b_cp_captured;
                
                /* Save new flag position */
                flag_pos = (current_team_id == TEAM_A ? POSITION_A : POSITION_B);

                /* Notify players of CP capture status via LoRa comms */
                sprintf(lora_packet, "CP-v%s,%d,%d,%d", FIRMWARE_VERSION,
                                                        CP_ID,
                                                        CP_CAPTURED,
                                                        current_team_id);
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
                    (flag_position_t)current_team_id != flag_pos)
               {                                        
                    /* Raise flag */
                    gpio_set_level(PIN_MOTOR_DIR,
                                   (current_team_id == TEAM_A ? MOTOR_DIR_CW :
                                                                MOTOR_DIR_CCW));
                    mcpwm_set_frequency(MCPWM_UNIT_0,
                                        MCPWM_TIMER_0,
                                        stepper_freq_hz[team[current_team_id].capture_speed]);
                    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

                    /* The opposing team is loosing the CP (Save CP new capture status) */
                    team[opposing_team_id].b_cp_captured = false;

                    /* Notify players of CP capture status via LoRa comms */
                    sprintf(lora_packet, "CP-v%s,%d,%d,%d", FIRMWARE_VERSION,
                                                            CP_ID,
                                                            CP_CAPTURING,
                                                            current_team_id);
                    lora_send_packet((uint8_t *)lora_packet, sizeof(lora_packet));
                }
                else {
                    /* Resume opposing team flag rotation if :
                    *  - There are still opposing team units in range  
                    *  - There are no current team units in range
                    * */
                    if (0 < team[opposing_team_id].units_counter &&
                        0 == team[current_team_id].units_counter)
                    {
                        /* Raise flag */
                        gpio_set_level(PIN_MOTOR_DIR,
                                       (opposing_team_id == TEAM_B ? MOTOR_DIR_CCW :
                                                                     MOTOR_DIR_CW));
                        mcpwm_set_frequency(MCPWM_UNIT_0,
                                            MCPWM_TIMER_0,
                                            stepper_freq_hz[team[opposing_team_id].capture_speed]);
                        mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

                        /* The current team is loosing the CP (Save CP new capture status) */
                        team[current_team_id].b_cp_captured = false;

                        /* Notify players of CP capture status via LoRa comms */
                        sprintf(lora_packet, "CP-v%s,%d,%d,%d", FIRMWARE_VERSION,
                                                                CP_ID,
                                                                CP_CAPTURING,
                                                                opposing_team_id);
                        lora_send_packet((uint8_t *)lora_packet, sizeof(lora_packet));
                    }
                    else {
                        if (0 == team[current_team_id].units_counter &&
                            0 == team[opposing_team_id].units_counter)
                        {
                            /* Start rotating flag to previous position */
                            if (POSITION_A == flag_pos) {
                                /* Position A */
                                gpio_set_level(PIN_MOTOR_DIR, MOTOR_DIR_CCW);
                            }
                            else if (POSITION_B == flag_pos) {
                                /* Position B */
                                gpio_set_level(PIN_MOTOR_DIR, MOTOR_DIR_CW);
                            }
                            else if (POSITION_START == flag_pos) {
                                /* Position Start */
                                gpio_set_level(PIN_MOTOR_DIR,
                                               (opposing_team_id == TEAM_A ? MOTOR_DIR_CCW :
                                                                             MOTOR_DIR_CW));
                            }
                            mcpwm_set_frequency(MCPWM_UNIT_0,
                                                MCPWM_TIMER_0,
                                                stepper_freq_hz[MAX_CAPTURE_SPEED-1]);
                            mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
                        }
                        else if (0 < team[current_team_id].units_counter &&
                                 0 < team[opposing_team_id].units_counter)
                        {
                            /* Stop flag rotation (Both team units are near the CP) */
                            mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                        }
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
            printf("******************************************\n\n");
        }
    }
}


/*!
 * @brief This internal task is used to .
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void optical_switch_task(void *arg)
{
    gpio_num_t optical_switch_num;
    team_info_t team = {0};
    
    /* Create queue for optical switches events */
    s_optical_switch_queue = xQueueCreate(1, sizeof(gpio_num_t));

    /* Set input mode for optical switch Pins */
    gpio_set_direction(PIN_OPTICAL_SWITCH_1, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_OPTICAL_SWITCH_2, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_OPTICAL_SWITCH_3, GPIO_MODE_INPUT);
    
    /* Set falling edge interrupt type for optical switch Pins */
    gpio_set_intr_type(PIN_OPTICAL_SWITCH_1, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(PIN_OPTICAL_SWITCH_2, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(PIN_OPTICAL_SWITCH_3, GPIO_INTR_NEGEDGE);

    /* Setup ISR for optical switch Pins */
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(PIN_OPTICAL_SWITCH_1,
                         optical_switch_isr,
                         (void*) PIN_OPTICAL_SWITCH_1);
    gpio_isr_handler_add(PIN_OPTICAL_SWITCH_2,
                         optical_switch_isr,
                         (void*) PIN_OPTICAL_SWITCH_2);
    gpio_isr_handler_add(PIN_OPTICAL_SWITCH_3,
                         optical_switch_isr,
                         (void*) PIN_OPTICAL_SWITCH_3);

    while (1) {
        /* Wait until a optical switch is reached */
        xQueueReceive(s_optical_switch_queue, &optical_switch_num, portMAX_DELAY);

        /* Disable interrupt of current optical switch Pin */
        gpio_intr_disable(optical_switch_num);

        /* Parse optical switch num */
        switch (optical_switch_num) {
            case PIN_OPTICAL_SWITCH_1:
                /* Flag reached TEAM_A position */
                mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);    // Stop flag rotation
                team.id = TEAM_A;                           // Save Team ID
                team.b_cp_captured = true;                  // Save capture state
                break;
            case PIN_OPTICAL_SWITCH_2:
                /* Flag reached starting position */
                team.id = TEAM_NONE;                        // Dummy save
                team.b_cp_captured = false;                 // Dummy save
                break;
            case PIN_OPTICAL_SWITCH_3:
                /* Flag reached TEAM_B position */
                mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);    // Stop flag rotation
                team.id = TEAM_B;                           // Save Team ID
                team.b_cp_captured = true;                  // Save capture state
                break;
            default:
                break;
        }

        /* Enqueue team info for cp_ctrl_task */
        xQueueSend(s_team_info_queue, &team, 0);

        /************************* Debounce optical switch *************************/

        delay_ms(200);  // Small delay to avoid the noise in the signal

        /* Enable interrupt of current optical switch Pin */
        gpio_intr_enable(optical_switch_num);
        
        if (LOW == gpio_get_level(optical_switch_num)){
            /* Set rising edge interrupt type for current optical switch */
            gpio_set_intr_type(optical_switch_num, GPIO_INTR_POSEDGE);

            /* Wait until a optical switch is clear */
            xQueueReceive(s_optical_switch_queue, &optical_switch_num, portMAX_DELAY);

            delay_ms(200);  // Small delay to avoid the noise in the signal
        
            /* Set falling edge interrupt type for current optical switch */
            gpio_set_intr_type(optical_switch_num, GPIO_INTR_NEGEDGE);
        }

        /* Empty queue from dummy ISR reading caused by optical switch bouncing */
        if (0 < uxQueueMessagesWaiting(s_optical_switch_queue)) {
            xQueueReceive(s_optical_switch_queue, &optical_switch_num, 0);
        }
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
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    printf("******************************************\n");
    printf("XstractiK Domination Project  -  CP-v%s\n", FIRMWARE_VERSION);
    printf("******************************************\n\n");

    /* Init NVS */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Init BLE radio */
    ble_init(ble_scan_params, raw_adv_data);

    /* Init stepper motor */
    stepper_init();
        
    /* Create EventGroup for system Init */
    s_sys_init_event_group = xEventGroupCreate();

    /* Create a task for optical switch monitoring */
    xTaskCreatePinnedToCore(&optical_switch_task,
                            "Optical Switch Task",
                            OPTICAL_SWITCH_TASK_STACK,
                            NULL,
                            OPTICAL_SWITCH_TASK_PRIORITY,
                            NULL,
                            OPTICAL_SWITCH_TASK_CORE);

    /* Create a task for CP control */
    xTaskCreatePinnedToCore(&cp_ctrl_task,
                            "CP ctrl task",
                            CP_CTRL_TASK_STACK,
                            NULL,
                            CP_CTRL_TASK_PRIORITY,
                            NULL,
                            CP_CTRL_TASK_CORE);
    
    /* Create a task to parse players */
    xTaskCreatePinnedToCore(&player_parser_task,
                            "Player parser task",
                            PLAYER_PARSER_TASK_STACK,
                            NULL,
                            PLAYER_PARSER_TASK_PRIORITY,
                            NULL,
                            PLAYER_PARSER_TASK_CORE);
}


/******************************************************************************************************/