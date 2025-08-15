/*
 * lora_simple_beacon.h
 *
 * Simple LoRa beacon transmitter header
 */

#ifndef INC_LORA_SIMPLE_BEACON_H_
#define INC_LORA_SIMPLE_BEACON_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* LR1121 driver includes */
#include "../../Drivers/lr11xx_driver/src/lr11xx_radio.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_system.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_regmem.h"
#include "../../Drivers/lr11xx_driver/src/lr11xx_hal.h"

/* Configuration */
#define SIMPLE_PAYLOAD_LENGTH       7       // Fixed payload length for "TEST123"
#define TRANSMISSION_INTERVAL_MS    3000    // 3 seconds as requested

/* GPIO pin definitions - using actual LR1121 dev kit pins */
#define LED_TX_GPIO_Port            LR_LED_TX_GPIO_Port
#define LED_TX_Pin                  LR_LED_TX_Pin
#define LED_RX_GPIO_Port            LR_LED_RX_GPIO_Port
#define LED_RX_Pin                  LR_LED_RX_Pin
#define LED_STATUS_GPIO_Port        LR_LED_SCAN_GPIO_Port
#define LED_STATUS_Pin              LR_LED_SCAN_Pin

#define LR_RESET_GPIO_Port          LR_RESET_GPIO_Port
#define LR_RESET_Pin                LR_RESET_Pin

/* Function prototypes */

/**
 * @brief Initialize the simple beacon system
 */
void simple_beacon_init(void);

/**
 * @brief Main processing loop - call this continuously from main()
 */
void simple_beacon_process(void);

/**
 * @brief Get total number of packets transmitted
 * @return Number of packets sent
 */
uint32_t simple_beacon_get_packet_count(void);

/**
 * @brief Get device ID
 * @return Device ID from STM32 unique ID
 */
uint32_t simple_beacon_get_device_id(void);

#endif /* INC_LORA_SIMPLE_BEACON_H_ */
