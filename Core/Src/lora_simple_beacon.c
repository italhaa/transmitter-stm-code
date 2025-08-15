/*
 * lora_simple_beacon.c
 *
 * Simple LoRa beacon transmitter - sends 150-byte packets every 3 seconds
 * No response handling, no complex protocol - just periodic transmission
 *
 * Created for LR1121 evaluation and testing
 */

#include "lora_simple_beacon.h"
#include "main.h"

/* External variables */
extern SPI_HandleTypeDef hspi1;

/* Private variables */
static uint32_t last_transmission_time = 0;
static uint32_t packet_counter = 0;
static uint8_t tx_buffer[SIMPLE_PAYLOAD_LENGTH];
static bool radio_initialized = false;

/* Device ID from STM32 unique ID */
static uint32_t device_id;

/* LoRa configuration - matching Pi receiver settings */
static const lr11xx_radio_mod_params_lora_t lora_mod_params = {
    .sf = LR11XX_RADIO_LORA_SF5,      // SF5
    .bw = LR11XX_RADIO_LORA_BW_500,   // 500 kHz
    .cr = LR11XX_RADIO_LORA_CR_4_7,   // 4/7
    .ldro = 0                         // Low Data Rate Optimization off
};

static const lr11xx_radio_pkt_params_lora_t lora_pkt_params = {
    .preamble_len_in_symb = 8,        // 8 symbols preamble
    .header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
    .pld_len_in_bytes = SIMPLE_PAYLOAD_LENGTH,
    .crc = LR11XX_RADIO_LORA_CRC_ON,
    .iq = LR11XX_RADIO_LORA_IQ_STANDARD
};

/* Private function prototypes */
static void init_radio(void);
static void prepare_packet(void);
static void transmit_packet(void);
static void update_leds(bool transmitting);

/**
 * @brief Initialize the simple beacon system
 */
void simple_beacon_init(void) {
    // Get device ID from STM32 unique ID
    device_id = HAL_GetUIDw0();
    
    // Initialize radio
    init_radio();
    
    // Prepare first packet
    prepare_packet();
    
    // Record initialization time
    last_transmission_time = HAL_GetTick();
    
    printf("Simple LoRa Beacon Initialized\n");
    printf("Device ID: 0x%08lX\n", device_id);
    printf("Transmitting fixed payload '%s' (%d bytes) every %d seconds at 2.4GHz\n", 
           "TEST123", SIMPLE_PAYLOAD_LENGTH, TRANSMISSION_INTERVAL_MS/1000);
}

/**
 * @brief Main processing loop - call this continuously
 */
void simple_beacon_process(void) {
    uint32_t current_time = HAL_GetTick();
    
    // Simple scan LED heartbeat (trigger once per second while idle)
    static uint32_t last_scan_led_time = 0;
    extern uint8_t enable_ui_scan; // from lora_base.c for LED handling
    if (current_time - last_scan_led_time >= 1000) {
        enable_ui_scan = 1;
        last_scan_led_time = current_time;
    }

    // Check if it's time to transmit
    if (current_time - last_transmission_time >= TRANSMISSION_INTERVAL_MS) {
        // Update packet data (fixed content)
        prepare_packet();

        // Transmit the packet
        transmit_packet();

        // Update timing
        last_transmission_time = current_time;
        packet_counter++;

        printf("Packet %lu transmitted (TEST123)\n", packet_counter);
    }
}

/**
 * @brief Initialize the LR1121 radio
 */
static void init_radio(void) {
    if (radio_initialized) {
        return;
    }
    
    // Reset the radio
    HAL_GPIO_WritePin(LR_RESET_GPIO_Port, LR_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LR_RESET_GPIO_Port, LR_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    
    // Initialize SPI communication
    lr11xx_hal_reset(NULL);
    HAL_Delay(100);
    
    // Set packet type to LoRa
    lr11xx_radio_set_pkt_type(NULL, LR11XX_RADIO_PKT_TYPE_LORA);
    
    // Set RF frequency to 2.444 GHz
    lr11xx_radio_set_rf_freq(NULL, 2444000000UL);
    
    // Set modulation parameters
    lr11xx_radio_set_lora_mod_params(NULL, &lora_mod_params);
    
    // Set packet parameters
    lr11xx_radio_set_lora_pkt_params(NULL, &lora_pkt_params);
    
    // Set sync word (0x11 to match Pi receiver)
    lr11xx_radio_set_lora_sync_word(NULL, 0x11);
    
    // Set TX power to +2 dBm
    lr11xx_radio_set_tx_params(NULL, 2, LR11XX_RADIO_RAMP_48_US);
    
    // Configure DIO for TX done interrupt
    lr11xx_system_set_dio_irq_params(NULL, 
                                    LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT,
                                    0);
    
    radio_initialized = true;
    printf("Radio initialized successfully\n");
}

/**
 * @brief Prepare the packet data
 */
static void prepare_packet(void) {
    // Fixed ASCII payload "TEST123"
    const char *msg = "TEST123"; // 7 bytes
    memcpy(tx_buffer, msg, SIMPLE_PAYLOAD_LENGTH);
}

/**
 * @brief Transmit the prepared packet
 */
static void transmit_packet(void) {
    // Clear any pending interrupts
    lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);
    
    // Load packet into radio buffer
    lr11xx_regmem_write_buffer8(NULL, tx_buffer, SIMPLE_PAYLOAD_LENGTH);
    
    // Turn on TX LED and flag UI tx event for blinking logic in main
    update_leds(true);
    extern uint8_t enable_ui_tx; // from lora_base.c
    enable_ui_tx = 1;
    
    // Start transmission
    lr11xx_radio_set_tx(NULL, 0);  // 0 = no timeout
    
    // Wait for transmission to complete (simple blocking approach)
    uint32_t timeout = HAL_GetTick() + 1000;  // 1 second timeout
    uint32_t irq_status = 0;
    
    while (HAL_GetTick() < timeout) {
        if (lr11xx_system_get_and_clear_irq_status(NULL, &irq_status) == LR11XX_STATUS_OK) {
            if (irq_status & LR11XX_SYSTEM_IRQ_TX_DONE) {
                break;  // Transmission completed
            }
            if (irq_status & LR11XX_SYSTEM_IRQ_TIMEOUT) {
                printf("TX Timeout!\n");
                break;
            }
        }
        HAL_Delay(1);
    }
    
    // Turn off TX LED
    update_leds(false);
}

/**
 * @brief Update LED status
 */
static void update_leds(bool transmitting) {
    if (transmitting) {
        // Turn on TX LED (assuming green LED)
        HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);
        // Turn off other LEDs during transmission
        HAL_GPIO_WritePin(LED_RX_GPIO_Port, LED_RX_Pin, GPIO_PIN_RESET);
    } else {
        // Turn off TX LED
        HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_RESET);
        // Show status LED (red) to indicate system active
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
    }
}

/**
 * @brief Get transmission statistics
 */
uint32_t simple_beacon_get_packet_count(void) {
    return packet_counter;
}

uint32_t simple_beacon_get_device_id(void) {
    return device_id;
}
