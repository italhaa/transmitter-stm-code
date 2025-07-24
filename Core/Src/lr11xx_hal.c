/*!
 * \file      lr11xx_hal.c
 *
 * \brief     Implements the lr11xx radio HAL functions for this specific implementation.
 *
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "../../Drivers/lr11xx_driver/src/lr11xx_hal.h"
#include "main.h"

/*
 * -----------------------------------------------------------------------------
 * --- SYSTEMWIDE VARIABLES -----------------------------------------------------------
 */
extern SPI_HandleTypeDef hspi1;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE ENUM -----------------------------------------------------------
 */
enum {
  LR11XX_REGMEM_WRITE_REGMEM32_OC      = 0x0105,
  LR11XX_REGMEM_READ_REGMEM32_OC       = 0x0106,
  LR11XX_REGMEM_WRITE_MEM8_OC          = 0x0107,
  LR11XX_REGMEM_READ_MEM8_OC           = 0x0108,
  LR11XX_REGMEM_WRITE_BUFFER8_OC       = 0x0109,
  LR11XX_REGMEM_READ_BUFFER8_OC        = 0x010A,
  LR11XX_REGMEM_CLEAR_RXBUFFER_OC      = 0x010B,
  LR11XX_REGMEM_WRITE_REGMEM32_MASK_OC = 0x010C,
};

// typedef enum HAL_StatusTypeDef_e {
//   HAL_OK       = 0x00U,
//   HAL_ERROR    = 0x01U,
//   HAL_BUSY     = 0x02U,
//   HAL_TIMEOUT  = 0x03U
// } HAL_StatusTypeDef;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
const uint8_t dummy_byte     = LR11XX_NOP;
uint8_t dummy_byte_rx  = LR11XX_NOP;
HAL_StatusTypeDef status;
uint8_t response[256];
uint16_t spi_timeout = 200;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Wait until radio busy pin returns to 0
 */
void lr11xx_hal_wait_on_busy( const void* radio );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{	
    HAL_GPIO_WritePin(LR_RESET_GPIO_Port, LR_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay( 10 ); // 5
    HAL_GPIO_WritePin(LR_RESET_GPIO_Port, LR_RESET_Pin, GPIO_PIN_SET);

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_RESET);
    HAL_Delay( 5 );  // 1
    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    #if defined( USE_LR11XX_CRC_OVER_SPI )
        uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
        cmd_crc         = lr11xx_hal_compute_crc( cmd_crc, data, data_length );
    #endif

    lr11xx_hal_wait_on_busy( NULL );

    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_RESET);
  	status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) command, response, command_length, spi_timeout);

    // command filter - certain commands have data and 
    if ( !(command[0] == (uint8_t) (LR11XX_REGMEM_WRITE_BUFFER8_OC >> 8) && (uint8_t) (command[1] == LR11XX_REGMEM_WRITE_BUFFER8_OC >> 0))){
      // HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);
    }

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      Error_Handler();
    }

    if (data_length > 0) {
      // HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_RESET);

      status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) data, response, data_length, spi_timeout);

      /* Check the communication status */
      if(status != HAL_OK)
      {
        /* Execute user timeout callback */
        Error_Handler();
      }
    }
	
    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);
    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{

    lr11xx_hal_wait_on_busy( NULL );

    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_RESET);
  	status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) command, response, command_length, spi_timeout);
    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      Error_Handler();
    }

    if (data_length > 0) {

      uint8_t read_buffer_primer[256] = { 0x00 };
      uint8_t raw_read_buffer[256] = { 0x00 }; 

      lr11xx_hal_wait_on_busy( NULL );
     
      HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_RESET);
      status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) read_buffer_primer, raw_read_buffer, data_length + 1, spi_timeout);
      // status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) data, *raw_read_buffer, data_length + 1, spi_timeout);
      HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);

      for (uint8_t i = 1; i < data_length + 1; i++){
        *(data + i - 1)  = raw_read_buffer[i];
      }      

      /* Check the communication status */
      if(status != HAL_OK)
      {
        /* Execute user timeout callback */
        Error_Handler();
      }
    }

    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* radio, uint8_t* data, const uint16_t data_length )
{

    uint8_t read_buffer_primer[256] = { 0x00 };
    uint8_t raw_read_buffer[256] = { 0x00 }; 

    lr11xx_hal_wait_on_busy( NULL );

    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_RESET);
  	status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) read_buffer_primer, (uint8_t*) data, data_length, spi_timeout);
    HAL_GPIO_WritePin(LR_NSS_GPIO_Port, LR_NSS_Pin, GPIO_PIN_SET);

    /* Check the communication status */
    if(status != HAL_OK)
    {
      /* Execute user timeout callback */
      Error_Handler();
    }

    return LR11XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void lr11xx_hal_wait_on_busy( const void* radio )
{
    while( HAL_GPIO_ReadPin(LR_BUSY_GPIO_Port, LR_BUSY_Pin) == 1 ){
      HAL_Delay(2);
    } 
}

/* --- EOF ------------------------------------------------------------------ */
