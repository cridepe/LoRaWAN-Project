/*
 * rfm95.h
 *
 * Code for RFM95 transceiver and STM32 boards.
 * This code has been rearranged starting from Henri Heimann's source code
 * available here: https://github.com/henriheimann/stm32-hal-rfm95
 *
 * Features added:
 * - Support for different SF (Receive windows adjusted accordingly)
 * - Support for adjustable power levels
 * - Support for SF9 in receive window 2
 * - Receive window timings have been adjusted compared to the original library (wrong calculations in the original code)
 * - Process mac commands extended: now all the commands specified in the LoRaWAN Link Layer are supported
 * - Added error detection mechanisms with 12 possible states to detect faulty behavior
 *
 * GitHub repo for this library: https://github.com/MarcoMiglio/STM32_RFM95
 *
 * TODO: - ADR mechanism
 *       - React to FPending bit set in FCtrl during downlinks
 *
 *  Created on: Nov 20, 2024
 *      Author: marcomiglio
 */

#ifndef INC_RFM95_H_
#define INC_RFM95_H_



#endif /* INC_RFM95_H_ */



#pragma once

#include <stdio.h>  // TODO additional for debug
#include <stdbool.h>
#include "stm32f2xx_hal.h"


// TODO: code for debug
extern UART_HandleTypeDef huart3;  // Declare huart2 as an external variable
#define RFM_DEBUG &huart3
int _write(int file, char *ptr, int len);


#ifndef RFM95_SPI_TIMEOUT
#define RFM95_SPI_TIMEOUT 10
#endif

#ifndef RFM95_WAKEUP_TIMEOUT
#define RFM95_WAKEUP_TIMEOUT 10
#endif

#ifndef RFM95_SEND_TIMEOUT
#define RFM95_SEND_TIMEOUT 2000
#endif

#ifndef RFM95_RECEIVE_TIMEOUT
#define RFM95_RECEIVE_TIMEOUT 2200
#endif

#define RFM95_EEPROM_CONFIG_MAGIC 0xab67

#define MAX_EIRP_EU 16

#define LOW_FREQ_BAND_EU868   863000000
#define HIGH_FREQ_BAND_EU868  870000000

#define RFM95_INTERRUPT_COUNT 3 // DIO0, DIO1, DIO5 interrupt events are used


// Set here base addresses for data transmission and reception
#define RFM95_FIFO_RX_BASE_ADDRESS 0x00
#define RFM95_FIFO_TX_BASE_ADDRESS 0x00

#define RFM95_MAX_MAC_PAYLOAD_LEN 222 + 1 + 7 + 1 + 4 // maximum frame payload for SF7 + 1byte MHDR + 7bytes FHDR + 1byte FPort + 4bytes MIC


/**
 * ACK - ERROR FLAGS:
 */

#define RFM95_STATUS_OK                   (1 << 0)  // 0x01   - Operation completed successfully
#define RFM95_STATUS_ERROR                (1 << 1)  // 0x02   - General error
#define RFM95_STATUS_RX_MAGIC_OK          (1 << 2)  // 0x04   - Received valid downlink magic
#define RFM95_STATUS_RX_MAGIC_ERR         (1 << 3)  // 0x08   - Received invalid downlink magic
#define RFM95_STATUS_RX_EMPTY             (1 << 4)  // 0x10   - No data received
#define RFM95_STATUS_RX_CRC_ERR           (1 << 5)  // 0x20   - CRC error in received data
#define RFM95_STATUS_SPI_ERROR            (1 << 6)  // 0x40   - SPI communication error
#define RFM95_STATUS_IRQ_TIMEOUT          (1 << 7)  // 0x80   - Timeout waiting for IRQ
#define RFM95_STATUS_MAC_CMD_ERR          (1 << 8)  // 0x100  - Error processing MAC commands
#define RFM95_STATUS_DECODE_ERR           (1 << 9)  // 0x200  - Error decoding payload
#define RFM95_STATUS_ACK_RECEIVED         (1 << 10) // 0x400  - ACK received during downlink
#define RFM95_STATUS_ENCODE_ERR           (1 << 11) // 0x800  - error while encoding
#define RFM95_STATUS_MAC_COMMANDS_PENDING (1 << 12) // 0x1000 - user data suspended to send pending MAC command answers


/*
 * This struct defines channel configuration:
 * usually uplink and downlink frequencies coincide during RX1, however the Network server
 * has the possibility to modify the downlink Freq. by using dedicated channels.
 *
 */
typedef struct {

  uint32_t frequency;
  uint32_t downlink_frequency;  // DlChannelReq cmd may modify Rx1 frequency for a certain channel

} rfm95_channel_config_t;


/*
 * This structure stores all the basic configurations for the RFM95 module and the LoRaWAN parameters
 * These values will be saved into dedicated EEPROM if save_config function is enabled.
 */
typedef struct {

  /**
   * MAGIC --> used to verify data integrity when saved into external EEPROM memory
   */
  uint16_t magic;

  /**
   * TRIGGER MAGIC --> to verify data intergity when receiving application data from the Application server
   */
  uint16_t lora_magic;

  /**
   * The current RX frame counter value.
   */
  uint16_t rx_frame_count;

  /**
   * The current TX frame counter value.
   */
  uint16_t tx_frame_count;

  /**
   *  Tx SF (7 to 12)
   */
  uint8_t tx_sf;

  /**
   * The delay to the RX1 window.
   */
  uint8_t rx1_delay;

  /**
   * The delay to the RX1 window.
   */
  uint8_t rx1_dr_offset;

  /**
   * Receive SF for rx2 (can be 9 or 12)
   */
  uint8_t rx2_sf;

  /**
   * Tx power (must be in range 2 -> 17 dBm)
   */
  uint8_t tx_power;

  /**
   * The configuration of channels;
   */
  rfm95_channel_config_t channels[16];

  /**
   * Mask defining which channels are enabled --> can be actively used by the network
   */
  uint16_t channel_mask;

  /**
   * Mask defining which channels are defined
   * --> have been shared by network and end-device, the channel mask specifies which ones
   *     can be used
   */
  uint16_t defined_channels;

} rfm95_eeprom_config_t;


// Not implemented here: function pointer to be executed after Interrupt have been enabled...
typedef void (*rfm95_on_after_interrupts_configured)();


/*
 * These two functions can be implemented to store and load configurations to and from the external memorization device (EEPROM, SD, ...)
 */
typedef bool (*rfm95_load_eeprom_config)(rfm95_eeprom_config_t *config);
typedef void (*rfm95_save_eeprom_config)(const rfm95_eeprom_config_t *config);


/*
 * Get precision tick is used for generating precise timing informations for precision sleep intervals
 * and for precise opening of receive windows.
 */
typedef uint32_t (*rfm95_get_precision_tick)();


/*
 * This function allows to sleep until the target time (RFM in sleep mode, MCU in stop mode 2)
 * with error reduced to the clock drift (less than 20 ppm for STM32 boards)
 */
typedef void (*rfm95_precision_sleep_until)(uint32_t ticks_target);


/*
 * This function is used to generate random number for channel selection.
 */
typedef uint8_t (*rfm95_random_int)(uint8_t max);


/*
 * This function is used to monitor battery level to respond DevStatusReq MAC commands.
 * (Notice that it is not mandatory to implement it...).
 */
typedef uint8_t (*rfm95_get_battery_level)();


/*
 * Define possible interrupt events registered into the dedicated interrupt buffer
 */
typedef enum
{
  RFM95_INTERRUPT_DIO0,
  RFM95_INTERRUPT_DIO1,
  RFM95_INTERRUPT_DIO5

} rfm95_interrupt_t;


/*
 * Define possible operating modes.
 * RFM95_RECEIVE_MODE_RX1_ONLY is NOT implemented in this code. If receive mode is enabled
 * both receive windows will always be used.
 */
typedef enum
{
  RFM95_RECEIVE_MODE_NONE,
  RFM95_RECEIVE_MODE_RX1_ONLY,
  RFM95_RECEIVE_MODE_RX12,
} rfm95_receive_mode_t;


/**
 * Structure defining a handle describing an RFM95(W) transceiver.
 */
typedef struct {

  /**
   * The handle to the SPI bus for the device.
   */
  SPI_HandleTypeDef *spi_handle;

  /**
   * The port of the NSS pin.
   */
  GPIO_TypeDef *nss_port;

  /**
   * The NSS pin.
   */
  uint16_t nss_pin;

  /**
   * The port of the RST pin.
   */
  GPIO_TypeDef *nrst_port;

  /**
   * The RST pin.
   */
  uint16_t nrst_pin;

  /**
   * The device address for the LoraWAN
   */
  uint8_t device_address[4];

  /**
   * The network session key for ABP activation with the LoraWAN
   */
  uint8_t network_session_key[16];

  /**
   * The application session key for ABP activation with the LoraWAN
   */
  uint8_t application_session_key[16];

  /**
   * The frequency of the precision tick in Hz.
   */
  uint32_t precision_tick_frequency;

  /**
   * The +/- timing drift per second in nanoseconds.
   */
  uint32_t precision_tick_drift_ns_per_s;

  /**
   * The receive mode to operate at.
   */
  rfm95_receive_mode_t receive_mode;

  /**
   * This flag is set when the ACK bit in a downlink is received, confirming a previously sent confirmed uplink request
   */
  bool uplink_acknowledged;

  /**
   * Flag set when confirmed answers are pending --> repeated until valid downlink is received
   */
  bool has_pending_answer;

  /**
   * Flag set to indicate that next uplink must set the ACK bit
   */
  bool confirmed_downlink;

  /**
   * Buffer containing MAC command answers that need confirmation
   */
  uint8_t answer_buff[51];

  /**
   * Answer buff length
   */
  uint8_t answer_buff_length;

  /**
   * Function provided that returns a precise tick for timing critical operations.
   */
  rfm95_get_precision_tick get_precision_tick;

  /**
   * Function that provides a precise sleep until a given tick count is reached.
   */
  rfm95_precision_sleep_until precision_sleep_until;

  /**
   * Function that provides a random integer.
   */
  rfm95_random_int random_int;

  /**
   * Function that returns the device's battery level.
   */
  rfm95_get_battery_level get_battery_level;

  /**
   * The load config function pointer can be used to load config values from non-volatile memory.
   * Can be set to NULL to skip.
   */
  rfm95_load_eeprom_config reload_config;

  /**
   * The save config function pointer can be used to store config values in non-volatile memory.
   * Can be set to NULL to skip.
   */
  rfm95_save_eeprom_config save_config;

  /**
   * Callback called after the interrupt functions have been properly configred;
   */
  rfm95_on_after_interrupts_configured on_after_interrupts_configured;

  /**
   * The config saved into the eeprom.
   */
  rfm95_eeprom_config_t config;

  /**
   * Tick values when each interrupt was called.
   */
  volatile uint32_t interrupt_times[RFM95_INTERRUPT_COUNT];

} rfm95_handle_t;


/*
 * Public functions signature --> User should rely only on these functions!
 */

uint16_t rfm95_init(rfm95_handle_t *handle);

bool rfm95_set_power(rfm95_handle_t *handle, int8_t power);

uint16_t rfm95_send_receive_cycle(rfm95_handle_t *handle, const uint8_t *send_data, size_t send_data_length, bool confirmed);

void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt);
