/*
 * rfm95.c
 *
 * Code for RFM95 transceiver and STM32 boards.
 * This code has been rearranged starting from Henri Heimann's source code
 * available here: https://github.com/henriheimann/stm32-hal-rfm95
 *
 * GitHub repo for this library: https://github.com/MarcoMiglio/STM32_RFM95
 *
 *  Created on: Nov 20, 2024
 *      Author: marcomiglio
 */


#include "rfm95.h"
#include "Encrypt_V31.h"

#include <assert.h>
#include <string.h>

#define RFM9x_VER 0x12

#define MY_DEBUG


#define RFM95_REGISTER_OP_MODE_SLEEP                            0x00
#define RFM95_REGISTER_OP_MODE_LORA_SLEEP                       0x80
#define RFM95_REGISTER_OP_MODE_LORA_STANDBY                     0x81
#define RFM95_REGISTER_OP_MODE_LORA_TX                          0x83
#define RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE                   0x86

#define RFM95_REGISTER_PA_DAC_LOW_POWER                         0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER                        0x87

#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE             0x40
#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE             0x00

#define RFM95_REGISTER_INVERT_IQ_1_TX                           0x27
#define RFM95_REGISTER_INVERT_IQ_2_TX                           0x1d

#define RFM95_REGISTER_INVERT_IQ_1_RX                           0x67
#define RFM95_REGISTER_INVERT_IQ_2_RX                           0x19



/**
 * Registers addresses.
 */
typedef enum
{
  RFM95_REGISTER_FIFO_ACCESS = 0x00,
  RFM95_REGISTER_OP_MODE = 0x01,
  RFM95_REGISTER_FR_MSB = 0x06,
  RFM95_REGISTER_FR_MID = 0x07,
  RFM95_REGISTER_FR_LSB = 0x08,
  RFM95_REGISTER_PA_CONFIG = 0x09,
  RFM95_REGISTER_LNA = 0x0C,
  RFM95_REGISTER_FIFO_ADDR_PTR = 0x0D,
  RFM95_REGISTER_FIFO_TX_BASE_ADDR = 0x0E,
  RFM95_REGISTER_FIFO_RX_BASE_ADDR = 0x0F,
  RFM95_REGISTER_IRQ_FLAGS = 0x12,
  RFM95_REGISTER_FIFO_RX_BYTES_NB = 0x13,
  RFM95_REGISTER_PACKET_SNR = 0x19,
  RFM95_REGISTER_MODEM_CONFIG_1 = 0x1D,
  RFM95_REGISTER_MODEM_CONFIG_2 = 0x1E,
  RFM95_REGISTER_SYMB_TIMEOUT_LSB = 0x1F,
  RFM95_REGISTER_PREAMBLE_MSB = 0x20,
  RFM95_REGISTER_PREAMBLE_LSB = 0x21,
  RFM95_REGISTER_PAYLOAD_LENGTH = 0x22,
  RFM95_REGISTER_MAX_PAYLOAD_LENGTH = 0x23,
  RFM95_REGISTER_MODEM_CONFIG_3 = 0x26,
  RFM95_REGISTER_INVERT_IQ_1 = 0x33,
  RFM95_REGISTER_SYNC_WORD = 0x39,
  RFM95_REGISTER_INVERT_IQ_2 = 0x3B,
  RFM95_REGISTER_DIO_MAPPING_1 = 0x40,
  RFM95_REGISTER_VERSION = 0x42,
  RFM95_REGISTER_PA_DAC = 0x4D
} rfm95_register_t;


/*
 *  This structure allows to define the byte to be written into the power config register of the RFM95
 *  while letting to visualize the single bits and their purpose.
 */
typedef struct
{
  union {
    struct {
      uint8_t output_power : 4;
      uint8_t max_power : 3;
      uint8_t pa_select : 1;
    };
    uint8_t buffer;
  };
} rfm95_register_pa_config_t;


/* Read RFM95 register using SPI protocol
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 * @param reg     rfm95_register_t specifier for one of the RFM95 internal registers
 * @param *buffer pointer to data buffer to be written into the register
 * @param length  size_t specifying amount of bytes to be written
 *
 * @return: false if an SPI error occurred, true otherwise
 */
static bool read_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t *buffer, size_t length)
{
  HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

  uint8_t transmit_buffer = (uint8_t)reg & 0x7fu;

  if (HAL_SPI_Transmit(handle->spi_handle, &transmit_buffer, 1, RFM95_SPI_TIMEOUT) != HAL_OK) {
    return false;
  }

  if (HAL_SPI_Receive(handle->spi_handle, buffer, length, RFM95_SPI_TIMEOUT) != HAL_OK) {
    return false;
  }

  HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

  return true;
}



/* Write to RFM95 register using SPI protocol
 *
 * @param *handle pointer to handle structure containing rfm95 configurations (refer to .h file for additional details
 *                about the structure).
 * @param reg     rfm95_register_t specifier for one of the RFM95 internal registers
 * @param value   uint8_t specifying byte to be written into the register
 *
 *
 * @return: false if an SPI error occurred, true otherwise
 */
static bool write_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t value)
{
  HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

  uint8_t transmit_buffer[2] = {((uint8_t)reg | 0x80u), value};

  if (HAL_SPI_Transmit(handle->spi_handle, transmit_buffer, 2, RFM95_SPI_TIMEOUT) != HAL_OK) {
    return false;
  }

  HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

  return true;
}



/* This internal function is used to deactivate a channel upon receiving the specific MAC command (NewChannelReq).
 * It sets the configured frequency to reset value (zero) for both uplink and downlink.
 * The specified channel index is additionally removed from defined_channl mask and active_channel mask.
 *
 * @param *handle       rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param channel_index uint8_t defining the channel index to be removed from the list.
 *
 * @return void.
 */
static void deactivate_channel(rfm95_handle_t *handle, uint8_t channel_index)
{
  assert(channel_index < 16);


  // New channel defined
  handle->config.channels[channel_index].frequency = 0;           // Set frequency to zero (flag to represent deactivated channels
  handle->config.channels[channel_index].downlink_frequency = 0;  // Deactivate also the downlink frequency for that channel
  handle->config.defined_channels &= ~((uint16_t)(1 << channel_index));

  // initially set channel enabled by default:
  handle->config.channel_mask &= ~((uint16_t)(1 << channel_index));
}



/* This internal function is used to activate a new channel upon receiving the specific MAC command (NewChannelReq).
 * It sets the configured frequency to the specified value for uplink.
 * Normally, downlink frequency for that channel is configured to the same value used for the uplink window.
 * However there might be situations in which Network server decides to use different frequencies for uplink and
 * downlink data transmission: see DlChannelReq MAC command for additional details.
 * Therefore, if not initialized, downlink frequency will be set to the same value used for the uplink window, otherwise downlink
 * configuration will be left to its current state.
 *
 * This function also sets defined channels mask and active channels mask.
 *
 * @param *handle       rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param channel_index uint8_t defining the channel index to be added to the channel list.
 * @param frequency     uint32_t defining the frequency for the required channel.
 *
 * @return void.
 */
static void config_set_channel(rfm95_handle_t *handle, uint8_t channel_index, uint32_t frequency)
{
  assert(channel_index < 16);


  // New channel defined
  handle->config.channels[channel_index].frequency = frequency;

  // if not specified Rx1 uses same frequency for uplink and downlink
  if(handle->config.channels[channel_index].downlink_frequency == 0){
    handle->config.channels[channel_index].downlink_frequency = frequency;
  }

  handle->config.defined_channels |= (1 << channel_index);

  // initially set channel enabled by default:
  handle->config.channel_mask |= (1 << channel_index);
}



/* This function is used during initialization process.
 * If no configurations are stored (external EEPROM), the default ones are used:
 * - resets up and down frame counters
 * - rx1 delay = 1 s, rx1 DR offset = 0
 * - rx2 SF = 12 (if not set to SF9 by the user)
 * - tx SF = 7 (if not specified by user)
 * - tx power = 17 dBm (if not specified by user)
 * - default channels (channel index 0, 1, 2): 868100000, 868300000, 868500000
 *   (Please notice that these channels should never be modified!)
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers.
 *
 * @return void.
 */
static void config_load_default(rfm95_handle_t *handle)
{
  handle->config.magic = RFM95_EEPROM_CONFIG_MAGIC;
  handle->config.tx_frame_count = 0;
  handle->config.rx_frame_count = 0;
  handle->config.rx1_delay = 1;
  handle->config.rx1_dr_offset = 0;
  handle->config.channel_mask = 0;

  if (handle->config.rx2_sf == 0) handle->config.rx2_sf = 12;
  if (handle->config.tx_sf == 0) handle->config.tx_sf = 7;

  if (handle->config.tx_power == 0) handle->config.tx_power = 17;

  config_set_channel(handle, 0, 868100000);
  config_set_channel(handle, 1, 868300000);
  config_set_channel(handle, 2, 868500000);
}


/* This function drives the RFM95 rest pin low to force an hardware reset.
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers.
 *
 * @return void.
 */
static void reset(rfm95_handle_t *handle)
{
  HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_RESET);
  HAL_Delay(1); // 0.1ms would theoretically be enough
  HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_SET);
  HAL_Delay(5);
}


/* This function allows to write the specified frequency into the RFM95 configuration registers
 * RFM95_REGISTER_FR_MSB, RFM95_REGISTER_FR_MID and RFM95_REGISTER_FR_LSB.
 *
 * @param *handle   rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param frequency uint32_t defining the frequency to be used.
 *
 *
 * @return: false if an SPI error occurred, true otherwise
 */
static bool configure_frequency(rfm95_handle_t *handle, uint32_t frequency) {
  // FQ = (FRF * 32 Mhz) / (2 ^ 19)
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  if (!write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
  if (!write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8))) return false;
  if (!write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0))) return false;

  return true;
}


/*
 * This function writes the configured frequency of a channel, as defined in the channel_mask,
 * into the RFM95 registers, enabling this channel for the next transmission.
 *
 * @param *handle       rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param channel_index size_t defining the channel index that will be used for the next transmission.
 *
 * @return: false if an SPI error occurred, true otherwise
 */
static bool configure_channel(rfm95_handle_t *handle, size_t channel_index){
  assert(handle->config.channel_mask & (1 << channel_index));
  return configure_frequency(handle, handle->config.channels[channel_index].frequency);
}


/* Wait for a single interrupt event. If exceeds the timeout threshold, the while execution
 * is interrupted.
 *
 * This function is used for interrupts on DIO5 pin (wait for the rfm95 module to be ready after setting new configurations)
 * and on DIO0 pin during transmission (wait for transmission done event).
 *
 * @param *handle    rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param interrupt  rfm95_interrupt_t defining the interrupt event to wait for (DIO0, DIO1, DIO5 supported in this version)
 * @param timeout_ms uint32_t defining the maximum timeout in ms.
 *
 * @return true if an interrupt is received within the specified timeout, false otherwise
 */
static bool wait_for_irq(rfm95_handle_t *handle, rfm95_interrupt_t interrupt, uint32_t timeout_ms) {
  uint32_t timeout_tick = handle->get_precision_tick() + timeout_ms * handle->precision_tick_frequency / 1000;

  while (handle->interrupt_times[interrupt] == 0) {
    if (handle->get_precision_tick() >= timeout_tick) {
      return false;
    }
  }

  return true;
}


/* Wait for interrupt events on pin DIO1 (No preamble detected) and DIO0 (Rx completed).
 * If exceeds the timeout threshold, the while execution is interrupted.
 *
 *
 * @param *handle    rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param timeout_ms uint32_t defining the maximum timeout in ms.
 *
 * @return: - RFM95_STATUS_IRQ_TIMEOUT if timeout threshold was exceeded during the two receive windows
 *          - RFM95_STATUS_RX_EMPTY if while execution is completed, but DIO1 was triggered (no preamble detected)
 *          - RFM95_STATUS_OK if while execution is completed with DIO0 set (Rx done)
 */
static uint16_t wait_for_rx_irqs(rfm95_handle_t *handle, uint16_t rx_timeout) {
  uint32_t timeout_tick = handle->get_precision_tick() +
                          rx_timeout * handle->precision_tick_frequency / 1000;

  while (handle->interrupt_times[RFM95_INTERRUPT_DIO0] == 0 && handle->interrupt_times[RFM95_INTERRUPT_DIO1] == 0) {
    if (handle->get_precision_tick() >= timeout_tick) {
      return RFM95_STATUS_IRQ_TIMEOUT;
    }
  }

  // this if is entered only if DIO1 interrupt is triggered (no preamble detected)
  if (handle->interrupt_times[RFM95_INTERRUPT_DIO0] == 0) return RFM95_STATUS_RX_EMPTY;

  // DIO0 set --> reception done correctly
  return RFM95_STATUS_OK;
}


/* This function sets the specified TX power.
 * RFM95 module has only the PAboost RF output connected, so the power range can be anything
 * in between 2 dBm and 17 dBm. High power configuration (+20 dBm) is also allowed, however in that
 * case duty cycle must be below 1% (limit current into the RF section and avoid excessive heating).
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param power int8_t defining the current power configuration.
 *
 * @return true if SPI write operation is correctly completed
 */
bool rfm95_set_power(rfm95_handle_t *handle, int8_t power) {
  assert((power >= 2 && power <= 17) || power == 20);

  rfm95_register_pa_config_t pa_config = {0};
  uint8_t pa_dac_config = 0;

  if (power >= 2 && power <= 17) {
    pa_config.max_power = 7;
    pa_config.pa_select = 1;
    pa_config.output_power = (power - 2);
    pa_dac_config = RFM95_REGISTER_PA_DAC_LOW_POWER;

  } else if (power == 20) {
    pa_config.max_power = 7;
    pa_config.pa_select = 1;
    pa_config.output_power = 15;
    pa_dac_config = RFM95_REGISTER_PA_DAC_HIGH_POWER;
  }

  if (!write_register(handle, RFM95_REGISTER_PA_CONFIG, pa_config.buffer)) return false;
  if (!write_register(handle, RFM95_REGISTER_PA_DAC, pa_dac_config)) return false;

  return true;
}


/* This function defines the maximum MAC PAYLOAD size for a given SF as described in the LoRaWAN
 * Link Layer specifications:
 *
 * DataRate    M      N
 *    0        59     51
 *    1        59     51
 *    2        59     51
 *    3       123    115
 *    4       230    222
 *    5       230    222
 *
 * - For uplink: FOpts field is never used in this library therefore M is always equals to M = N + 8
 *   where N is the actual number of bytes for frame payload (user data).
 *
 * - For downlink: TODO since FOpts may be present... and SF may be different depending on the target receive window
 *
 *
 *
 * @param *handle   rfm95_handle_t structure containing RFM95 configurations and function pointers.
 *
 *
 * @return uint8_t  defining the maximum MAC payload size for the SF currently configured in handle->config.tx_sf
 */
uint8_t max_mac_payload_length(rfm95_handle_t *handle){
  uint8_t sf = handle->config.tx_sf; // get current SF in use
  uint8_t max_frm_bytes;

  switch (sf) { //define max payload size per SF
    case 7: //SF7
      max_frm_bytes = 230;
      break;

    case 8: //SF8
      max_frm_bytes = 230;
      break;

    case 9: //SF9
      max_frm_bytes = 123;
      break;

    case 10: //SF10
      max_frm_bytes = 59;
      break;

    case 11: //SF11
      max_frm_bytes = 59;
      break;

    case 12: //SF12
      max_frm_bytes = 59;
      break;

    default: //Default case: SF7
      max_frm_bytes = 230;
      break;
  }

  return max_frm_bytes;
}


/* This function initializes RFM95 module:
 * - sets SPI pins and configurations;
 * - reload parameters stored into external memory, if not present loads default channel configurations;
 * - sets other configurations according to the Semtech Application note AN1200.24
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers.
 *
 * @return: - RFM95_STATUS_SPI_ERROR if any SPI communication error while writing configurations to RFM95
 *          - RFM95_STATUS_ERROR if RFM95 version doesn't match the standard value in the datasheet (0x12)
 *          - RFM95_STATUS_OK if the module was correctly initialized
 */
uint16_t rfm95_init(rfm95_handle_t *handle) {
  assert(handle->spi_handle->Init.Mode == SPI_MODE_MASTER);
  assert(handle->spi_handle->Init.Direction == SPI_DIRECTION_2LINES);
  assert(handle->spi_handle->Init.DataSize == SPI_DATASIZE_8BIT);
  assert(handle->spi_handle->Init.CLKPolarity == SPI_POLARITY_LOW);
  assert(handle->spi_handle->Init.CLKPhase == SPI_PHASE_1EDGE);
  assert(handle->get_precision_tick != NULL);
  assert(handle->random_int != NULL);
  assert(handle->precision_sleep_until != NULL);
  assert(handle->precision_tick_frequency > 10000);

  reset(handle);

  // If there is reload function or the reload was unsuccessful or the magic does not match restore default.
  if (handle->reload_config == NULL || !handle->reload_config(&handle->config) ||
      handle->config.magic != RFM95_EEPROM_CONFIG_MAGIC) {
    config_load_default(handle);
  }

  // Check for correct version.
  uint8_t version;
  if (!read_register(handle, RFM95_REGISTER_VERSION, &version, 1)) return RFM95_STATUS_SPI_ERROR;
  if (version != RFM9x_VER) return RFM95_STATUS_ERROR;

  // Module must be placed in sleep mode before switching to lora.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_SLEEP)) return RFM95_STATUS_SPI_ERROR;
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return RFM95_STATUS_SPI_ERROR;

  // Default interrupt configuration, must be done to prevent DIO5 clock interrupts at 1Mhz
  if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return RFM95_STATUS_SPI_ERROR;

  if (handle->on_after_interrupts_configured != NULL) {
    handle->on_after_interrupts_configured();
  }

  // Set module power: (default 17 dBm if not specified by user)
  if (!rfm95_set_power(handle, handle->config.tx_power)) return RFM95_STATUS_SPI_ERROR;

  // Set LNA to the highest gain with 150% boost.
  if (!write_register(handle, RFM95_REGISTER_LNA, 0x23)) return RFM95_STATUS_SPI_ERROR;

  // Preamble set to 8 + 4.25 = 12.25 symbols. --> Only used in continuous RX mode
  if (!write_register(handle, RFM95_REGISTER_PREAMBLE_MSB, 0x00)) return RFM95_STATUS_SPI_ERROR;
  if (!write_register(handle, RFM95_REGISTER_PREAMBLE_LSB, 0x08)) return RFM95_STATUS_SPI_ERROR;

  // Set TTN sync word 0x34.
  if (!write_register(handle, RFM95_REGISTER_SYNC_WORD, 0x34)) return RFM95_STATUS_SPI_ERROR;

  // Set up TX and RX FIFO base addresses.
  if (!write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, RFM95_FIFO_TX_BASE_ADDRESS)) return RFM95_STATUS_SPI_ERROR;
  if (!write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, RFM95_FIFO_RX_BASE_ADDRESS)) return RFM95_STATUS_SPI_ERROR;

  // Maximum payload length of the RFM95 is 0xFF.
  if (!write_register(handle, RFM95_REGISTER_MAX_PAYLOAD_LENGTH, 0xFF)) return RFM95_STATUS_SPI_ERROR;

  // Let module sleep after initialization.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return RFM95_STATUS_SPI_ERROR;

  return RFM95_STATUS_OK;
}


/* This function is used to process the MAC commands when frame payload is received at Port 0.
 * This version supports the following command list:
 * - LinkADRReq
 * - DutyCycleReq
 * - RXParamSetupReq
 * - DevStatusReq
 * - NewChannelReq
 * - RXTimingSetupReq
 * - TxParamSetupReq
 * - DlChannelReq
 *
 * Please, refer to the LoRa Alliance Link_Layer specifications for additional details about these commands.
 *
 * Keep in mind that the answer buffer (containing MAC command answers) produced by this function, is limited up to the maximum
 * number of bytes that can be transmitted with the current SF (see max_mac_payload_length function for additional details). Therefore,
 * the scan of the received payload is stopped when the maximum answer length is reached.
 * This allows to respond to the MAC commands sent by the network server in chunks that can be transmitted in a single operation. Indeed,
 * all the commands that have not been answered yet, will be repeated in subsequent downlinks until all commands have been correctly processed
 * and answered.
 *
 * @param *handle              rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param *frame_payload       uint8_t pointer to frame payload to be processed.
 * @param frame_payload_length size_t defining the actual frame payload size.
 * @param answer_buff          uint8_t pointer to buffer used for MAC response data.
 * @param answer_buffer_length uint8_t pointer to the final length of the answer buffer containing MAC command responses.
 * @param snr                  uint8_t defining the SNR of the last uplink message.
 *
 * @return: true if the entire payload has been processed, false if any MAC command size doesn't correspond
 *          to the expected bytes reported in the LoRa Alliance specifications.
 *
 */
static bool process_mac_commands(rfm95_handle_t *handle, const uint8_t *frame_payload,
                                 size_t frame_payload_length, uint8_t answer_buffer[51], uint8_t *answer_buffer_length,
                                 int8_t snr)
{
  uint8_t index = 0;
  uint8_t answer_index = 0;

  // In the absence of the FOpts field N = "frame payload length" = M - 8, where M = "MAC payload length"
  uint8_t max_answer_len = max_mac_payload_length(handle) - 8;

  bool answer_overflow = false;

  while ((index < frame_payload_length) && (!answer_overflow)) {  // continue until entire MAC commands have been processed or the
                                                                  // available space for answering is exceeded
    switch (frame_payload[index++])
    {
      case 0x01: // ResetConf
      {
        if (index >= frame_payload_length) return false;

        index += 1;
        break;
      }
      case 0x02: // LinkCheckReq
      {
        /**
         *
         * An end-device MAY use the LinkCheckReq command to validate its connectivity with the
         * Network. The command has no payload
         *
         */

        if ((index + 1) >= frame_payload_length) return false;

        index += 2;
        break;
      }
      case 0x03: // LinkADRReq
      {

        /**
         *
         * A Network Server MAY use the LinkADRReq command to request that an end-device
         * performs a rate adaptation. A Network Server MAY include multiple LinkADRReq commands within a single downlink
         * frame. For the purpose of configuring the end-device channel mask, the end-device SHALL
         * process all contiguous LinkADRReq commands in the order present in the downlink frame as
         * a single atomic block command
         *
         * TODO: ADR not implemented here --> Only channel mask is being processed: PowerACK and DR_ACK always set to zero
         *
         */

        uint8_t LinkADRReq_cnt = 1;   // account for current request
        uint8_t i = index+4;          // point to next MAC-ID
        while ((i + 5 < frame_payload_length) && frame_payload[i] == 0x03) {
          LinkADRReq_cnt++;
          i += 5; // Move to the next command (each LinkADRReq is 5 bytes long (1 MAC id + 4 payload bytes))
        }

        // check if enough space is available to respond in a single atomic block:
        // answer = 0x03 + 1byte ack
        if((answer_index + LinkADRReq_cnt*2 - 1) >= max_answer_len) {
          answer_overflow = true; // No enough space remained to answer in a single operation --> wait next cycle
          break;
        }

#ifdef MY_DEBUG
        printf("LinkADRReq cnt: %d\r\n", LinkADRReq_cnt);
#endif

        bool dr_ack      = 0;
        bool tx_pow_ack  = 0;
        bool ch_mask_ack = 1;

        uint16_t ch_masks[4] = {0};
        i = 0;

        // process atomic block and elaborate cumulative answer:
        while(LinkADRReq_cnt > 0) {
          // check if all payload bytes are present:
          if ((index + 3) >= frame_payload_length) return false;

          uint8_t dr_txPow    = frame_payload[index++];
          uint8_t ch_mask_lsb = frame_payload[index++];
          uint8_t ch_mask_msb = frame_payload[index++];
          uint8_t redundancy  = frame_payload[index++];

          // elaborate channel mask:
          uint8_t ChMaskCntl = (redundancy >> 4) & 0x07;

          if (ChMaskCntl == 6){
            // ChMaskCntl = 6 (all defined channels enabled)
            handle->config.channel_mask = handle->config.defined_channels;
          } else if(ChMaskCntl == 0){
            // channels enabled individually:
            uint16_t ch_mask = ch_mask_lsb | (ch_mask_msb << 8);

            // if trying to enable channels not defined yet or if channel mask deactivates all channels
            if(((~(handle->config.defined_channels) & ch_mask) != 0) || (ch_mask == 0)) {
              ch_mask_ack = 0;
            } else {
              ch_masks[i] = ch_mask;
            }

          } else {
            // RFU bit
          }

          // Process Power, data rate and NbTrans only for last command:
          if(LinkADRReq_cnt == 1){

            // Power and DR skipped since ADR is not implemented here...

            uint8_t NbTrans = redundancy & 0x0F;

            // TODO: do something with this NbTrans... (retransmissions not implemented yet)
            NbTrans++; // prevent warning errors
            dr_txPow++;

          } else {
            index++; // skip MAC-ID = 0x03 and move to payload part of next LinkADRReq block
          }

          LinkADRReq_cnt--;
          i++;
        }

        // answer (replicated N-times)
        uint8_t answer = (tx_pow_ack << 2) | (dr_ack << 1) | (ch_mask_ack);

        // compact the atomic block and prepare answer buff:
        uint16_t overall_mask = 0;

        for(uint8_t j = 0; j < i; j++){
          overall_mask |= ch_masks[j];

          // At this point the answer size shouldn't exceed! (we checked array length before processing the atomic block)
          if(answer_index + 1 >= max_answer_len) return false;

          answer_buffer[answer_index++] = 0x03;
          answer_buffer[answer_index++] = answer;
        }

        // apply mask to current configuration:
        if(ch_mask_ack == 1) handle->config.channel_mask = overall_mask;

        break;
      }
      case 0x04: // DutyCycleReq
      {
        /**
         *
         * The DutyCycleReq command can be used by the Network to limit the maximum aggregated
         * transmit duty cycle of an end-device.
         *
         */

        if (index >= frame_payload_length) return false;
        if (answer_index >= max_answer_len) { // there's no space left for answering --> wait next tx window
          answer_overflow = true;
          break;
        }

        uint8_t maxDuty = (frame_payload[index++] & 0x0F);

        if (maxDuty != 0){ // duty limitations by network server:

          // TODO: implement code to extend sleep time!

#ifdef MY_DEBUG
          printf("duty limit: %d\r\n", maxDuty);
#endif
        }

        answer_buffer[answer_index++] = 0x04; // no payload needed in answer buff

        break;
      }
      case 0x05: // RXParamSetupReq
      {

        /**
         *
         * The RXParamSetupReq command allows a change to the frequency and the data rate set
         * for RX2 following each uplink. The command also allows an offset to be programmed between
         * the uplink and the RX1 slot downlink data rate
         *
         */

        if ((index + 3) >= frame_payload_length) return false;
        if ((answer_index + 1) >= max_answer_len)  { // there's no space left for answering --> wait next tx window
          answer_overflow = true;
          break;
        }


        // This command must be send until downlink is received
        handle->has_pending_answer = 1;

        uint8_t dl_settings = frame_payload[index++];
        uint8_t frequency_lsb = frame_payload[index++];
        uint8_t frequency_msb = frame_payload[index++];
        uint8_t frequency_hsb = frame_payload[index++];
        uint32_t frequency = (frequency_lsb | (frequency_msb << 8) | (frequency_hsb << 16)) * 100;

        uint8_t rx2_sf = dl_settings & 0x0F;
        uint8_t rx1_dr_offset = (dl_settings >> 4) & 0x07;


        answer_buffer[answer_index++] = 0x05;

        if(!(rx1_dr_offset <= 5 && ((rx2_sf == 0) || (rx2_sf == 3)) && (frequency == 869525000))) {
#ifdef MY_DEBUG
          printf("Err RXParamSetupReq\r\n");
#endif
          // NACK setup request
          answer_buffer[answer_index++] = 0x00;
          break;
        }

#ifdef MY_DEBUG
          printf("Update RXParamSetupReq\r\n");
          printf("DR1 offset: %d\r\n", rx1_dr_offset);
          printf("RX2 SF: %d\r\n", rx2_sf);
#endif

        // else accept command and change local settings:
        handle->config.rx1_dr_offset = rx1_dr_offset;
        handle->config.rx2_sf = rx2_sf;

        answer_buffer[answer_index++] = 0b0000111;
        break;
      }
      case 0x06: // DevStatusReq
      {

        /**
         *
         * A Network Server can use the DevStatusReq command to request status information from
         * an end-device. The command has no payload.
         *
         */

#ifdef MY_DEBUG
        printf("Device status request\r\n");
#endif

        if ((answer_index + 2) >= max_answer_len)  { // there's no space left for answering --> wait next tx window
          answer_overflow = true;
          break;
        }

        uint8_t margin = (uint8_t)(snr & 0x1f);
        uint8_t battery_level = handle->get_battery_level == NULL ? 0xff : handle->get_battery_level();


        answer_buffer[answer_index++] = 0x06;
        answer_buffer[answer_index++] = battery_level;
        answer_buffer[answer_index++] = margin;
        break;
      }
      case 0x07: // NewChannelReq
      {

        /**
         *
         * A Network Server can use the NewChannelReq command either to create a new bidirectional
         * channel or to modify the parameters of an existing one. The command sets the center
         * frequency of the new channel and the range of uplink data rates that are usable on this
         * channel:
         *
         */

        if ((index + 4) >= frame_payload_length) return false;
        if ((answer_index + 1) >= max_answer_len)  { // there's no space left for answering --> wait next tx window
          answer_overflow = true;
          break;
        }

        uint8_t channel_index = frame_payload[index++];
        uint8_t frequency_lsb = frame_payload[index++];
        uint8_t frequency_msb = frame_payload[index++];
        uint8_t frequency_hsb = frame_payload[index++];
        uint8_t min_max_dr = frame_payload[index++];

        uint32_t frequency = (frequency_lsb | (frequency_msb << 8) | (frequency_hsb << 16)) * 100;
        uint8_t min_dr = min_max_dr & 0x0f;
        uint8_t max_dr = (min_max_dr >> 4) & 0x0f;

        // check if minimum DR0 (SF12) and maximum DR5 (SF7) are supported:
        bool dr_support = (min_dr == 0) && (max_dr >= 5);

#ifdef MY_DEBUG
        printf("F: %lu, %d <= DR <= %d\r\n", frequency, min_dr, max_dr);
#endif
        uint8_t ans_payload = 0x00;

        if (channel_index >= 3) { // channel index OK (not modifying default channels)

          if((frequency >= LOW_FREQ_BAND_EU868) && (frequency <= HIGH_FREQ_BAND_EU868) && dr_support){ // if frequency range and data rate are OK

            config_set_channel(handle, channel_index, frequency);
            ans_payload = 0x01 | (dr_support << 1);

          } else if (frequency == 0){ // frequency == 0 --> channel is being deactivated

            deactivate_channel(handle, channel_index);
            ans_payload = 0x01 | (dr_support << 1);

          } else { // either frequency or DR are not supported

            // send NACK back and do not modify current setup
            ans_payload = 0x00;

          }

        } else {
          // Default channels (index 0,1,2) cannot be modified!
#ifdef MY_DEBUG
          printf("Cannot modify default channels!\r\n");
#endif
          // neglect all changes!
          ans_payload = 0x00;

        }

        answer_buffer[answer_index++] = 0x07;
        answer_buffer[answer_index++] = ans_payload;
        break;
      }
      case 0x08: // RXTimingSetupReq
      {

        /**
         *
         * A Network Server can use the RXTimingSetupReq command to configure the delay between
         * the end of the TX uplink transmission and the opening of RX1. RX2 always opens 1s after
         * RX1
         *
         */

        // This command must be answered until downlink is received:
        handle->has_pending_answer = true;

        if (index >= frame_payload_length) return false;
        if (answer_index >= max_answer_len)  { // there's no space left for answering --> wait next tx window
          answer_overflow = true;
          break;
        }

        handle->config.rx1_delay = frame_payload[index++] & 0x0F;
        if (handle->config.rx1_delay == 0) {
          handle->config.rx1_delay = 1;
        }

#ifdef MY_DEBUG
        printf("RXTimingSetupReq: %d\r\n", handle->config.rx1_delay);
#endif

        answer_buffer[answer_index++] = 0x08;
        break;
      }
      case 0x09: // TxParamSetupReq
      {

        /**
         *
         * A Network Server MAY use the TXParamSetupReq command to notify the end-device of the
         * maximum allowed dwell time, i.e. the maximum continuous transmit time of a packet over the
         * air, as well as the maximum allowed end-device Effective Isotropic Radiated Power (EIRP).
         *
         */

        if (index >= frame_payload_length) return false;

        // skip this command since no dwell time limitations are present in band EU868MHz

        break;
      }
      case 0x0a: // DlChannelReq
      {

        /**
         *
         * The DlChannelReq command allows the Network to associate a different downlink frequency
         * with the RX1 slot. This command is applicable for all regions supporting the NewChannelReq
         * command (e.g., EU863-870 and CN779-787, but not US902-928 or AU915-928). In regions
         * where that command is not defined, the end-device SHALL silently drop it.
         *
         */

#ifdef MY_DEBUG
        printf("DlChannelReq\r\n");
#endif

        if ((index + 3) >= frame_payload_length) return false;
        if ((answer_index + 1) >= max_answer_len)  { // there's no space left for answering --> wait next tx window
          answer_overflow = true;
          break;
        }

        // This command must be answered until downlink is received:
        handle->has_pending_answer = true;


        uint8_t channel_index = frame_payload[index++];
        uint8_t frequency_lsb = frame_payload[index++];
        uint8_t frequency_msb = frame_payload[index++];
        uint8_t frequency_hsb = frame_payload[index++];

        uint32_t frequency = (frequency_lsb | (frequency_msb << 8) | (frequency_hsb << 16)) * 100;

        bool channel_frequency_ok = 0;
        bool uplink_frequency_exists = 0;

        // check if frequency is in working range
        if((frequency >= LOW_FREQ_BAND_EU868) && (frequency <= HIGH_FREQ_BAND_EU868)){ // if frequency range and data rate are OK

          channel_frequency_ok = 1;

          // check if channel is defined:
          if(((1 << channel_index) & handle->config.defined_channels) != 0) uplink_frequency_exists = 1;

        }

        if(channel_frequency_ok && uplink_frequency_exists) {

          // apply modified settings:
          handle->config.channels[channel_index].downlink_frequency = frequency;

        }

        answer_buffer[answer_index++] = 0x0A;
        answer_buffer[answer_index++] = (uplink_frequency_exists << 1) | channel_frequency_ok;

        break;
      }
      case 0x0b: // RekeyConf
      {

        /**
         *
         * 0x0B is RFU, should not be received
         *
         */

#ifdef MY_DEBUG
        printf("processed RekeyConf\r\n");
#endif

        if (index >= frame_payload_length) return false;

        break;
      }
      case 0x0c: // ADRParamSetupReq
      {

        /**
         *
         * 0x0C is RFU, should not be received
         *
         */

#ifdef MY_DEBUG
        printf("processed ADRParamSetupReq\r\n");
#endif

        if (index >= frame_payload_length) return false;

        break;
      }
      case 0x0d: // DeviceTimeReq
      {

        // TODO: implement if needed
        break;
      }
    }

  }

  *answer_buffer_length = answer_index;
  return true;
}


/* This function sets the RFM95 into receive-single mode.
 * DIO1 is configured to trigger symbol timeout: if a preamble is not detected within N-symbols wide window, DIO1 goes high.
 * DIO0 is configured to trigger an RX done interrupt.
 * The rfm95 module remains in sleep state and the MCU in stop2 mode until 1 ms before the RX1 opening time.
 *
 *
 * @param *handle        rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param scheduled_time uint32_t defining the predicted opening time (expressed as clock ticks) for RX window.
 *
 * @return: - RFM95_STATUS_SPI_ERROR if any SPI error is registered during communication
 *          - RFM95_STATUS_IRQ_TIMEOUT if a timeout occurs while setting module configurations (interrupts on DIO5 to flag module ready did not occur)
 *          - RFM95_STATUS_OK if the module is correctly moved into data reception mode
 */
static uint16_t receive_at_scheduled_time(rfm95_handle_t *handle, uint32_t scheduled_time) {

  // Sleep until 1ms before the scheduled time.
  handle->precision_sleep_until(scheduled_time - handle->precision_tick_frequency / 1000);

  // Clear flags and previous interrupt time, configure mapping for RX done.
  if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return RFM95_STATUS_SPI_ERROR;
  if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return RFM95_STATUS_SPI_ERROR;
  handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;
  handle->interrupt_times[RFM95_INTERRUPT_DIO1] = 0;
  handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;

  // Move modem to lora standby.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return RFM95_STATUS_SPI_ERROR;

  // Wait for the modem to be ready.
  if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT)) return RFM95_STATUS_IRQ_TIMEOUT;

  // Now sleep until the real scheduled time.
  handle->precision_sleep_until(scheduled_time);

  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE)) return RFM95_STATUS_SPI_ERROR;

  return RFM95_STATUS_OK;
}

/*
 * Do not remove this commented section for now... Used to calculate RX timing in the old version of this library
 */
//static void calculate_rx_timings(rfm95_handle_t *handle, uint32_t bw, uint8_t sf, uint32_t tx_ticks,
//                                 uint32_t *rx_target, uint32_t *rx_window_symbols)
//{
//  volatile int32_t symbol_rate_ns = (int32_t)(((2 << (sf - 1)) * 1000000) / bw);
//
//  volatile int32_t rx_timing_error_ns = (int32_t)(handle->precision_tick_drift_ns_per_s * handle->config.rx1_delay);
//  volatile int32_t rx_window_ns = 2 * symbol_rate_ns + 2 * rx_timing_error_ns;
//  volatile int32_t rx_offset_ns = 4 * symbol_rate_ns - (rx_timing_error_ns / 2);
//  volatile int32_t rx_offset_ticks = (int32_t)(((int64_t)rx_offset_ns * (int64_t)handle->precision_tick_frequency) / 1000000);
//  *rx_target = tx_ticks + handle->precision_tick_frequency * handle->config.rx1_delay + rx_offset_ticks;
//  *rx_window_symbols = rx_window_ns / symbol_rate_ns;
//}



/*
 * Calculate Receive timing as specified in the Semtech Application note (Please refer to the link:
 * https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/440000001NAw/7YN8ZamV70_xR.vPDAAAshm.0Wt4jmRX0nOKkOzQqiI )
 *
 * Notice also that application note assumes systems with poor clock accuracy, with a drift error at least >= 1.5 ms during the 1 second
 * receive window. Therefore, since STM LPTIM can provide very accurate timings, in the case in which rx_offset < 1.5 ms, we set
 * the default value to be exactly 1.5 ms.
 *
 *
 * @param *handle            rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param window             bool to define which RX window is used (0 for RX1, 1 for RX2)
 * @param bw                 uint32_t defining current BW (fixed to 125 kHz in this code...)
 * @param tx_ticks           uint32_t specifying the clock ticks corresponding to the last uplink. This allows to precisely calculate
 *                           the delay for opening RX windows.
 * @param *rx_target         uint32_t pointer used to return the target clock ticks for RX window opening.
 * @param *rx_window_symbols uint32_t pointer defining the duration of the symbol timeout for the preamble detection.
 *
 * @return: void
 */
static void calculate_rx_timings(rfm95_handle_t *handle, bool window, uint32_t bw, uint8_t sf, uint32_t tx_ticks,
                                 uint32_t *rx_target, uint32_t *rx_window_symbols)
{
  // RX window delay expressed in seconds:
  volatile uint8_t delay = (window == 0) ? handle->config.rx1_delay : handle->config.rx1_delay + 1;

  volatile int32_t symbol_rate_us = (int32_t)(((2 << (sf - 1)) * 1000000) / bw);
  volatile int32_t rx_timing_error_us = (int32_t)(handle->precision_tick_drift_ns_per_s * delay/1000);
  volatile int32_t rx_window_us = 2 * symbol_rate_us + 2 * rx_timing_error_us;
  volatile int32_t rx_offset_us = 4 * symbol_rate_us - (rx_timing_error_us / 2);

  // if window is too short, then use minimum 5 symbols length:
  if (rx_window_us <  5 * symbol_rate_us) {
    rx_window_us =    6 * symbol_rate_us;
    rx_offset_us = -0.5 * symbol_rate_us;
  }

  volatile int32_t rx_offset_ticks = (int32_t)(((int64_t)rx_offset_us * (int64_t)handle->precision_tick_frequency) / 1000000);

  *rx_target = tx_ticks + handle->precision_tick_frequency * delay + rx_offset_ticks;
  *rx_window_symbols = rx_window_us / symbol_rate_us;
}


/* This function performs a complete receive cycle: opens RX1, if nothing is received then it proceeds with RX2.
 * If something is received and no CRC error is generated, the code proceeds with payload
 * extraction by reading the RFM95 internal FIFO.
 *
 *
 * @param *handle      rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param tx_ticks     uint32_t used to calculate precise opening ticks for the two receive windows.
 * @param *payload_buf uint8_t pointer used to store received payload data.
 * @param *payload_len uint8_t pointer that stores actual payload length that will be processed.
 * @param *snr         uint8_t pointer to store SNR of the last received packet.
 * @param channel      uint8_t index defying which channel among the active ones will be used for RX1 window.
 *
 *
 * @return: - RFM95_STATUS_SPI_ERROR if any interface error occurs while setting up the module
 *          - RFM95_STATUS_IRQ_TIMEOUT if timeout occurred while waiting for any IRQ
 *          - RFM95_STATUS_RX_EMPTY if no data is received during RX1 and RX2
 *          - RFM95_STATUS_RX_CRC_ERR if CRC error is detected in receive mode
 *          - RFM95_STATUS_OK if data is correctly received with no errors
 */
static uint16_t receive_package(rfm95_handle_t *handle, uint32_t tx_ticks, uint8_t *payload_buf, size_t *payload_len,
                            int8_t *snr, uint8_t channel)
{
  *payload_len = 0;

  // calculate rx1 SF accounting for DR offset
  uint8_t rx1_sf = handle->config.tx_sf + handle->config.rx1_dr_offset;
  rx1_sf = (rx1_sf > 12 ? 12 : rx1_sf);

  // calculate new RX1 timings:
  uint32_t rx1_target, rx1_window_symbols;
  calculate_rx_timings(handle, 0, 125000, rx1_sf, tx_ticks, &rx1_target, &rx1_window_symbols);

  // calculate new RX2 timings:
  uint32_t rx2_target, rx2_window_symbols;
  calculate_rx_timings(handle, 1, 125000, handle->config.rx2_sf, tx_ticks, &rx2_target, &rx2_window_symbols);

  assert(rx1_window_symbols <= 0x3ff);

  // Configure downlink frequency for Rx1 (it should be equal to the uplink):
  if (!configure_frequency(handle, handle->config.channels[channel].downlink_frequency)) return RFM95_STATUS_SPI_ERROR;

  // Configure modem (125kHz, 4/5 error coding rate):
  if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return RFM95_STATUS_SPI_ERROR;

  // Configure modem SF (depends on user configuration + DR offset):
  uint8_t sf_bits = (rx1_sf << 4) | 0x04; // set SF + CRC enable
  if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, sf_bits | ((rx1_window_symbols >> 8) & 0x3))) return RFM95_STATUS_SPI_ERROR;

  // AGC on (suggested in application note), LDR optimization on only for SF >= 11
  uint8_t LDRoptimize = rx1_sf > 10 ? 0x0C : 0x04;
  if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, LDRoptimize)) return RFM95_STATUS_SPI_ERROR;

  // Set maximum symbol timeout.
  if (!write_register(handle, RFM95_REGISTER_SYMB_TIMEOUT_LSB, rx1_window_symbols)) return RFM95_STATUS_SPI_ERROR;

  // Set IQ registers according to AN1200.24.
  if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_RX)) return RFM95_STATUS_SPI_ERROR;
  if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_RX)) return RFM95_STATUS_SPI_ERROR;

  // wait for reception:
  uint16_t rfm95_status = receive_at_scheduled_time(handle, rx1_target);
  if (rfm95_status != RFM95_STATUS_OK) return rfm95_status;

  rfm95_status = wait_for_rx_irqs(handle, RFM95_RECEIVE_TIMEOUT);

  // If there was nothing received during RX1, try RX2.
  if (rfm95_status != RFM95_STATUS_OK) {

    // If RX1 fails return modem to sleep awaiting for RX2:
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return RFM95_STATUS_SPI_ERROR;

    // Open RX2 here:
    if (handle->receive_mode == RFM95_RECEIVE_MODE_RX12) {

#ifdef MY_DEBUG
    printf("Opening RX2\r\n");
#endif

      // Configure 869.525 MHz
      if (!configure_frequency(handle, 869525000)) return RFM95_STATUS_SPI_ERROR;

      // Configure modem (125kHz, 4/5 error coding rate):
      if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return RFM95_STATUS_SPI_ERROR;

      // Configure modem SF9 or 12 (depends on user configuration):
      sf_bits = (handle->config.rx2_sf << 4) | 0x04; // set SF + CRC enable
      if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, sf_bits | ((rx2_window_symbols >> 8) & 0x3))) return RFM95_STATUS_SPI_ERROR;

      // AGC on (suggested in application note), LDR optimization on only for SF >= 11
      LDRoptimize = handle->config.rx2_sf > 10? 0x0C : 0x04;
      if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, LDRoptimize)) return RFM95_STATUS_SPI_ERROR;

      // Set maximum symbol timeout
      if (!write_register(handle, RFM95_REGISTER_SYMB_TIMEOUT_LSB, rx2_window_symbols)) return RFM95_STATUS_SPI_ERROR;

      rfm95_status = receive_at_scheduled_time(handle, rx2_target);;
      if (rfm95_status != RFM95_STATUS_OK) return rfm95_status;

      rfm95_status = wait_for_rx_irqs(handle, RFM95_RECEIVE_TIMEOUT);
      if (rfm95_status != RFM95_STATUS_OK) {
        // No payload during in RX1 and RX2

        // Return modem to sleep.
        if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return RFM95_STATUS_SPI_ERROR;
        return rfm95_status;
      }
    }
  }

  // proceed with payload extraction:
  uint8_t irq_flags;
  read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irq_flags, 1);

  // Check if there was a CRC error.
  if (irq_flags & 0x20) {
    // Return modem to sleep.
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return RFM95_STATUS_SPI_ERROR;
    return RFM95_STATUS_RX_CRC_ERR;
  }

  int8_t packet_snr;
  if (!read_register(handle, RFM95_REGISTER_PACKET_SNR, (uint8_t *)&packet_snr, 1)) return RFM95_STATUS_SPI_ERROR;
  *snr = (int8_t)(packet_snr / 4);

  // Read received payload length.
  uint8_t payload_len_internal;
  if (!read_register(handle, RFM95_REGISTER_FIFO_RX_BYTES_NB, &payload_len_internal, 1)) return RFM95_STATUS_SPI_ERROR;

  // Read received payload itself.
  if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, RFM95_FIFO_RX_BASE_ADDRESS)) return RFM95_STATUS_SPI_ERROR;
  if (!read_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf, payload_len_internal)) return RFM95_STATUS_SPI_ERROR;

  // Return modem to sleep.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return RFM95_STATUS_SPI_ERROR;

  // Successful payload receive, set payload length to tell caller.
  *payload_len = payload_len_internal;
  return RFM95_STATUS_OK;
}


/* This function is used to send an uplink message containing user data.
 * All the configurations (SF, Tx POW, ...) must be set using the handle structure, in particular by
 * properly configuring the handle->config field.
 *
 * DIO5 interrupt is used to wait for the module to be ready upon switching to TX mode, while DIO0 is used
 * to trigger the TX completed event.
 *
 * @param *handle      rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param *payload_buf uint8_t pointer containing user data.
 * @param payload_len  uint8_t that stores actual payload length.
 * @param channel      uint8_t index defying which channel among the active ones will be used for RX1 window.
 * @param *tx_ticks    uint32_t pointer to store clock tick corresponding to the end of the transmission. This value will
 *                              be used to calculate precise timings for opening receive windows.
 *
 *
 * @return: - RFM95_STATUS_SPI_ERROR if any SPI error is observed while configuring RFM95 module
 *          - RFM95_STATUS_IRQ_TIMEOUT if RFM busy (interrupt on DIO5 not received) or TX fails (interrupt on DIO0 not received)
 *          - RFM95_STATUS_OK if transmission is correctly completed
 */
static uint16_t send_package(rfm95_handle_t *handle, uint8_t *payload_buf, size_t payload_len, uint8_t channel,
                         uint32_t *tx_ticks)
{
  // Configure channel for transmission.
  if (!configure_channel(handle, channel)) return RFM95_STATUS_SPI_ERROR;

  // Configure modem (125kHz, 4/5 error coding rate):
  if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return RFM95_STATUS_SPI_ERROR;

  // Configure modem SF7, ..., SF12 (depends on user configuration):
  uint8_t sf_bits = (handle->config.tx_sf << 4) | 0x04; // set SF + CRC enable
  if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, sf_bits)) return RFM95_STATUS_SPI_ERROR;

  // AGC on (suggested in application note), LDR optimization on only for SF >= 11
  uint8_t LDRoptimize = handle->config.tx_sf > 10? 0x0C : 0x04;
  if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, LDRoptimize)) return RFM95_STATUS_SPI_ERROR;

  // set TX power according to user configuration:
  if (!rfm95_set_power(handle, handle->config.tx_power)) return RFM95_STATUS_SPI_ERROR;

  // Set IQ registers according to AN1200.24.
  if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_TX)) return RFM95_STATUS_SPI_ERROR;
  if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_TX)) return RFM95_STATUS_SPI_ERROR;

  // Set the payload length.
  if (!write_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, payload_len)) return RFM95_STATUS_SPI_ERROR;

  // Enable tx-done interrupt, clear flags and previous interrupt time.
  if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE)) return RFM95_STATUS_SPI_ERROR;
  if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return RFM95_STATUS_SPI_ERROR;
  handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;
  handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;

  // Move modem to lora standby.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return RFM95_STATUS_SPI_ERROR;

  // Wait for the modem to be ready.
  if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT)) return RFM95_STATUS_IRQ_TIMEOUT;

  // Set pointer to start of TX section in FIFO.
  if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, RFM95_FIFO_TX_BASE_ADDRESS)) return RFM95_STATUS_SPI_ERROR;

  // Write payload to FIFO.
  for (size_t i = 0; i < payload_len; i++) {
    write_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf[i]);
  }

  // Set modem to tx mode.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_TX)) return RFM95_STATUS_SPI_ERROR;

  // Wait for the transfer complete interrupt.
  if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO0, RFM95_SEND_TIMEOUT)) return RFM95_STATUS_SPI_ERROR;

  // Set real tx time in ticks.
  *tx_ticks = handle->interrupt_times[RFM95_INTERRUPT_DIO0];

  // Return modem to sleep.
  if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return RFM95_STATUS_SPI_ERROR;

  // Increment tx frame counter.
  handle->config.tx_frame_count++;

  return RFM95_STATUS_OK;
}



/* This function is used for encoding the payload according to LoRaWAN specifications.
 * Starting from the "user payload" MHDR, FHDR, FPORT and MIC fields are added.
 *
 * If requested by the user the MAC header is configured for confirmed uplink messages, otherwise as default value
 * unconfirmed ones are used. Frame control byte is set to 0x00 by default as ADR is not implemented here and all
 * the MAC commands are sent in the payload with FPort 0 rather than using FOpts field. Support for ACK bit in FCtrl is used
 * to confirm confirmed downlink requests.
 *
 * @param *handle              rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param payload_buf          uint8_t pointer to buffer that will contain the encoded data.
 * @param *frame_payload       uint8_t pointer to buffer containing user data.
 * @param frame_payload_length size_t defining the user buffer length.
 * @param port                 uint8_t specifying Frame port field: 0 for MAC commands reserved for Network server.
 *                             1 for application data for Application server.
 * @param up_ack               boolean used to request confirmed uplink (0 = unconfirmed, 1 = confirmed).
 * @param confirmed_down       boolean used to set ACK bit in FCtrl byte to confirm a confirmed downlink request.
 *
 * @return size_t              - actual length of the complete frame (MHDR+MAC-PAYLOAD+MIC)
 *                             - 0 is returned if encoding error occurred (frame payoad too long for the selected SF).
 */
static size_t encode_phy_payload(rfm95_handle_t *handle, uint8_t payload_buf[64], const uint8_t *frame_payload,
                                 size_t frame_payload_length, uint8_t port, bool up_ack, bool confirmed_down)
{
  size_t payload_len = 0;

  //check that frame_payload_length is valid for the selected SF:
  uint8_t max_mac_length = max_mac_payload_length(handle);

  // +7 +1 includes FHDR (without FOpts field never used in uplink) and FPort (always present in uplink)
  if (frame_payload_length + 7 + 1 > max_mac_length) return 0;


  payload_buf[0] = (up_ack ? 0x80 : 0x40);    // MAC Header (confirmation required if up_ack set)
  payload_buf[1] = handle->device_address[3];
  payload_buf[2] = handle->device_address[2];
  payload_buf[3] = handle->device_address[1];
  payload_buf[4] = handle->device_address[0];
  payload_buf[5] = 0x00; // Initialize Fcntrl to 0
  payload_buf[5]|= (confirmed_down << 5); // Frame Control: set ACK uplink if requested
  payload_buf[6] = (handle->config.tx_frame_count & 0x00ffu);
  payload_buf[7] = ((uint16_t)(handle->config.tx_frame_count >> 8u) & 0x00ffu);
  payload_buf[8] = port; // Frame Port
  payload_len += 9;

  // Encrypt payload in place in payload_buf.
  memcpy(payload_buf + payload_len, frame_payload, frame_payload_length);
  if (port == 0) {
    Encrypt_Payload(payload_buf + payload_len, frame_payload_length, handle->config.tx_frame_count,
                    0, handle->network_session_key, handle->device_address);
  } else {
    Encrypt_Payload(payload_buf + payload_len, frame_payload_length, handle->config.tx_frame_count,
                    0, handle->application_session_key, handle->device_address);
  }
  payload_len += frame_payload_length;

  // Calculate MIC and copy to last 4 bytes of the payload_buf.
  uint8_t mic[4];
  Calculate_MIC(payload_buf, mic, payload_len, handle->config.tx_frame_count, 0,
                handle->network_session_key, handle->device_address);
  for (uint8_t i = 0; i < 4; i++) {
    payload_buf[payload_len + i] = mic[i];
  }
  payload_len += 4;

  return payload_len;
}


/* This function is used for decoding the payload according to LoRaWAN specifications.
 * Starting from the received downlink it returns to the user the frame payload containing actual data.
 *
 *
 * @param *handle                       rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param payload_buf                   uint8_t pointer to buffer containing received data (to be decoded).
 * @param payload_length                uint8_t defining the actual payload size.
 * @param **decoded_frame_payload_ptr   double pointer to set the correct starting point of the decoded user data.
 * @param *decoded_frame_payload_length uint8_t pointer that will contain the actual length of the decoded payload.
 * @param *frame_  port                 uint8_t pointer that will specify Frame port field: 0 for MAC commands that need to be processed
 *                                      by using process_mac_commands function, 1 for application data for Application server.
 *
 * @return void
 */
static bool decode_phy_payload(rfm95_handle_t *handle, uint8_t payload_buf[64], uint8_t payload_length,
                               uint8_t **decoded_frame_payload_ptr, uint8_t *decoded_frame_payload_length, uint8_t *frame_port)
{
  // confirmed downlink received --> indicates that ACK in next uplink is needed
  if (payload_buf[0] == 0xA0) {
    handle->confirmed_downlink = 1;
#ifdef MY_DEBUG
    printf("Confirmed downlink received\r\n");
#endif
  }

  // Does the device address match?
  if (payload_buf[1] != handle->device_address[3] || payload_buf[2] != handle->device_address[2] ||
      payload_buf[3] != handle->device_address[1] || payload_buf[4] != handle->device_address[0]) {
    return false;
  }

  uint8_t frame_control = payload_buf[5];
  uint8_t frame_opts_length = frame_control & 0x0f;
  uint16_t rx_frame_count = (payload_buf[7] << 8) | payload_buf[6];

  // check if downlink set ACK bit to confirm a previously sent confirmed uplink request:
  if ((frame_control & 0x20) != 0) handle->uplink_acknowledged = true;

  // Check if rx frame count is valid and if so, update accordingly.
  if (rx_frame_count < handle->config.rx_frame_count) {
    return false;
  }
  handle->config.rx_frame_count = rx_frame_count;

  uint8_t check_mic[4];
  Calculate_MIC(payload_buf, check_mic, payload_length - 4, rx_frame_count, 1,
                handle->network_session_key, handle->device_address);
  if (memcmp(check_mic, &payload_buf[payload_length - 4], 4) != 0) {
    return false;
  }

  if (payload_length - 12 - frame_opts_length == 0) { // FPort is missing --> data only in the FOpts field
    *frame_port = 0;
    *decoded_frame_payload_ptr = &payload_buf[8];
    *decoded_frame_payload_length = frame_opts_length;

  } else { // All bytes are sent in the frame payload field, FOopts is empty --> Need to decode data accordingly
    *frame_port = payload_buf[8];

    uint8_t frame_payload_start = 9 + frame_opts_length;
    uint8_t frame_payload_end = payload_length - 4;
    uint8_t frame_payload_length = frame_payload_end - frame_payload_start;

    if (*frame_port == 0) {
      Encrypt_Payload(&payload_buf[frame_payload_start], frame_payload_length, rx_frame_count,
                      1, handle->network_session_key, handle->device_address);
    } else {
      Encrypt_Payload(&payload_buf[frame_payload_start], frame_payload_length, rx_frame_count,
                      1, handle->application_session_key, handle->device_address);
    }

    *decoded_frame_payload_ptr = &payload_buf[frame_payload_start];
    *decoded_frame_payload_length = frame_payload_length;
  }

  return true;
}


/* This function is used to randomly select a channel for uplink
 * among the active ones.
 *
 *
 * @param *handle rfm95_handle_t structure containing RFM95 configurations and function pointers.
 *
 * @return uint8_t corresponding to the channel index randomly selected
 */
static uint8_t select_random_channel(rfm95_handle_t *handle)
{
  uint8_t channel_count = 0;

  for (uint8_t i = 0; i < 16; i++) {
    if (handle->config.channel_mask & (1 << i)) {
      channel_count++;
    }
  }

  uint8_t random_channel = handle->random_int(channel_count);

  for (uint8_t i = 0; i < 16; i++) {
    if (handle->config.channel_mask & (1 << i)) {
      if (random_channel == 0) {
        return i;
      } else {
        random_channel--;
      }
    }
  }

  return 0;
}


/* Send user data and open two receive windows for downlink reception.
 *
 * -> If MAC commands that require downlink confirmation are waiting, they will have the priority over user data.
 *    If handle->has_pending_answer is set, those commands will be repeated until the Network sends a confirmation downlink in accordance
 *    to what is specified in the LoRaWAN Link Layer official specifications. (see process_mac_commands function for more details about
 *    how the answer buffer is managed).
 *
 * -> If MAC command responses do not require confirmation, an uplink is sent immediately after the end of 2nd receive window; otherwise
 *    the buffer will be stored locally and postponed to next uplink cycle (see point above).
 *
 * -> If needed this function automatically manages downlink confirmation by sending uplink with ACK bit set.
 *
 * -> The function also detects downlink with ACK bit set to confirm previously required confirmed uplink messages.
 *
 *
 * @param *handle          rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param *send_data       uint8_t pointer to user data to be sent in the uplink window.
 * @param send_data_length size_t used to define the payload length
 * @param confirmed        boolean for specifying if confirmed uplink is needed.
 *
 * return:
 *         ERRORS: :(
 *         - RFM95_STATUS_SPI_ERROR    if any SPI error is observed while configuring RFM95 module
 *         - RFM95_STATUS_RX_CRC_ERR   if CRC error is detected in receive mode
 *         - RFM95_STATUS_IRQ_TIMEOUT  if any timeout is generated on IRQ
 *         - RFM95_STATUS_DECODE_ERR   if an error occurred while decoding the frame payload
 *         - RFM95_STATUS_ENCODE_ERR   if error occurred while encoding (likely due to too long frame for given SF)
 *
 *         GOOD STATUS: :)
 *         - RFM95_STATUS_MAC_COMMANDS_PENDING if user packet is temporary suspended since there were pending MAC command answers
 *         - RFM95_STATUS_MAC_CMD_ERR          if an error occurred while processing MAC commands (wrong processing code?)
 *         - RFM95_STATUS_RX_EMPTY             if no data is received during RX1 and RX2
 *         - RFM95_STATUS_RX_MAGIC_OK          if magic keyword received in application data matches user keyword
 *         - RFM95_STATUS_RX_MAGIC_ER          if magic keyword received in application doesn't match user keyword
 *         - RFM95_STATUS_OK                   if cycle is correctly completed
 */
uint16_t rfm95_send_receive_cycle(rfm95_handle_t *handle, const uint8_t *send_data, size_t send_data_length, bool confirmed) {
  uint8_t phy_payload_buf[RFM95_MAX_MAC_PAYLOAD_LEN] = { 0 }; // Init to the maximum possible length
  size_t  phy_payload_len; // Phy payload len will then vary depending on frame length and MAC Header length

  uint8_t  random_channel = select_random_channel(handle);

  uint16_t rfm_status = 0x0000;

  uint32_t tx_ticks;



  /****************
   *
   * FRAME UPLINK
   *
   ****************
   */

  /***** MAC COMMAND ANSWERS ARE PENDING *****/
  if (handle->has_pending_answer){

    // Build the up-link with mac command answers: --> Request for a confirmed uplink (force network to acknowledge this frame)
    phy_payload_len = encode_phy_payload(handle, phy_payload_buf, handle->answer_buff, handle->answer_buff_length, 0, 1, handle->confirmed_downlink);

    // if error occurs while encoding: (wrong frame size?)
    if (phy_payload_len == 0) return RFM95_STATUS_ENCODE_ERR;

    handle->confirmed_downlink = 0; // ACK was sent, now reset flag

    // discard user data for now: give priority to send back MAC command answers
    rfm_status = send_package(handle, phy_payload_buf, phy_payload_len, random_channel, &tx_ticks);

    rfm_status |= RFM95_STATUS_MAC_COMMANDS_PENDING;  // notify user that user_data were not transmitted


    if ((rfm_status & RFM95_STATUS_OK) == 0) {
      write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);
      return rfm_status;
    }

#ifdef MY_DEBUG
    printf("Sending confirmed uplink\r\n");
#endif

  } else { /***** UPLINK WITH USER DATA *****/

    // Build the up-link with user data:
    phy_payload_len = encode_phy_payload(handle, phy_payload_buf, send_data, send_data_length, 1, confirmed, handle->confirmed_downlink);

    // if error occurs while encoding: (wrong frame size?)
    if (phy_payload_len == 0) return RFM95_STATUS_ENCODE_ERR;

    handle->confirmed_downlink = 0; // ACK was sent, now reset flag

    // Send the requested up-link.
    rfm_status = send_package(handle, phy_payload_buf, phy_payload_len, random_channel, &tx_ticks);
    if (rfm_status != RFM95_STATUS_OK) {
      write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);
      return rfm_status;
    }
  }


#ifdef MY_DEBUG
    printf("RFM95 send success\r\n");
#endif

  // Clear payload buffer to reuse for the down-link message.
  memset(phy_payload_buf, 0x00, sizeof(phy_payload_buf));
  phy_payload_len = 0;


  /*******************
   *
   * OPEN RX WINDOWS
   *
   *******************
   */

  if (handle->receive_mode != RFM95_RECEIVE_MODE_NONE) {

    int8_t snr;

    // Receive data:
    rfm_status &= ~(RFM95_STATUS_OK); // Set to zero status OK (so check can be performed also on receive windows)
    rfm_status |= receive_package(handle, tx_ticks, phy_payload_buf, &phy_payload_len, &snr, random_channel);

    // If any error occurred (or empty windows):
    if ((rfm_status & RFM95_STATUS_OK) == 0) {
      // set RFM back to sleep:
      write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);

      // If allowed to save on EEPROM memory:
      if (handle->save_config) {
        handle->save_config(&(handle->config));
      }

      return rfm_status;
    }


    /***** RX WINDOWS NOT EMPTY AND NO ERRORS OCCURRED *****/

#ifdef MY_DEBUG
    printf("Received something\r\n");
#endif

    // Reset flags and status
    rfm_status &= ~(RFM95_STATUS_OK); // Set to zero status OK
    handle->uplink_acknowledged = false;

    // Downlink received --> answer buff and pending flags can be cleared
    handle->has_pending_answer = false;
    memset(handle->answer_buff, 0x00, sizeof(handle->answer_buff));

    // process the new received downlink
    uint8_t *frame_payload;
    uint8_t frame_payload_len = 0;
    uint8_t frame_port;


    /***** DECODE RECEIVED PAYLOAD *****/
    if (decode_phy_payload(handle, phy_payload_buf, phy_payload_len, &frame_payload, &frame_payload_len,
                           &frame_port)) {
#ifdef MY_DEBUG
      printf("Data decoded\r\n");
#endif

      // check if an acknowledgment was set in the downlink frame control:
      if (handle->uplink_acknowledged == true) rfm_status |= RFM95_STATUS_ACK_RECEIVED;

      /***** PROCESS MAC COMMANDS *****/
      if (frame_port == 0) {

        // Accommodate space for the maximum possible answer --> then the mac_response_len will be adjusted accordingly
        uint8_t mac_response_data[RFM95_MAX_MAC_PAYLOAD_LEN] = {0};
        uint8_t mac_response_len = 0;

        bool process_mac_status = process_mac_commands(handle, frame_payload, frame_payload_len, mac_response_data,
                                                       &mac_response_len, snr);

        if (process_mac_status && (mac_response_len > 0)) {


          /*
           *  if (pending_answer is set) --> MAC commands must be responded
           *  with confirmed uplink until downlink received:
           *   - save answer buff
           *   - set a flag so next uplink will send it with confirmation required
           *   - avoid sending uplink immediately
           *
           *
           *  else: (can answer immediately since repetition is not needed!)
           */
          if (handle->has_pending_answer) { // do not respond immediately since confirmation downlink is required (wait next cycle)

#ifdef MY_DEBUG
            printf("preparing confirmed uplink\r\n");
#endif

            handle->answer_buff_length = mac_response_len;
            memcpy(handle->answer_buff, mac_response_data, mac_response_len);

          } else { // confirmation not needed: answer immediately after downlink

#ifdef MY_DEBUG
            printf("Sending uplink back\r\n");
#endif
            // Build the up-link physical payload.
            phy_payload_len = encode_phy_payload(handle, phy_payload_buf, mac_response_data,
                                                mac_response_len, 0, 0, handle->confirmed_downlink);

            // if error occurs while encoding: (wrong frame size?)
            if (phy_payload_len == 0) {
#ifdef MY_DEBUG
              printf("Encode error! (too long buff?)\r\n");
#endif

              return rfm_status | RFM95_STATUS_ENCODE_ERR;
            }

            handle->confirmed_downlink = 0; // ACK was sent, now reset flag

            rfm_status |= send_package(handle, phy_payload_buf, phy_payload_len, random_channel, &tx_ticks);

            if ((rfm_status & RFM95_STATUS_OK) == 0) { // If any error occurred:

              write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);

              // If allowed to save on EEPROM memory:
              if (handle->save_config) {
                handle->save_config(&(handle->config));
              }

              return rfm_status;
            }

            // 2nd uplink succeeded --> MAC configurations correctly set

          }

        } else if (process_mac_status && (mac_response_len == 0)){ // If empty downlink was received (Only MHDR and FHDR without FOpts bytes)

          // this might me the case of network server sending ACK for a confirmed uplink request
#ifdef MY_DEBUG
          printf("Received empty frame.\r\n");
#endif
          // no additional code is needed...

        } else { // failed processing MAC response (wrong buffer management?)
          if (handle->save_config) {
            handle->save_config(&(handle->config));
          }
          return rfm_status | RFM95_STATUS_MAC_CMD_ERR;
        }

      } else { /***** RECEIVE APPLICATION DATA *****/

#ifdef MY_DEBUG
        printf("Received application data: %d bytes\r\n", frame_payload_len);
#endif

        for(uint8_t i = 0; i < frame_payload_len; i++){
          printf("0x%02X\r\n", frame_payload[i]);
        }

        // For now app data are used to send trigger signals to the end device...
        uint16_t sync_word = (frame_payload[0] << 8) | frame_payload[1];
        if(handle->config.lora_magic == sync_word) {
          rfm_status |= RFM95_STATUS_RX_MAGIC_OK;
        } else {
          rfm_status |= RFM95_STATUS_RX_MAGIC_ERR;
        }
      }

    } else {
      // failed to decode physical payload (Possible errors: Dev address or frame counter not matching, or MIC error)
      if (handle->save_config) {
        handle->save_config(&(handle->config));
      }

      return rfm_status | RFM95_STATUS_DECODE_ERR;
    }



  } // End of receive cycle

  if (handle->save_config) {
    handle->save_config(&(handle->config));
  }

  return rfm_status | RFM95_STATUS_OK;
}



/* This function is linked to the EXTI interrupt lines. Once a trigger event
 * has been received, it is registered into this dedicated buffer where they
 * can be seen by the RFM internal IRQs.
 *
 * @param *handle   rfm95_handle_t structure containing RFM95 configurations and function pointers.
 * @param interrupt rfm95_interrupt_t defining which interrupt triggered this IRQ (either DIO0, DIO1 or DIO5).
 *
 * @return void.
 */
void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt)
{
  handle->interrupt_times[interrupt] = handle->get_precision_tick();
}


// Function to redirect printf output to UART ---> Used only for debug purpose
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(RFM_DEBUG, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
