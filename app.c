/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"

#include "app_assert.h"

#include "em_rtcc.h"
#include "em_iadc.h"
#include "em_cmu.h"

#include "sl_bluetooth.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "sl_si70xx.h"
#include "sl_board_control.h"

#include "app.h"

#define TIMER_CLK_FREQ ((uint32_t)32768)

#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))

#define UINT32_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT32_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))
#define UINT32_TO_BYTE2(n)            ((uint8_t) ((n) >> 16))
#define UINT32_TO_BYTE3(n)            ((uint8_t) ((n) >> 24))

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

//static uint32_t timeBase;
//static uint32_t timeBaseCnt             = 0;
//static uint32_t lastPacketTimeBaseCnt   = 0;
static uint32_t packetCnt         = 0;

static uint32_t timRtc          = 0;
static uint64_t timNum64        = 0;

static uint32_t relativeHumidity = 0;
static int32_t  temperature = 0x8000;  // = -128.0
static uint16_t batteryVoltage = 0;
static uint8_t batteryLevel = 0; /* Battery Level (%) */
static uint32_t uptime = 0;

const uint16_t swVersion = 0x20;

static sl_i2cspm_t *rht_sensor;

static sl_sleeptimer_timer_handle_t measureTimer;
static sl_sleeptimer_timer_handle_t advertiseTimer;
static sl_sleeptimer_timer_handle_t delayTimer;


typedef struct __attribute__((__packed__))
{
    int16_t   temperature;
    int16_t   humidity;
    uint16_t  battery_voltage;
    uint8_t   battery_level;
    uint32_t  uptime;
    uint8_t   spare[5];
} ble_sens_payload_t;

/* Format:
 *  packetId = AA55
 *  payload =
 *        2-bytes temperature
 *        2-bytes relativeHumidity
 *        2-bytes voltage
 *        1-byte  battery level
 *        4-bytes uptime
 *        1-bytes spare
 *        4-bytes magic field AA55FF00 (must decrypt correctly = MAC)
 */
static struct {
    uint8_t flagsLen;     /* Length of the Flags field. */
    uint8_t flagsType;    /* Type of the Flags field. */
    uint8_t flags;        /* Flags field. */
    uint8_t mandataLen;   /* Length of the Manufacturer Data field. */
    uint8_t mandataType;  /* Type of the Manufacturer Data field. */

    uint8_t packetId[2];
//    uint8_t payload[16];  /* Format: */
    ble_sens_payload_t payload;
    uint8_t nonce[4];
  }
  sens_adv_data
    = {
    /* Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags. */
    2,  /* length  */
    0x01, /* type */
    0x04 | 0x02, /* Flags: LE General Discoverable Mode, BR/EDR is disabled. */

    /* Manufacturer specific data */
    23,  /* length of field*/
    0xFF, /* type of field */

  #if ENCRYPTION
    {0xAA,0x05},            /* PacketId AA05 for encrypted packet*/
  #else
  {0xAA,0x55},            /* PacketId AA55 for plaintext packet*/
  #endif
  {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00}    /* Nonce init */

  };


  typedef struct {
    uint16_t      voltage;
    uint8_t       capacity;
  } VoltageCapacityPair;

  static VoltageCapacityPair battCR2032Model[] =
  { { 300, 100 }, { 290, 80 }, { 280, 60 }, { 270, 40 }, { 260, 30 },
    { 250, 20 }, { 240, 10 }, { 200, 0 } };


  static void advertise(sl_sleeptimer_timer_handle_t *handle, void *data);
  static void measure(sl_sleeptimer_timer_handle_t *handle, void *data);
  static void enableSensor(sl_sleeptimer_timer_handle_t *handle, void *data);
  static void startMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data);

  void adcInit(void);
  uint16_t measureBatteryVoltage(void);
  static uint32_t measureOneAdcSample(void);
  static uint8_t calculateLevel(uint16_t voltage, VoltageCapacityPair *model, uint8_t modelEntryCount);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  sl_sleeptimer_init();
  sl_board_disable_sensor(SL_BOARD_SENSOR_IMU);
  sl_board_disable_sensor(SL_BOARD_SENSOR_MICROPHONE);
  rht_sensor = sl_sensor_select(SL_BOARD_SENSOR_RHT);
  sl_board_enable_sensor(SL_BOARD_SENSOR_RHT);

  sl_si70xx_init(rht_sensor, SI7021_ADDR);
  sl_board_disable_sensor(SL_BOARD_SENSOR_RHT);
//  initNonce();
  adcInit();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  int16_t ret_power_min, ret_power_max;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Set 0 dBm maximum Transmit Power.
      sc = sl_bt_system_set_tx_power(SL_BT_CONFIG_MIN_TX_POWER, 0,
                                     &ret_power_min, &ret_power_max);
      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      sc = sl_sleeptimer_start_periodic_timer_ms(&advertiseTimer, 5000, advertise, (void *)NULL, 0, 0);
      app_assert_status(sc);

      sc = sl_sleeptimer_start_periodic_timer_ms(&measureTimer, 90000, enableSensor, (void *)NULL, 0, 0);
      app_assert_status(sc);

      // Manual immediate measurement
      enableSensor(NULL, NULL);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

void advertise(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  sl_status_t sc;
  (void)handle;
  (void)data;

  sens_adv_data.payload.temperature = (int16_t)temperature;
  sens_adv_data.payload.humidity = (int16_t)relativeHumidity;
  sens_adv_data.payload.battery_voltage = batteryVoltage;
  sens_adv_data.payload.battery_level = batteryLevel;
  sens_adv_data.payload.uptime = uptime;
//  sens_adv_data.payload[0] = UINT16_TO_BYTE1(temperature);
//  sens_adv_data.payload[1] = UINT16_TO_BYTE0(temperature);
//  sens_adv_data.payload[2] = UINT16_TO_BYTE1(relativeHumidity);
//  sens_adv_data.payload[3] = UINT16_TO_BYTE0(relativeHumidity);
//  sens_adv_data.payload[4] = UINT16_TO_BYTE1(batteryVoltage);
//  sens_adv_data.payload[5] = UINT16_TO_BYTE0(batteryVoltage);
//  sens_adv_data.payload[6] = batteryLevel;
//  sens_adv_data.payload[7] = UINT32_TO_BYTE3(uptime);
//  sens_adv_data.payload[8] = UINT32_TO_BYTE2(uptime);
//  sens_adv_data.payload[9] = UINT32_TO_BYTE1(uptime);
//  sens_adv_data.payload[10] = UINT32_TO_BYTE0(uptime);
//  sens_adv_data.payload[11] = swVersion;
//  sens_adv_data.payload[12] = 0xAA;
//  sens_adv_data.payload[13] = 0x55;
//  sens_adv_data.payload[14] = 0x00;
//  sens_adv_data.payload[15] = 0xFF;

  // Set custom advertising data.
  sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle,
                                        0,
                                        sizeof(sens_adv_data),
                                        (uint8_t *)(&sens_adv_data));
  app_assert_status(sc);

  // Set advertising parameters. 100ms - 200ms -  advertisement interval.
  sc = sl_bt_advertiser_set_timing(
    advertising_set_handle,
    160,     // min. adv. interval (milliseconds * 1.6)
    320,     // max. adv. interval (milliseconds * 1.6)
    0,       // adv. duration
    2);      // max. num. adv. events
  app_assert_status(sc);

  // Start advertising in user mode and disable connections.
  sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_non_connectable);
  app_assert_status(sc);
}

/***********************************************************************************************//**
* Main program flow:
*  Sleep 30 s (measureTimer)
*  Power on sensor, sleep 80 ms (delayTimer)
*  Start measurement, sleep 50 ms (delayTimer)
*  Read measurement from sensor and send if required
**************************************************************************************************/
void enableSensor(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  sl_status_t sc;

  sl_board_enable_sensor(SL_BOARD_SENSOR_RHT);

  sc = sl_sleeptimer_start_timer_ms(&delayTimer, 80, startMeasurement, NULL, 0, 0);
  app_assert_status(sc);
}

void startMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  sl_status_t sc;

  sc = sl_si70xx_start_no_hold_measure_rh_and_temp(rht_sensor, SI7006_ADDR);
  app_assert_status(sc);

  sc = sl_sleeptimer_start_timer_ms(&delayTimer, 50, measure, NULL, 0, 0);
  app_assert_status(sc);
}

void measure(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  /* This function sets up a custom advertisement package if temperature changed, or if the ticker mandates full packet. */

  (void)handle;
  (void)data;
  sl_status_t sc;

  // Start with a temperature measurement
//  bool sending = false;
  sc = sl_si70xx_read_rh_and_temp(rht_sensor, SI7006_ADDR, &relativeHumidity, &temperature);
  sl_board_disable_sensor(SL_BOARD_SENSOR_RHT);
  app_assert_status(sc);

//  // Measure and send full packet regardless of differential state
//  if ( timeBaseCnt - lastPacketTimeBaseCnt > heartbeatTimebaseDelta ){
//    sending = true;
//  }
//
//  // If a value has changed significantly, send anyways
//  if ( abs(temperature - lastTemperature) > 100 * tempDiffThreshold || abs(relativeHumidity - lastRelativeHumidity) > 100 * humDiffThreshold){
//    sending = true;
//  }

//  if ( sending ){
//    lastTemperature = temperature;
//    lastRelativeHumidity = relativeHumidity;
//    lastPacketTimeBaseCnt = timeBaseCnt;

    temperature = (temperature + 5) / 10;
    relativeHumidity = (relativeHumidity + 5) / 10;
    if ( relativeHumidity > 10000 ){
      relativeHumidity = 10000;
    }

    batteryVoltage = (measureBatteryVoltage() + 5) / 10;
    batteryLevel = calculateLevel(batteryVoltage, battCR2032Model, sizeof(battCR2032Model) / sizeof(VoltageCapacityPair));
    uint32_t timRtcNow = RTCC_CounterGet();
    timNum64 += (timRtcNow - timRtc);
    timRtc = timRtcNow;
    uptime = (uint32_t)(timNum64 / (TIMER_CLK_FREQ ));

    packetCnt++;
}

void adcInit(void)
{
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  initSingleInput.posInput = iadcPosInputAvdd; // Actually means AVDD / 4

  IADC_init(IADC0, &init, &initAllConfigs);
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

}

uint16_t measureBatteryVoltage(void)
{
  uint32_t adcData;
  uint16_t voltage;

  adcData = measureOneAdcSample();
  // Reference 1.21 V, 12 bits, measuring AVDD/4
  voltage = ((adcData * 1210 + 1) / 4096 * 4);

  return voltage;
}


static uint32_t measureOneAdcSample(void)
{
  IADC_Result_t result;

  // Start single
  IADC_command(IADC0, iadcCmdStartSingle);

  while((IADC_getInt(IADC0) & IADC_IF_SINGLEDONE) != IADC_IF_SINGLEDONE)
  {
  }
  result = IADC_readSingleResult(IADC0);

  return result.data;
}

static uint8_t calculateLevel(uint16_t voltage, VoltageCapacityPair *model, uint8_t modelEntryCount)
{
  uint8_t res = 0;
  int i;
  int32_t tmp;

  for (i = 0; i < modelEntryCount; i++) {
    if (i == modelEntryCount) {
      return model[i].capacity;
    } else if (voltage >= model[i].voltage) {
      res = model[i].capacity;
      break;
    } else if ((voltage < model[i].voltage) && (voltage >= model[i + 1].voltage)) {
      //
      tmp = (voltage - model[i].voltage)
            * (model[i + 1].capacity - model[i].capacity)
            / (model[i + 1].voltage - model[i].voltage);
      tmp += model[i].capacity;
      res = tmp;
      break;
    }
  }

  return res;
}
