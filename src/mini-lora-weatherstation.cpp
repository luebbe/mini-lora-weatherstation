#include <LoRaWan_APP.h>
#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "ttnparams.h"

bool ENABLE_SERIAL = false;       // Enable serial debug output here if required
uint8_t i2cAddress = 0x76;        // Check whether your BME280 is on 0x76 or 0x77
uint32_t appTxDutyCycle = 300000; // The frequency of readings, in milliseconds (set 300s)

uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = LORAWAN_CLASS;
bool overTheAirActivation = LORAWAN_NETMODE;
bool loraWanAdr = LORAWAN_ADR;
bool keepNet = LORAWAN_NET_RESERVE;
bool isTxConfirmed = LORAWAN_UPLINKMODE;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

int temperature, humidity, batteryVoltage, batteryLevel;
long pressure; // Pressure needs three bytes unless you give up one or two decimal places

Adafruit_BME280 bme280;

static void readData()
{
  // This enables the output to power the sensor
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(500);

  if (!bme280.begin(i2cAddress))
  {
    if (ENABLE_SERIAL)
    {
      Serial.println("BME280 sensor not found!");
    }
    return;
  }

  // This delay is required to allow the sensor time to init
  delay(500);

  // Temperature offset to compensate for wrong measurements. The Adafruit BME280 library 
  // takes this offset into consideration when it calculates the other values.
  // bme280.setTemperatureCompensation(1.23f);

  temperature = bme280.readTemperature() * 100; // In order to have two decimal places stored in the integer
  humidity = bme280.readHumidity();
  pressure = bme280.readPressure();

  Wire.end();

  // Turn the power to the sensor off again
  digitalWrite(Vext, HIGH);

  batteryVoltage = getBatteryVoltage();
  batteryLevel = BoardGetBatteryLevel() * 100 / 254;

  if (ENABLE_SERIAL)
  {
    Serial.print("Temperature: ");
    Serial.print(temperature / 100);
    Serial.print("C, Humidity: ");
    Serial.print(humidity);
    Serial.print("%, Pressure: ");
    Serial.print(pressure / 100);
    Serial.print(" mbar, Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.print(" mV, Battery Level: ");
    Serial.print(batteryLevel);
    Serial.println(" %");
  }
}

static void prepareTxFrame(uint8_t port)
{
  readData();

  appDataSize = 12;
  appData[0] = highByte(temperature);
  appData[1] = lowByte(temperature);

  appData[2] = highByte(humidity);
  appData[3] = lowByte(humidity);

  appData[4] = (byte)((pressure & 0xFF000000) >> 24);
  appData[5] = (byte)((pressure & 0x00FF0000) >> 16);
  appData[6] = (byte)((pressure & 0x0000FF00) >> 8);
  appData[7] = (byte)((pressure & 0X000000FF));

  appData[8] = highByte(batteryVoltage);
  appData[9] = lowByte(batteryVoltage);

  appData[10] = highByte(batteryLevel);
  appData[11] = lowByte(batteryLevel);
}

void setup()
{
  if (ENABLE_SERIAL)
  {
    Serial.begin(SERIAL_SPEED);
  }

  boardInitMcu();

  // Read Data once in order to check if the Sensor is active and working
  readData();

  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
    printDevParam();
    LoRaWAN.init(loraWanClass, loraWanRegion);
    deviceState = DEVICE_STATE_JOIN;
    break;
  }
  case DEVICE_STATE_JOIN:
  {
    LoRaWAN.join();
    break;
  }
  case DEVICE_STATE_SEND:
  {
    prepareTxFrame(appPort);
    LoRaWAN.send();
    deviceState = DEVICE_STATE_CYCLE;
    break;
  }
  case DEVICE_STATE_CYCLE:
  {
    // Schedule next packet transmission
    txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    break;
  }
  case DEVICE_STATE_SLEEP:
  {
    LoRaWAN.sleep();
    break;
  }
  default:
  {
    deviceState = DEVICE_STATE_INIT;
    break;
  }
  }
}
