/*********************************************************************************
 * Code adapted from the Node Building Workshop using a modified LoraTracker board
 * 
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Copyright (c) 2017 Caspar Armster for the modifications concerning 
 * a) the sensor support for the AM2302 and SDS011
 * b) the battery voltage measurement
 * c) integration of the lora-serialization library
 * d) the Pololu 5V StepUp U1V11F5 support
 * 
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This sketch will send Battery Voltage (in mV), Temperature (in Celsius),
 * Humidity (in %) and PM10/PM2.5 counts using the lora-serialization library 
 * matching setttings have to be added to the payload decoder funtion in the
 * The Things Network console/backend.
 * 
 * The Application will 'sleep' 75x8 seconds (10 minutes) and then run the
 * SDS011 sensor for 30 seconds to get a good reading on the pm2.5 and
 * pm10 count. You can adjust those sleep and uptimes with the variables
 * "int sleepcycles = 75;" and "#define sdsSamples 30"
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey. Do not forget to adjust the payload decoder function.
 * 
 * In the payload function change the decode function, by adding the code from
 * https://github.com/thesolarnomad/lora-serialization/blob/master/src/decoder.js
 * to the function right below the "function Decoder(bytes, port) {" and delete
 * everything below exept the last "}". Right before the last line add this code
 * "return decode(bytes, [uint16, uint16, uint16, temperature, humidity], ['battery', 'pm25', 'pm10', 'temp', 'humi']);"
 * and you get a json containing the stats for battery, pm25, pm10, temp and humi
 *
 *********************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <LoraEncoder.h>
#include <LoraMessage.h>
#include <TvBLoRaWANNodeRev2.h>
#include "DHT.h"
#include <Arduino.h>
#include "ttn_secrets.h"

TvBLoRaWANNodeRev2 TvBLoRaWANNodeRev2;
#define DHTPIN 7
#define DHTTYPE DHT22     // DHT 22  (AM2302), AM2321
//DHT dht(DHTPIN, DHTTYPE);
float humi = 0;
float temp = 0;

#define sdsLEN 9          // Byte length of recieved data
#define sdsSleepPin 3     // Pin to set SDS011 to sleep with a Pololu 5V StepUp U1V11F5
#define sdsSamples 30     // Number of samples to take (equals the time the sensor runs in seconds)
unsigned char incomingByte = 0;
unsigned char buf[sdsLEN];
int PM25 = 0;
int PM10 = 0;

int sleepcycles = 4;     // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec, 75 = 10min
bool joined = false;
bool sleeping = false;
int port = 4;

// LoRaWAN keys
static const u1_t app_eui[8]  = SECRET_APP_EUI;
static const u1_t dev_eui[8]  = SECRET_DEV_EUI;
static const u1_t app_key[16] = SECRET_APP_KEY;

// provide APPEUI (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

// provide DEVEUI (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}

// provide APPKEY key (16 bytes)
void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);
}

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping for loratracker.uk PCB
const lmic_pinmap lmic_pins = {
    .nss = LORAWAN_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LORAWAN_DIO0, LORAWAN_DIO1, LORAWAN_DIO2},
};

void onEvent (ev_t ev) {
  int i,j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING: 
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);      
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        i=(LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i>10) {
          i=10;     // maximum number of BLINKs
        }
        for(j=0;j<i;j++) {
          TvBLoRaWANNodeRev2.Blink(LED_RED, 200, 0);
          delay(400);
        }
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      TvBLoRaWANNodeRev2.Blink(LED_GREEN, 1000, 0);
      delay(50);  // delay to complete Serial Output before Sleeping
      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void loop_sds() {
  int i, j;
  int PM25Sample[sdsSamples];
  int PM10Sample[sdsSamples];
  unsigned char checksum;
  //analogWrite(sdsSleepPin, 255);
  for(j=0; j<sdsSamples; j++) {
    PM25Sample[j] = 0;
    PM10Sample[j] = 0;
    while((PM25Sample[j] == 0) && (PM10Sample[j] == 0)) {
      incomingByte = 0;
      checksum = 0;
      if (Serial1.available() > 0) {
        incomingByte = Serial1.read(); 
      }
      if (incomingByte == 0xAA) {
        Serial1.readBytes(buf, sdsLEN);
        if ((buf[0] == 0xC0) && (buf[8] == 0xAB)) {
          for (i=1; i<=6; i++) {
            checksum = checksum + buf[i];
          }
          if (checksum == buf[7]) {
            PM25Sample[j]=((buf[2]<<8) + buf[1])/10;
            PM10Sample[j]=((buf[4]<<8) + buf[3])/10;
          }
          else {
            Serial.println("SDS011 checksum Error");
          }
        }
        else {
          Serial.println("SDS011 frame error");
        }
      }
    }
    // ca. every second there is new data available, so wait for close to one second
    delay(800);
  }
  for(j=0; j<sdsSamples; j++) {
    PM25 = PM25 + PM25Sample[j];
    PM10 = PM10 + PM10Sample[j];
  }
  PM25 = round(PM25 / sdsSamples);
  PM10 = round(PM10 / sdsSamples);
  //analogWrite(sdsSleepPin, 0);
}

void do_send(osjob_t* j) {
  //float humi = dht.readHumidity();
  //float temp = dht.readTemperature();
  // getting sensor values
  Serial.print(F("TEMP: "));
  Serial.print(temp);
  Serial.print(F(" HUMI: "));
  Serial.print(humi);
  loop_sds();
  // looping through the data from the SDS011 for sdsSamples times
  Serial.print(F(" PM25: "));
  Serial.print(PM25);
  Serial.print(F(" ug/m3"));
  Serial.print(F(" PM10: "));
  Serial.print(PM10);
  Serial.print(F(" ug/m3"));
  float vccValue = TvBLoRaWANNodeRev2.Voltage();
  Serial.print(F(" Battery: "));
  Serial.print(vccValue);
  Serial.println(F(" V"));
  LoraMessage message;
  message
    .addUint16(vccValue*1000)
    .addUint16(PM25)
    .addUint16(PM10)
    .addTemperature(temp)
    .addHumidity(humi);
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(port, message.getBytes(), message.getLength() , 0);
    Serial.println(F("Sending: "));
  }
}

// initial job
static void initfunc (osjob_t* j) {
  // reset MAC state
  LMIC_reset();
  // got this fix from forum: https://www.thethingsnetwork.org/forum/t/over-the-air-activation-otaa-with-lmic/1921/36
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // start joining
  LMIC_startJoining();
  // init done - onEvent() callback will be invoked...
}

void setup() {
  TvBLoRaWANNodeRev2.Init();
  Serial.begin(115200);
  Serial.println(F("Starting"));
  //dht.begin();
  //analogReference(INTERNAL); //reference will be 1,1V internal on 3.3V Arduino
  // LED is connected to pin 10, if this port is NOT set as output before
  //pinMode(LedPin, OUTPUT);
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  os_setCallback(&initjob, initfunc);
  LMIC_reset();
  // Setting up connection to the SDS011 sensor
  //pinMode(sdsSleepPin, OUTPUT);
  Serial1.begin(9600);
}

void loop()
{
  // start OTAA JOIN
  if (joined == false) {
    os_runloop_once();
  } else {
    do_send(&sendjob);            // Sent sensor values
    while(sleeping == false) {
      os_runloop_once();
    }
    sleeping = false;
    for (int i=0;i<sleepcycles;i++) {
      TvBLoRaWANNodeRev2.Blink(LED_BLUE, 100, 0);
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);           //sleep 8 seconds
    }
  }
}

