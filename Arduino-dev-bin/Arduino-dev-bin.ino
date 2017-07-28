#include <Arduino.h>
#include "TEE_UC20.h"
#include "internet.h"
#include "gnss.h"
#include "CMMC_Interval.hpp"
#include "tcp.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//#include <AltSoftSerial.h>
//#include <Sleep_n0m1.h>
//Sleep sleep;
//unsigned long sleepTime;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

GNSS gps;

INTERNET net;
TCP tcp;

CMMC_Interval interval2;

#define SHOW_RAM 1
#define DEBUG_SERIAL 1

#if SHOW_RAM
#include <MemoryFree.h>
#endif

#define APN "internet"
//#define APN "bmta.fleet"
#define USER ""
#define PASS ""

#define BINID            "90"
//#define APPID            "SmartTrash"


//AltSoftSerial mySerial;

#define LED 13
#define MODE_PIN A4
#define SS_pin                A5
#define _volume_OK            (1<<0)
#define _lidStatus_OK         (1<<1)
#define _temp_OK              (1<<2)
#define _humid_OK             (1<<3)
#define _flameStatus_OK       (1<<4)
#define _soundStatus_OK       (1<<5)
#define _carbon_OK            (1<<6)
#define _methane_OK           (1<<7)
#define _light_OK             (1<<8)
#define _pitch_OK             (1<<9)
#define _roll_OK              (1<<10)
#define _press_OK             (1<<11)
#define _batt_OK              (1<<12)


uint16_t is_data_OK = 0;

// #define RX_buffer_size 255
#define RX_buffer_size 140
volatile uint8_t RX_buffer[RX_buffer_size] = {0};
volatile uint16_t  RX_pointer = 0;
volatile uint16_t  Decode_pointer = 0;
volatile uint16_t prev = 0;

volatile char GNSS_data[58] = "";
String gps_data = "";
String gps_lat = "";
String gps_lon = "";
String gps_alt = "";

float _volume, _pitch, _roll, _batt, V_batt;
float _temp, _humid, _lat, _lon, _alt, _soundStatus;
uint16_t _lidStatus, _flameStatus, _press, _light, _carbon, _methane;

float _tempBME, _humidBME, _pressBME;

uint8_t gpsCounter = 0;
float subTime = 5;

#include "STM32.h"

#if DEBUG_SERIAL
void debug(String data) {
  Serial.println(data);
}
#endif

//////////////////////////////mainSETUP////////////////////////////////
bool open_tcp();

void setup()  {
  Serial.begin(9600);
#if DEBUG_SERIAL
  Serial.println(F("Program Start."));
#endif
#if SHOW_RAM
  Serial.print(F("freeMemory()="));
  Serial.println(freeMemory());
#endif

  //  sleepTime = 300000; // sleep 3 minute

  pinMode(LED, OUTPUT);
  pinMode(SS_pin, OUTPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);

  bme.begin();

  int z = 0;
  while (z < 5) {
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(10);
    z++;
  }

  if (digitalRead(MODE_PIN) == HIGH) {
    // subTime 1 hour
    subTime = 60;
#if DEBUG_SERIAL
    Serial.println("HIGH");
#endif
  } else {
    // subTime 3 minute 3 * 60 = 180
    subTime = 180 + 1000;
#if DEBUG_SERIAL
    Serial.println("LOW");
#endif
  }

  /////////////////////////////3G//////////////////////////////////
#if DEBUG_SERIAL
  gsm.Event_debug = debug;
#endif
  gsm.begin(&Serial3, 9600); // myserial
  gsm.PowerOn();
  while (gsm.WaitReady()) {}

  Serial.print(F("GetOperator --> "));
  Serial.println(gsm.GetOperator());
  Serial.print(F("SignalQuality --> "));
  _temp = gsm.SignalQuality();
  Serial.println(gsm.SignalQuality());

  Serial.println(F("Disconnect net"));
  net.DisConnect();
  net.Configure(APN, USER, PASS);
  net.Connect();
#if DEBUG_SERIAL
  Serial.println(F("NET Connected"));
#endif
  //////////////////////////////GPS//////////////////////////////
  gps.Start();
  gps.EnableNMEA();
  gps_data = gps.GetNMEA("GGA");
  gpsCounter = 0;
  bool gps_linked = true;
  uint32_t gpsTimeoutNextTick = millis() + 50 * 1000L ;
  while ((gps_data.substring(0, 8) == "$GPGGA,," ||
          gps_data.substring(0, 8) == "Please W")) {
    gps_data = gps.GetNMEA("GGA");

    // Serial.print("GNSS_data = ");
    // Serial.println(gps_data);

    digitalWrite(LED, HIGH);
    delay(5);
    digitalWrite(LED, LOW);
    delay(100);
    gpsCounter += 1;
    Serial.println("Wating GPS...");

    if (millis() > gpsTimeoutNextTick) {
      gps_lat = "0.0";
      gps_lon = "0.0";
      gps_alt = "0.0";
      gps_linked = false;
      break;
    }
  }

  if (gps_linked) {
    if (gps_data.substring(0, 6) == "$GPGGA") {
      gps_data.toCharArray(GNSS_data, 58);

      String gps_cal_s = gps_data.substring(18, 27);
      float gps_cal_f = gps_cal_s.toFloat();
      gps_cal_f /= 60.0f;
      uint32_t gps_cal_l = gps_cal_f * 10000000;

      gps_lat += GNSS_data[16] ;
      gps_lat += GNSS_data[17] ;
      gps_lat += ".";
      gps_lat += gps_cal_l;
      gps_lat += GNSS_data[28] ;

      gps_cal_s = gps_data.substring(33, 42);
      gps_cal_f = gps_cal_s.toFloat();
      gps_cal_f /= 60.0f;
      gps_cal_l = gps_cal_f * 10000000;

      gps_lon += GNSS_data[30] ;
      gps_lon += GNSS_data[31] ;
      gps_lon += GNSS_data[32] ;
      gps_lon += ".";
      gps_lon += gps_cal_l;
      gps_lon += GNSS_data[43] ;

      gps_alt += GNSS_data[54];
      gps_alt += GNSS_data[55];
      gps_alt += GNSS_data[56];
      // gps_alt += GNSS_data[57];
      // gps_alt += GNSS_data[58];

      Serial.print(gps_lat);
      Serial.print(F("  "));
      Serial.print(gps_lon);
      Serial.print(F("  "));
      Serial.println(gps_alt);
      delay(1000);
      Serial.println(F("Stop GPS"));
      gps.Stop();
      gps.DisableNMEA();
    }
  }
  else {
  }
  //  clear mode stm
  Sent_value(0xf1, &subTime);
  Sent_value(0xf1, &subTime);
  Sent_value(0xf1, &subTime);
}

bool open_tcp()
{
  Serial.println();
  // bool ret = tcp.Open("sock.traffy.xyz","10777");
  //  bool ret = tcp.Open("api.traffy.xyz", "10777");
  // bool ret = tcp.Open("red.cmmc.io","9991");
  bool ret = tcp.Open("128.199.143.200", "10777");
  return ret;
}

bool dirty = false;
static uint32_t nextTick;




//////////////////////////////mainLOOP////////////////////////////////
void loop() {
  //  interval2.every_ms(400, []() {
  //    // Sent_value(0xf1, &subTime);
  //    if (dirty == false) {
  //      read_uart();
  //      if (is_data_OK == 0x1fff) {
  //        dirty = true;
  //        digitalWrite(LED, HIGH);
  //        delay(20);
  //        digitalWrite(LED, LOW);
  //#if DEBUG_SERIAL
  //        Serial.print(F("VALID: "));
  //        Serial.println(is_data_OK, BIN);
  //#endif
  //      }
  //      else {
  //#if DEBUG_SERIAL
  //        Serial.print(F("INVALID: "));
  //        Serial.println(is_data_OK, BIN);
  //#endif
  //      }
  //    }
  //    else {
  //#if DEBUG_SERIAL
  //      Serial.println(F("DIRTY.. so skipped."));
  //#endif
  //    }
  //  });





  float mq4_co, mq9_ch4;
  //  if (dirty) {

  if (1) {

    _temp = bme.readTemperature();
    _humid = bme.readHumidity();
    _press = bme.readPressure() / 100.0F;

    pinMode(A0, INPUT);
    _batt = analogRead(A0);

    mq4_co = _carbon;
    mq9_ch4 = _methane;
    _carbon = MQ4GetGasPercentage((float)MQ4Read(_carbon) / 30.0f, 3);
    _methane = MQ9GetGasPercentage(MQ9Read(_methane) / 21.8f, 2);

    digitalWrite(LED, HIGH);
#if DEBUG_SERIAL
    Serial.print(F("print : "));
#endif
    _volume++;
    // publish("/" APPID "/gearname/" BINID "/data1", buffer, false);
    String data1 = String (BINID ":");
    String data_s = String(_volume) + "," + String(_lidStatus) + "," + String(_temp) + ","
                    + String(_humid) + "," + String(_flameStatus);
    data1 += data_s;
#if DEBUG_SERIAL
    Serial.println(data1);
#endif



    String data2 = String (BINID ":");
    data_s = String(_pitch) + "," + String(_roll) + "," + String(_press) + "," + String(_batt);
    data2 += data_s;
#if DEBUG_SERIAL
    Serial.println(data2);
#endif



    String data3 = String (BINID ":");
    data_s = String(_soundStatus) + "," + String(mq4_co) + "," +
             String(mq9_ch4) + "," + String(_light) + "," + String(subTime) + "," + String(millis() / 1000.00) + "," +
             String(_methane) + "," +
             String(_carbon);
    data3 += data_s;
#if DEBUG_SERIAL
    Serial.println(data3);
#endif



    String data4 = String (BINID ":");
    data_s = gps_lat + "," + gps_lon + "," + gps_alt;
    data4 += data_s;
    Serial.println(data4);

    bool tcpOpenFailed = false;
    int count_down = 100;
    while ( !open_tcp() && count_down ) {
#if DEBUG_SERIAL
      Serial.println("retry open tcp...");
#endif
      count_down--;
      delay(1000);
    }
    if (count_down == 0) {
      tcpOpenFailed = true;
#if DEBUG_SERIAL
      Serial.println("TCP Open failed...");
#endif
    }
    else {
      count_down = 100;
      while ( !tcp.StartSend() && count_down ) {
#if DEBUG_SERIAL
        Serial.println("retry send");
#endif
        count_down--;
        delay(500);
      }
      if (count_down) {
        tcp.println(data1);
        tcp.println(data2);
        tcp.println(data3);
        tcp.print(data4);
        tcp.StopSend();
      }
      count_down = 30;
      while ( !tcp.Close() && count_down ) {
        count_down--;
        delay(500);
#if DEBUG_SERIAL
        Serial.println("close tcp");
#endif
      }
    }

    Sent_value(0xf1, &subTime);
    delay(10);
    Sent_value(0xf1, &subTime);

    dirty = false;
    is_data_OK = 0;


    Serial.println(F("Sent..."));
  }

  Serial.println(F("gsm PowerOff zzZ"));
  gsm.PowerOff();
  delay(60000);
  asm volatile ("  jmp 0");


  //  sleep.pwrSaveMode();
  //  sleep.pwrDownMode();
  //  sleep.sleepDelay(sleepTime); // 300000 = 5 minute
  //  asm volatile ("  jmp 0");


  //  Serial.println(F("gsm.PowerOff"));
  //  delay(60000);
  //  Serial.println(F("RESET"));
  //  asm volatile ("  jmp 0");
}
