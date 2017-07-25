#define SERIAL_RX_BUFFER_SIZE 256

#include <Arduino.h>
#include <SPI.h>
#include "TEE_UC20.h"
#include "internet.h"
//#include "CMMC_Interval.hpp"
#include "tcp.h"
#include <EEPROM.h>
#include <Wire.h>
#include "File.h"
#include "http.h"
#include <ArduinoJson.h>

#define BINID     "100"
uint8_t LED = 13;
#define MODE_PIN A4

#include "./sensors.hpp"
#include "gps.hpp"

struct EEPROMStructure {
  double lat;
  double lng;
  uint32_t sleepTimeS;
};

HTTP http;
float eepromFloatInitializedByte = 0.000f;
EEPROMStructure globalCachedEEPROM;
int eeAddress = 0;

extern bool gotGPSLocation;

// bool ret = tcp.Open("sock.traffy.xyz","Connecting to... ");
//  bool ret = tcp.Open("api.traffy.xyz", "10777");
// String TCP_SERVER_ENDPOINT = "128.199.143.200";
// String TCP_SERVER_PORT     = "10777";
// String TCP_SERVER_ENDPOINT = "mqtt.cmmc.io";
// String TCP_SERVER_PORT     = "1883";

String TCP_SERVER_ENDPOINT = "api.traffy.xyz";
String TCP_SERVER_PORT     = "10778";

INTERNET net;
TCP tcp;
UC_FILE file;

///////////////////////////////////////////////////////////////////////////////

static uint32_t Sound_sumDBm, Sound_count;
static uint16_t Sound_max, Sound_min;

struct NODEStructure {
  uint8_t nid;
  uint8_t mac[6];
  uint8_t buff[80];
  uint8_t len;
};

#define tmpSize  100
uint8_t tmp[tmpSize];
uint8_t tmp_index;

NODEStructure Node1;
NODEStructure Node2;
NODEStructure Node3;
NODEStructure Node4;


void serialEvent2() {
  while (Serial2.available() && !((tmp[tmp_index - 1] == 0x0A) && (tmp[tmp_index - 2] == 0x0D))) {
    tmp[tmp_index] = Serial2.read();
    tmp_index++;
    if (tmp_index >= tmpSize)tmp_index = 0;
  }
  MassageAnalysis();
}

uint8_t MassageAnalysis() {
  uint8_t mac[6] = {0};
  for (int i = tmp_index; i >= 2; i--) {
    if ((tmp[i - 1] == 0x0A) && (tmp[i - 2] == 0x0D)) {
      tmp_index = 0;
      for (int j = 0; j < i; j++) {
        if ((tmp[j] == 0xFC) && (tmp[j + 1] == 0xFD)) {
          uint8_t sum = 0;
          for (uint8_t k = 0; k <=  (i - j - 2); k++) {
            sum ^= tmp[j + k];
          }
          if (sum == tmp[i - j - 2]) {
            digitalWrite(A3,1);
            memcpy(&mac, &tmp[j + 8], 6);
            MassageSave(mac, &tmp[j], i - j);
            digitalWrite(A3,0);
            return 1;
          } else {
            return 0;
          }
        }
      }
    }
  }
  return 0;
}

uint8_t CheckMac(uint8_t* mac1, uint8_t* mac2) {
  uint8_t val = 0x01;
  for (int i = 0; i < 6; i++) {
    if (mac1[i] != mac2[i]) {
      return 0;
    }
  }
  return 1;
}

void MassageSave(uint8_t* tmp, uint8_t* data, uint8_t len) {
  static int index;
  if (CheckMac(tmp, Node1.mac)) {
    Node1.nid = 1;
    memset(Node1.buff, 0, sizeof(Node1.buff));
    memcpy(&Node1.buff, data, len);
    Node1.len = len;

    Serial.println("");
    Serial.println("data node1 comming....");

  } else if (CheckMac(tmp, Node2.mac)) {
    Node2.nid = 2;
    memset(Node2.buff, 0, sizeof(Node2.buff));
    memcpy(&Node2.buff, data, len);
    Node2.len = len;

    Serial.println("");
    Serial.println("data node2 comming....");

  } else if (CheckMac(tmp, Node3.mac)) {
    Node3.nid = 3;
    memset(Node3.buff, 0, sizeof(Node3.buff));
    memcpy(&Node3.buff, data, len);
    Node3.len = len;

    Serial.println("");
    Serial.println("data node3 comming....");

  } else if (CheckMac(tmp, Node4.mac)) {
    Node4.nid = 4;
    memset(Node4.buff, 0, sizeof(Node4.buff));
    memcpy(&Node4.buff, data, len);
    Node4.len = len;

    Serial.println("");
    Serial.println("data node4 comming....");

  } else {
    switch (index) {
      case 0:
        memcpy(&Node1.mac, tmp, 6);
        memset(Node1.buff, 0, sizeof(Node1.buff));
        memcpy(&Node1.buff, data, len);
        Node1.len = len;
        Node1.nid = 1;
        Serial.println("");
        Serial.println("data node1 Registered....");
        break;
      case 1:
        memcpy(&Node2.mac, tmp, 6);
        memset(Node2.buff, 0, sizeof(Node2.buff));
        memcpy(&Node2.buff, data, len);
        Node2.len = len;
        Node2.nid = 2;

        Serial.println("");
        Serial.println("data node2 Registered....");

        break;
      case 2:
        memcpy(&Node3.mac, tmp, 6);
        memset(Node3.buff, 0, sizeof(Node3.buff));
        memcpy(&Node3.buff, data, len);
        Node3.len = len;
        Node3.nid = 3;

        Serial.println("");
        Serial.println("data node3 Registered....");

        break;
      case 3:
        memcpy(&Node4.mac, tmp, 6);
        memset(Node4.buff, 0, sizeof(Node4.buff));
        memcpy(&Node4.buff, data, len);
        Node4.len = len;
        Node4.nid = 4;

        Serial.println("");
        Serial.println("data node4 Registered....");

        break;
    }
    index++;
    if (index == 4) index = 0;
  }
}


///////////////////////////////////////////////////////////////////////////////

long globalSleepTimeFromNetpieInMemory = 3;

#define SHOW_RAM 1
#define DEBUG_SERIAL 1

#if SHOW_RAM
#include <MemoryFree.h>
#endif

//#define APN "internet"
#define APN "bmta.fleet"
#define USER ""
#define PASS ""

//#define APPID           "SmartTrash"

// uint8_t stmSleepTimeInMinute = 10;

#if DEBUG_SERIAL
void debug(String data) {
  Serial.println(data);
}
#endif

void setEEProm() {
  EEPROM.get(eeAddress, eepromFloatInitializedByte);
  Serial.println(eepromFloatInitializedByte, 3);

  // intialize EEPROM
  if (eepromFloatInitializedByte != 123.456f) {
    eepromFloatInitializedByte = 123.456f;
    // write first signature byte
    EEPROM.put(eeAddress, eepromFloatInitializedByte);
    // intialize eeprom structure
    EEPROMStructure defaultEEPROMValue = { 0.0f, 0.0f, 10 };
    eeAddress += sizeof(float);

    // write default value to eeprom
    EEPROM.put(eeAddress, defaultEEPROMValue);
    eeAddress += sizeof(EEPROMStructure);

    // LOAD EEPROM to global cache
    Serial.println("Initialized EEPROM");
    Serial.println("LOAD EEPROM to globalCachedEEPROM");
    EEPROM.get(sizeof(float), globalCachedEEPROM);
    printEEPROMInformation();
    delay(100);
  }
  else { /* load EEPROM */
    // eeAddress = sizeof(float);
    Serial.println("========================");
    Serial.println("LOADING CACHED IN EEPROM");
    Serial.println("========================");
    EEPROM.get(0 + sizeof(float), globalCachedEEPROM);
    printEEPROMInformation();
    delay(100);
  }
}

String netpieJsonString;

long getSleepTimeFromNetpie() {
  Serial.println(F("Send HTTP GET"));
  http.url("http://api.netpie.io/topic/SmartTrash/time/master/3?auth=xTsWAyTWJk3Ba5h:3UzQJ3DeGT50PwfwlmJE0vQF9");
  Serial.println(http.get());
  // Serial.println(F("Clear data in RAM"));
  file.Delete(RAM, "*");
  Serial.println(F("Save HTTP Response To RAM"));
  http.SaveResponseToMemory(RAM, "netpie.json");
  // Serial.println(F("Read data in RAM"));

  // clear String
  netpieJsonString = "";
  read_file(RAM, "netpie.json");
  // Serial.println("READ FILE JSON");
  Serial.println(netpieJsonString);
  DynamicJsonBuffer jsonBuffer;
  JsonArray& root = jsonBuffer.parseArray(netpieJsonString.c_str());

  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }
  else {
    JsonObject& netpieJsonObject = root[0];
    const char* payload = netpieJsonObject["payload"];
    const char* topic = netpieJsonObject["topic"];
    const char* lastUpdated = netpieJsonObject["lastUpdated"];
    long payloadInt = String(payload).toInt();
    Serial.print("payloadInt: ");
    Serial.println(payloadInt);
    return payloadInt;
  }
};


void data_out(char data)
{
  netpieJsonString += String(data);
}

void read_file(String pattern, String file_name)
{
  file.DataOutput =  data_out;
  file.ReadFile(pattern, file_name);
}

bool open_tcp() {
  Serial.println("===========");
  Serial.println("open_tcp");
  Serial.print("Connecting to... ");
  Serial.print(TCP_SERVER_ENDPOINT);
  Serial.print(":");
  Serial.println(TCP_SERVER_PORT);
  Serial.println("===========");
  bool ret = false;
  // uint32_t _timeout = (60*1000L) + millis();
  uint32_t max_retries = 30;
  while (0 == tcp.Open(TCP_SERVER_ENDPOINT, TCP_SERVER_PORT) ) {
    Serial.println("... OPEN TCP SOCKET FAILED....");
    delay(100);
    // Serial.print("timeout = ");
    // Serial.println(_timeout);
    // Serial.print("millis() = ");
    // Serial.println(millis());
    if (--max_retries == 0) {
      Serial.println("more than 30 seconds ... reboot");
      asm volatile ("  jmp 0");
    }
  }

  Serial.println("TCP Connected..");
  ret = true;
  return ret;
}

void debugSlave() {
  Serial.println("======== DEBUG SLAVE ========");
  Serial.println(millis());
  Serial.print("MAC1 ");
  for (int i = 0; i < 6; i++) {
    Serial.print(Node1.mac[i], HEX);
    Serial.print(" ");
  }

  //  Serial.print("\t");  Serial.print("\t");
  //  for (int i = 0; i < Node1.len; i++) {
  //    Serial.print(Node1.buff[i], HEX);
  //    Serial.print(" ");
  //  }
  //
  Serial.print("\n");
  Serial.print("MAC2 ");
  for (int i = 0; i < 6; i++) {
    Serial.print(Node2.mac[i], HEX);
    Serial.print(" ");
  }
  //  Serial.print("\t");  Serial.print("\t");
  //  for (int i = 0; i < Node2.len; i++) {
  //    Serial.print(Node2.buff[i], HEX);
  //    Serial.print(" ");
  //  }
  Serial.print("\n");
  Serial.print("MAC3 ");
  for (int i = 0; i < 6; i++) {
    Serial.print(Node3.mac[i], HEX);
    Serial.print(" ");
  }
  //  Serial.print("\t");  Serial.print("\t");
  //  for (int i = 0; i < Node3.len; i++) {
  //    Serial.print(Node3.buff[i], HEX);
  //    Serial.print(" ");
  //  }
  Serial.print("\n");
  Serial.print("MAC4 ");
  for (int i = 0; i < 6; i++) {
    Serial.print(Node4.mac[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\n");
  Serial.println("/======== DEBUG SLAVE ========");
}


void writeForwaredSensorFromSlave(NODEStructure &node) {
  debugSlave();
  Serial.println(F("StartSend Node 1 ..."));
  Serial.println("Caling StartSend");
  tcp.StartSend();
  delay(1000);
  Serial.print("----- NODE FORWARD HEX FROM ID = ");
  Serial.print(node.nid);
  Serial.println(" -----");
  for(int u = 0; u < node.len; u++){
    tcp.write((uint8_t)node.buff[u]);
    Serial.print(node.buff[u], HEX);
    delay(1);
  }
  Serial.println("Calling StopSend..");
  tcp.StopSend();
  Serial.println("finish!");
}

void writeArduinoSensor() {
  // ARDUINO's SENSOR
  // {
  //   Serial.println("Being Sent");
    #define SensorMsg_size 100
    char SensorMsg[SensorMsg_size] = "";
    String GpsMsg = gps_lat + "," + gps_lon + "," + gps_alt + "," + _rssi;
    int sensor_len = addSensorMsg((uint8_t *)&SensorMsg, (uint8_t *)&Node1.buff[8],
        (uint8_t *)&GpsMsg);

    Serial.print("\n");
    Serial.print("\n");

    delay(1000);
    tcp.StartSend();
  //   // Serial.println();
    Serial.println("----- Sensor Value HEX -----");
    Serial.println(sensor_len);
    for(int u = 0; u < sensor_len; u++){
      tcp.write((uint8_t)SensorMsg[u]);
      Serial.print((uint8_t)SensorMsg[u], HEX);
    }
    Serial.println();
    tcp.StopSend();
    delay(1000);
  // }
}

bool writeDataStringToTCPSocket() {
  Serial.println("Write From Slave");

  uint8_t tmp[6] = {0};

  if(!CheckMac(tmp, Node1.mac)){
    delay(1000);
    writeArduinoSensor();
    delay(1000);
    writeForwaredSensorFromSlave(Node1);
  }

  if(!CheckMac(tmp, Node2.mac)){
    delay(1000);
    writeForwaredSensorFromSlave(Node2);
  }

  if(!CheckMac(tmp, Node3.mac)){
    delay(1000);
    writeForwaredSensorFromSlave(Node3);
  }

  if(!CheckMac(tmp, Node4.mac)){
    delay(1000);
    writeForwaredSensorFromSlave(Node4);
  }

  delay(1000);

  Serial.println("Write Arduino Sensor");
}

void array_to_string(byte array[], unsigned int len, char buffer[])
{
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i * 2 + 1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len * 2] = '\0';
}

void sendSleepTimeInSecondToNode(uint8_t stmSleepTimeInMinute) {
  // writeSleep to STM
  Serial.print("Send stemSleepTimeToSTM => ");
  Serial.print(stmSleepTimeInMinute);
  Serial.println(" min.");

  Serial2.write(stmSleepTimeInMinute);
  delay(10);
  Serial2.write(stmSleepTimeInMinute);
  delay(10);
  Serial2.write(stmSleepTimeInMinute);
  delay(10);
  // while(Serial2.available()) {
  //   Serial.println(Serial2.read());
  // }
  Serial.println(F("Sent..."));
}

void printEEPROMInformation() {
  Serial.println("====================");
  Serial.println("  CACHED EEPROM  ");
  Serial.println("====================");
  Serial.print("globalCachedEEPROM.lat= ");
  Serial.println(globalCachedEEPROM.lat);
  Serial.print("globalCachedEEPROM.lng= ");
  Serial.println(globalCachedEEPROM.lng);
  Serial.print("globalCachedEEPROM.sleepTimeS = ");
  Serial.println(globalCachedEEPROM.sleepTimeS);
  Serial.println("====================");
  Serial.println("====================");
}

void sleepArduino(uint32_t sleepTimeInMs) {
  Serial.println(F("gsm PowerOff zzZ"));
  // Serial.print("sleep for");
  gsm.PowerOff();
  //  sleep.pwrSaveMode();
  //  sleepCtrl.pwrDownMode();
  // STM Sleep for n seconds
  Serial.print("Being sleep for ..");
  Serial.println(sleepTimeInMs);
  Serial.println(millis());
  Serial.println("go to bed...");
  //  sleepCtrl.sleepDelay(sleepTimeInMs); // in MS
  // Arduino Reset
  asm volatile ("  jmp 0");
}

void sendDataOverTCPSocket() {

  Serial.println("sendDataOverTCPSocket");

  while (!open_tcp())
  {
    Serial.println("[2] Opening tcp...");
    delay(1000);
  }

  writeDataStringToTCPSocket();

  while (!tcp.Close()) {
    Serial.println("[2] Closing tcp...");
    delay(1000);
  }

}

//////////////////////////////mainSETUP////////////////////////////////

void setup()  {
  Serial.begin(115200);
  Serial2.begin(9600);  //  serial connect to esp8266

  Serial.println(F("Program Start."));
  Serial.print(F("freeMemory()="));
  Serial.println(freeMemory());

  pinMode(LED, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  //  setEEProm();
  //  bme.begin();  // bme sensor begin

  // Just blink when program started
  int z = 0;
  while (z < 5) {
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                // wait for a second
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(10);
    z++;
  }
}

//////////////////////////////mainLOOP////////////////////////////////
long time_now, time_prev1, time_prev2, time_prev3 ;
uint8_t Peroid = 0;
void loop() {
  time_now = millis();
  if (time_now > (15000L)) {
      if (time_now < time_prev1) {
        asm volatile ("  jmp 0");
      }

      if (time_now - time_prev1 >= ((long)Peroid * 60000L)) {
        time_prev1 = time_now;

        readAllSensors();
        GET_Sound ();

        // Serial.println("====  Printing Sensor Values ======");
        // Serial.print("\treadTemperature= ");
        // Serial.print(_tempBME);
        // Serial.print("\treadHumidity= ");
        // Serial.print(_humidBME);
        // Serial.print("\treadPressure= ");
        // Serial.print(_pressBME);
        // Serial.print("\tSound= ");
        // Serial.print(_soundStatus);
        // Serial.print("\tanalogRead= ");
        // Serial.println(_batt*30/1023);
        // Serial.println("====  Printing Sensor Values ======");

        if((_batt*30/1023) > 9.0f){
          GET_Position();
          SentNodeData();
        }

        Peroid = globalSleepTimeFromNetpieInMemory;
      }
    }

    if (time_now - time_prev2 >= 1000) {
      time_prev2 = time_now;


      // readAllSensors();
      // GET_Sound ();
      //
      // Serial.println("====  Printing Sensor Values ======");
      // Serial.print("\treadTemperature= ");
      // Serial.print(_tempBME);
      // Serial.print("\treadHumidity= ");
      // Serial.print(_humidBME);
      // Serial.print("\treadPressure= ");
      // Serial.print(_pressBME);
      // Serial.print("\tSound= ");
      // Serial.print(_soundStatus);
      // Serial.print("\tanalogRead= ");
      // Serial.println(_batt*30/1023);
      // Serial.println("====  Printing Sensor Values ======");


      digitalWrite(LED, !digitalRead(LED));
    }

    if (time_now - time_prev3 >= 100) {
      time_prev3 = time_now;

      int16_t tmp = abs(Sound_max - Sound_min);
      float ttmp = (13.619551*(log(tmp))+ 35);
      if(ttmp > 0 && ttmp < 120) {
        Sound_sumDBm += ttmp;
        Sound_count++;
      }
      Sound_max = 0;
      Sound_min = 1023;
    }

    uint16_t Sound_tmp = analogRead(A0);
    if(Sound_tmp > Sound_max) Sound_max = Sound_tmp;
    if(Sound_tmp < Sound_min) Sound_min = Sound_tmp;
  }

void SentNodeData (void) {

  /////////////////////////////3G//////////////////////////////////
#if DEBUG_SERIAL
  // gsm.Event_debug = debug;
#endif
  gsm.begin(&Serial3, 9600); // myserial
  gsm.PowerOn();
  while (gsm.WaitReady()) {}

  Serial.print(F("GetOperator --> "));
  Serial.println(gsm.GetOperator());
  Serial.print(F("SignalQuality --> "));
  _rssi = gsm.SignalQuality();
  Serial.println(gsm.SignalQuality());

  Serial.println(F("Calling net.DisConnect"));
  bool netDisConnectStatus = net.DisConnect();
  Serial.print("netDisconnect Status = ");
  Serial.println(netDisConnectStatus);
  net.Configure(APN, USER, PASS);
  Serial.println(F("Calling net.Connect"));
  bool netConnectStatus = net.Connect();
  Serial.print("netConnectStatus = ");
  Serial.println(netConnectStatus);
  if (netConnectStatus == 0) {
    Serial.println("net.Connect failed.");
    // asm volatile ("  jmp 0");
  }
  else {
    Serial.println(F("NET Connected"));
  }

  Serial.println(F("Show My IP"));
  Serial.println(net.GetIP());

  globalSleepTimeFromNetpieInMemory = getSleepTimeFromNetpie();

  Serial.print("SLEEP TIME [NETPIE] = ");
  Serial.println(globalSleepTimeFromNetpieInMemory);

  Serial.println(millis() / 1000);
  http.begin(1);

  // builDataStringForTCPSocket();
  // readAllSensors();

  sendDataOverTCPSocket();

  gsm.PowerOff();

  Serial.println("  Being slept...");

  sendSleepTimeInSecondToNode(globalSleepTimeFromNetpieInMemory);

}

void GET_Position (void) {
  gsm.begin(&Serial3, 9600); // myserial
  gsm.PowerOn();
  while (gsm.WaitReady()) {}
  //////////////////////////////GPS//////////////////////////////
  startGPSService();
  //////////////////////////////GPS//////////////////////////////
  gsm.PowerOff();

}

void GET_Sound (void) {
  _soundStatus = ((float)Sound_sumDBm/Sound_count);
  Sound_sumDBm =  0;
  Sound_count = 0;
}
