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
//Sleep sleepCtrl;
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
struct NODEStructure {
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
            memcpy(&mac, &tmp[j + 8], 6);
            MassageSave(mac, &tmp[j], i - j + 1);
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
    memset(Node1.buff, 0, sizeof(Node1.buff));
    memcpy(&Node1.buff, data, len);
    Node1.len = len;

    Serial.println("");
    Serial.println("data node1 comming....");

  } else if (CheckMac(tmp, Node2.mac)) {
    memset(Node2.buff, 0, sizeof(Node2.buff));
    memcpy(&Node2.buff, data, len);
    Node2.len = len;

    Serial.println("");
    Serial.println("data node2 comming....");

  } else if (CheckMac(tmp, Node3.mac)) {
    memset(Node3.buff, 0, sizeof(Node3.buff));
    memcpy(&Node3.buff, data, len);
    Node3.len = len;

    Serial.println("");
    Serial.println("data node3 comming....");

  } else if (CheckMac(tmp, Node4.mac)) {
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

        Serial.println("");
        Serial.println("data node1 Registered....");

        break;
      case 1:
        memcpy(&Node2.mac, tmp, 6);
        memset(Node2.buff, 0, sizeof(Node2.buff));
        memcpy(&Node2.buff, data, len);
        Node2.len = len;

        Serial.println("");
        Serial.println("data node2 Registered....");

        break;
      case 2:
        memcpy(&Node3.mac, tmp, 6);
        memset(Node3.buff, 0, sizeof(Node3.buff));
        memcpy(&Node3.buff, data, len);
        Node3.len = len;

        Serial.println("");
        Serial.println("data node3 Registered....");

        break;
      case 3:
        memcpy(&Node4.mac, tmp, 6);
        memset(Node4.buff, 0, sizeof(Node4.buff));
        memcpy(&Node4.buff, data, len);
        Node4.len = len;

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
  http.url("http://api.netpie.io/topic/SmartTrash/time?retain&auth=YGO1C5bATVNctTE:wN7khNDXgadngRN5WxMGMc7z0");
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
    // Serial.println("parsed OK.");
    JsonObject& netpieJsonObject = root[0];
    // Serial.print("TIME PAYLOAD: ");
    const char* payload = netpieJsonObject["payload"];
    const char* topic = netpieJsonObject["topic"];
    const char* lastUpdated = netpieJsonObject["lastUpdated"];
    long payloadInt = String(payload).toInt();
    // Serial.print("payload: ");
    // Serial.println(payload);
    //
    Serial.print("payloadInt: ");
    Serial.println(payloadInt);
    //
    // Serial.print("topic: ");
    // Serial.println(topic);
    //
    // Serial.print("lastUpdated: ");
    // Serial.println(lastUpdated);
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

String globalData0Version;
String globalData1;
String globalData2;
String globalData3;
String globalData4GPS;
String globalData5;

void builDataStringForTCPSocket() {
  String data_s;
  globalData0Version = String (BINID ":2,2,2,2,2");
  //  {
  //    globalData1 = String (BINID ":");
  //    data_s = String(_volume) + "," + String(_lidStatus) + "," + String(_temp) + ","
  //             + String(_humid) + "," + String(_flameStatus);
  //    globalData1 += data_s;
  //#if DEBUG_SERIAL
  //    Serial.println(globalData1);
  //#endif
  //    // DATA2 Preparation
  //    globalData2 = String (BINID ":");
  //    String _battery_percent = "0";
  //    data_s = String(_pitch) + "," + String(_roll) + "," + String(_press) + "," + String(_battery_percent);
  //    globalData2 += data_s;
  //#if DEBUG_SERIAL
  //    Serial.println(globalData2);
  //#endif
  //  }
  //  { // DATA3 Preparation
  //    globalData3 = String (BINID ":");
  //    data_s = String(_soundStatus) + "," + String(mq4_co) + "," +
  //             String(mq9_ch4) + "," + String(_light) + "," + String(globalSleepTimeFromNetpieInMemory * 60) + "," + String(millis() / 1000.00) + "," +
  //             String(_methane) + "," +
  //             String(_carbon);
  //    globalData3 += data_s;
  //#if DEBUG_SERIAL
  //    Serial.println(globalData3);
  //#endif
  //  }
  {
    // DATA4 Preparation
    globalData4GPS = String (BINID ":");
    String data_s = gps_lat + "," + gps_lon + "," + gps_alt + "," + _rssi ;
    globalData4GPS += data_s;
    Serial.println(globalData4GPS);
  }
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

bool writeDataStringToTCPSocket() {
  Serial.print(millis());
  Serial.println(" writeDataStringToTCPSocket");

  /*
    data0,version
    91:2,2,2,2,2
    data1:dist,lid,temp,humid,flame
    91:61.00,0,27.56,52.87,0
    data2:pitch,roll,pressure,batt
    91:0.00,0.00,969,621.00
    data3:sound,mq4,mq9,lux,sleep,millis(),ch4,co
    91:0.00,0.00,0.00,25,10,60.50,0,0
    data4:lat,lng,alt
    91:18.7828670N,098.9788563E,268
  */



  // tcp.println(globalData0Version);
  // tcp.println(globalData1);
  // tcp.println(globalData2);
  // tcp.println(globalData3);
  // tcp.println(globalData4GPS);
  // tcp.print(globalData5);

  // char dataInfo[]  = {
  //   0xfc, 0xfd, 0x18, 0xfe, 0x34, 0xdb, 0x3b, 0x98, 0x18, 0xfe, 0x34, 0xee, 0xcd, 0x53, 0x30,
  //   0xff, 0xfa, 0x1, 0x2, 0x3, 0x1, 0x28, 0xa, 0x0, 0x0, 0x38, 0x18, 0x0, 0x0, 0xe7, 0x3, 0x0, 0x0, 0x19, 0x5, 0x0,
  //   0x0, 0xa, 0x65, 0x73, 0x70, 0x4c, 0x6f, 0x67, 0x30, 0x30, 0x35, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  //   0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xd, 0xa
  // };

  // byte dataInfo[]  = {  // 21 data
  //   0xfa, 0xfb, 0xff, 0xff, 0xff, 0xff,
  //   0x5c, 0xcf, 0x7f, 0x9, 0x50, 0xa7,
  //   0x5e, 0xcf, 0x7f, 0x9, 0x50, 0xa7,
  //   0x3,
  //   0xd, 0xa
  // };

  // byte dataInfo[]  = {  // 70 data
  //   0xfc, 0xfd, 0x18, 0xfe, 0x34, 0xdb, 0x3b, 0x98, 0x18, 0xfe, 0x34, 0xee, 0xcd, 0x53, 0x30, // wrap
  //   0xff, 0xfa, 0x1, 0x2, 0x3, 0x1,   //  head data
  //   0x28, 0xa, 0x0, 0x0,    //  val1
  //   0x38, 0x18, 0x0, 0x0,   //  val2
  //   0xe7, 0x3, 0x0, 0x0,    //  val3
  //   0x19, 0x5, 0x0, 0x0,    //  batt
  //   0xa,                    //  name len
  //   0x65, 0x73, 0x70, 0x4c, 0x6f, 0x67, 0x30, 0x30, 0x35, 0x20,   //  name to String
  //   0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,   //  free data
  //   0xfa, 0xd, 0xa          //  end wrap
  //  };

  // char str[32] = "";
  // array_to_string(dataInfo, 21, str);



  // String dataInfoSent = "" ; //= "0xfa, 0xfb, 0xff, 0xff, 0xff, 0xff,0x5c, 0xcf, 0x7f, 0x9, 0x50, 0xa7,0x5e, 0xcf, 0x7f, 0x9, 0x50, 0xa7,0x3,0xd, 0xa";
  // dataInfoSent = String(dataInfo[0]) + String(dataInfo[1]) + String(dataInfo[2]);
  // tcp.println(globalData0Version);
  // tcp.println(globalData4GPS);


  //  Serial.print("\n");
  //  Serial.print("\n");
  //
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
  Serial.print("\n");

  char str[100] = "";

  delay(1000);
  Serial.println(F("StartSend Node 1 ..."));
  array_to_string(Node1.buff, Node1.len, str);
  tcp.StartSend();
  tcp.println(globalData0Version);
  tcp.println(str);
  tcp.print(globalData4GPS);
  tcp.StopSend();
  delay(1000);
  Serial.println(str);

  delay(1000);
  Serial.println(F("StartSend Node 1 ..."));
  array_to_string(Node1.buff, Node1.len, str);
  tcp.StartSend();
    tcp.println("KKao test");
  tcp.println(globalData0Version);

  Serial.print("\n");
  Serial.print("\n");

  for(int u = 0; u < Node1.len; u++){
    tcp.write((uint8_t)Node1.buff[u]);
    Serial.write((uint8_t)Node1.buff[u]);
  }
  Serial.print("\n");
  Serial.print("\n");

  tcp.print(globalData4GPS);
  tcp.StopSend();
  delay(1000);
  Serial.println(str);




  // Serial.println(F("StartSend Node 2 ..."));
  // array_to_string(Node2.buff, Node2.len, str);
  // tcp.StartSend();
  // tcp.println(globalData0Version);
  // tcp.println(str);
  // tcp.print(globalData4GPS);
  // tcp.StopSend();
  // delay(1000);
  // Serial.println(str);
  //
  // Serial.println(F("StartSend Node 3 ..."));
  // array_to_string(Node3.buff, Node3.len, str);
  // tcp.StartSend();
  // tcp.println(globalData0Version);
  // tcp.println(str);
  // tcp.print(globalData4GPS);
  // tcp.StopSend();
  // delay(1000);
  // Serial.println(str);
  //
  // Serial.println(F("StartSend Node 4 ..."));
  // array_to_string(Node4.buff, Node4.len, str);
  // tcp.StartSend();
  // tcp.println(globalData0Version);
  // tcp.println(str);
  // tcp.print(globalData4GPS);
  // tcp.StopSend();
  // delay(1000);
  // Serial.println(str);

  Serial.print(millis());
  Serial.println(" writeDataStringToTCPSocket");
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

long time_now, time_prev1, time_prev2 ;
uint8_t Peroid = 0;
void loop() {
  time_now = millis();

if (time_now > 5000)
  //if (time_now > 35000)
  {

    if (time_now < time_prev1) {
      asm volatile ("  jmp 0");
    }

    if (time_now - time_prev1 >= ((long)Peroid * 60000L)) {
      time_prev1 = time_now;
      //GET_Position();
      SentNodeData();
      Peroid = globalSleepTimeFromNetpieInMemory;
    }
  }

  if (time_now - time_prev2 >= 1000) {
    time_prev2 = time_now;
    digitalWrite(LED, !digitalRead(LED));
  }
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

  Serial.println(F("Disconnect net"));
  bool netDisConnectStatus = net.DisConnect();
  Serial.print("netDisconnect Status = ");
  Serial.println(netDisConnectStatus);
  if (netDisConnectStatus) {
    Serial.println("NET DISCONNECTED OK.");
  }
  else {
    Serial.println("NET DISCONNECTED FAILED.");
  }
  net.Configure(APN, USER, PASS);
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

  globalSleepTimeFromNetpieInMemory = getSleepTimeFromNetpie() / 20;

  Serial.print("SLEEP TIME [NETPIE] = ");
  Serial.println(globalSleepTimeFromNetpieInMemory);

  Serial.println(millis() / 1000);
  http.begin(1);

  builDataStringForTCPSocket();


  readAllSensors();
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
