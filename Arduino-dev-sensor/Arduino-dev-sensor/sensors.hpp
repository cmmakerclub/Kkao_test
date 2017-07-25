#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// float _volume, _pitch, _roll, _batt, V_batt;
// float _temp, _humid, _lat, _lon, _alt, _soundStatus;
// uint16_t _lidStatus, _flameStatus, _press, _light, _carbon, _methane;
int _rssi;

float _tempBME, _humidBME, _pressBME, _soundStatus, _batt;

#define ECHO  5
#define TRIG  7
extern uint8_t LED;
long duration;

void readAllSensors() {

  bme.begin();  // bme sensor begin
  _tempBME = bme.readTemperature();
  _humidBME = bme.readHumidity();
  _pressBME = bme.readPressure() / 100.0F;

  pinMode(A1, INPUT);
  _batt = analogRead(A1);

}

  int addSensorMsg(uint8_t *message, uint8_t *mac, String *gps) {
  const uint8_t MESSAGE_SIZE = 25;

  message[0] = 0xfa; //<---- header
  message[1] = 0xfb; //<---- header

  message[2] = 0xff; //<------ reserved
  message[3] = 0xff; //<------ reserved
  message[4] = 0xff; //<------ reserved
  message[5] = 0xff; //<------ reserved

  memcpy(&message[6], mac, 6); //<----------- mac1
  // message[6] = 0x5c; //<----------- mac1
  // message[7] = 0xcf;
  // message[8] = 0x7f;
  // message[9] = 0x09;
  // message[10] = 0x50;
  // message[11] = 0xa7;

  message[12] = 0xff; //<----------- mac2
  message[13] = 0xff;
  message[14] = 0xff;
  message[15] = 0xff;
  message[16] = 0xff;
  message[17] = 0xff;

  message[18] = 0x14; //<----------- lenght


  uint32_t field1 = _tempBME*100;
  uint32_t field2 = _humidBME*100;
  uint32_t field3 = _pressBME;
  uint32_t field4 = _soundStatus*100;
  uint32_t field5 = _batt;

  Serial.println(field1, HEX);
  Serial.println(field2, HEX);
  Serial.println(field3, HEX);
  Serial.println(field4, HEX);
  Serial.println(field5, HEX);

  memcpy(&message[19 + 4*0], (const void *)&field1, 4);
  memcpy(&message[19 + 4*1], (const void *)&field2, 4);
  memcpy(&message[19 + 4*2], (const void *)&field3, 4);
  memcpy(&message[19 + 4*3], (const void *)&field4, 4);
  memcpy(&message[19 + 4*4], (const void *)&field5, 4);

  Serial.println("==========================");
  Serial.println("       print data         ");
  Serial.println("==========================");

  //
  char gpsBuffer[50];
  uint8_t gpsLength = gps->length();
  gps->getBytes(gpsBuffer, gpsLength);
  message[20 + 4*5] = gps->length();
  memcpy(&message[21 + 4*5], gpsBuffer, gpsLength);

  Serial.print("____GPS LEN =");
  Serial.println(gps->length());
  Serial.print("____GPS VAL =");
  Serial.println(gpsBuffer);
  uint8_t sum = 0;
  for (uint8_t i = 0; i < (21 + 4*5 + gpsLength); i++) {
    sum ^= message[i];
  }

  message[21 + 4*5 + gps->length()+1] = sum;
  message[21 + 4*5 + gps->length()+2] = 0x0d;
  message[21 + 4*5 + gps->length()+3] = 0x0a;

  return 21 + 4*5 + gpsLength+3 +1;
}
