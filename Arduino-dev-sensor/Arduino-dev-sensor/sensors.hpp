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

  pinMode(A0, INPUT);
  _batt = analogRead(A0);

}

void addSensorMsg(uint8_t *message) {
  const uint8_t MESSAGE_SIZE = 25;
    uint32_t field1 = _tempBME*100;
    uint32_t field2 = _humidBME*100;
    uint32_t field3 = _pressBME;
    uint32_t field4 = _soundStatus*100;
    uint32_t field5 = _batt;

    message[0] = 0xff;
    message[1] = 0xfb;

    memcpy(&message[2], (const void *)&field1, 4);
    memcpy(&message[6], (const void *)&field2, 4);
    memcpy(&message[10], (const void *)&field3, 4);
    memcpy(&message[14], (const void *)&field4, 4);
    memcpy(&message[18], (const void *)&field5, 4);

    Serial.println("==========================");
    Serial.println("       print data         ");
    Serial.println("==========================");
    uint8_t sum = 0;
    for (uint8_t i = 0; i < MESSAGE_SIZE-3; i++) {
      sum ^= message[i];
    }
    message[MESSAGE_SIZE-3] = sum;
    message[MESSAGE_SIZE-2] = 0x0d;
    message[MESSAGE_SIZE-1] = 0x0a;

}
