#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "DFRobot_SHT20.h"

struct Linear {
  float m;
  float b;
};

class Variable {
  private:
    float _value = 0.0;
    Linear _eq;
  public:
    void begin(float m, float b) {
      _eq.m = m;
      _eq.b = b;
    }
    void set(float value) {
      _value = (_eq.m * value) + _eq.b;
    }
    void force(float value) {
      _value = value;
    }
    float get(void) {
      return _value;
    }
    String convert(uint8_t i) {
      return String(_value, i);
    }
    float map(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    void clear(void) {
      _value = 0.0;
    }
};

#define IN_SENSOR_ADDRESS 0x30
#define OUT_SENSOR_ADDRESS 0x31
#define IN_SENSOR_PIN D3
#define OUT_SENSOR_PIN D4

Adafruit_VL53L0X in_sensor = Adafruit_VL53L0X();
Adafruit_VL53L0X out_sensor = Adafruit_VL53L0X();
DFRobot_SHT20 humidity_sensor;

VL53L0X_RangingMeasurementData_t in_measure;
VL53L0X_RangingMeasurementData_t out_measure;

Variable temperature;
Variable humidity;
uint16_t _values_in[5] = {0, 0, 0, 0, 0};
uint16_t _values_out[5] = {0, 0, 0, 0, 0};
uint8_t people = 0;

void initialize(void) {
  digitalWrite(IN_SENSOR_PIN, LOW);
  digitalWrite(OUT_SENSOR_PIN, LOW);
  delay(10);
  digitalWrite(IN_SENSOR_PIN, HIGH);
  digitalWrite(OUT_SENSOR_PIN, HIGH);
  delay(10);
  digitalWrite(IN_SENSOR_PIN, HIGH);
  digitalWrite(OUT_SENSOR_PIN, LOW);
  delay(10);
  if (!in_sensor.begin(IN_SENSOR_ADDRESS)) {
    Serial.println(F("ERROR WITH IN SENSOR..."));
  } else {
    Serial.println(F("IN SENSOR SUCCESSFULLY INITIALIZED..."));
  }
  delay(10);
  digitalWrite(OUT_SENSOR_PIN, HIGH);
  delay(10);
  if (!out_sensor.begin(OUT_SENSOR_ADDRESS)) {
    Serial.println(F("ERROR WITH OUT SENSOR..."));
  } else {
    Serial.println(F("OUT SENSOR SUCCESSFULLY INITIALIZED..."));
  }
  delay(10);
}

void process(void) {
  //Puerta
  if ((_values_out[0] < 200) && (_values_out[1] < 200) && (_values_out[2] < 200) && (_values_out[3] < 200) && (_values_out[4] < 200)) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  //Entrada
  if ((_values_out[3] < 1100) && (_values_in[3] > 1100) && (_values_in[0] < 1100)) {
    digitalWrite(LED_BUILTIN, LOW);
    people += 1;
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
  }
//  //Salida
  if ((_values_in[3] < 1100) && (_values_out[3] > 1100) && (_values_out[0] < 1100)) {
    digitalWrite(LED_BUILTIN, LOW);
    people -= 1;
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void show(void) {
  Serial.print(_values_in[0]);
  Serial.print(F(","));
//  Serial.print(_values_in[1]);
//  Serial.print(F(" "));
//  Serial.print(_values_in[2]);
//  Serial.print(F(" "));
//  Serial.print(_values_in[3]);
//  Serial.print(F(" "));
//  Serial.print(_values_in[4]);
//  Serial.print(F(" ||| "));
  Serial.println(_values_out[0]);
//  Serial.print(_values_out[1]);
//  Serial.print(F(" "));
//  Serial.print(_values_out[2]);
//  Serial.print(F(" "));
//  Serial.print(_values_out[3]);
//  Serial.print(F(" "));
//  Serial.print(_values_out[4]);
//  Serial.print(F(" ||| "));
//  Serial.print(temperature.convert(1));
//  Serial.print(F(","));
//  Serial.println(humidity.convert(1));
}

void read(void) {
  static uint32_t _t = 10000;
  out_sensor.rangingTest(&out_measure, false);
  in_sensor.rangingTest(&in_measure, false);
  _values_out[4] = _values_out[3];
  _values_out[3] = _values_out[2];
  _values_out[2] = _values_out[1];
  _values_out[1] = _values_out[0];
  _values_out[0] = out_measure.RangeMilliMeter;
  _values_in[4] = _values_in[3];
  _values_in[3] = _values_in[2];
  _values_in[2] = _values_in[1];
  _values_in[1] = _values_in[0];
  _values_in[0] = in_measure.RangeMilliMeter;
  if (millis() - _t > 10000) {
    temperature.clear();
    for (uint8_t i = 0; i < 5; i++) {
      temperature.force(temperature.get() + (humidity_sensor.readTemperature() / 5.0));
      delay(10);
    }
    temperature.set(temperature.get());
    humidity.clear();
    for (uint8_t i = 0; i < 5; i++) {
      humidity.force(humidity.get() + (humidity_sensor.readHumidity() / 5.0));
      delay(10);
    }
    humidity.set(humidity.get());
    _t = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  while (!Serial) {
    delay(1);
  }
  Serial.println();
  Serial.println(F("STARTING..."));
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN_SENSOR_PIN, OUTPUT);
  pinMode(OUT_SENSOR_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(IN_SENSOR_PIN, LOW);
  digitalWrite(OUT_SENSOR_PIN, LOW);
  Serial.println(F("OUTPUTS INITIALIZED..."));
  initialize();
  Serial.println(F("DISTANCE SENSORS INITIALIZED..."));
  temperature.begin(1.0, 0.0);
  humidity.begin(1.0, 0.0);
  humidity_sensor.initSHT20();
  humidity_sensor.checkSHT20();
  Serial.println(F("TEMPERATURE AND HUMIDITY SENSOR INITIALIZED..."));
  delay(100);
}

void loop() {
  read();
  process();
  show();
  //delay(100);
}
