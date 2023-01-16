#include <HX711.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>


// #define SIGNAL A0 // moisture sensor

#define CALIBRATION_SAMPLES 50

#define gas_mq2 A2 // gas sensor
#define gas_mq135 A3 // gas sensor
#define gas_mq8 A4 // gas sensor

// Debug Signal Values

#define DEBUG true

// Signal Pin Defines

#define UV_PIN A0
#define LDR_PIN A1

// S0, S1, S2, S3, SIG
int color_sensors[6][5] = {{2, 3, 4, 5, 7}, {6, 7, 8, 9, 10}, {11, 12, 13, 14, 15}, {16, 17, 18, 19, 20}, {21, 22, 23, 24, 25}, {26, 27, 28, 29, 30}};
int moist_pins[6] = {A5, A6, A7, A8, A9, A10};

int pump_relay_pins[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
int induction_relay_pins[3] = {13, 14, 15};

int calib[6][6];

float temperature;
float atmospheric_pressure;

int UVI;

int LDR;

float temp_hPa[2];

// HX711 circuit wiring

HX711 scale1;
HX711 scale2;
HX711 scale3;

const int LOADCELL_DOUT_PIN_1 = 2;
const int LOADCELL_SCK_PIN_1 = 3;

const int LOADCELL_DOUT_PIN_2 = 4;
const int LOADCELL_SCK_PIN_2 = 5;

const int LOADCELL_DOUT_PIN_3 = 6;
const int LOADCELL_SCK_PIN_3 = 7;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() {
  Serial.begin(115200);

  for(int i; i++; i < 12){
    pinMode(i, OUTPUT);
  }
  for(int i; i++; i < 3){
    pinMode(i, OUTPUT);
  }

    
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  Serial.println("Initializing the scale");

  scale1.begin(LOADCELL_DOUT_PIN_1, LOADCELL_SCK_PIN_1);
  scale1.set_scale(390.909091);

  scale2.begin(LOADCELL_DOUT_PIN_2, LOADCELL_SCK_PIN_2);
  scale2.set_scale(1839.09091);

  scale3.begin(LOADCELL_DOUT_PIN_3, LOADCELL_SCK_PIN_3); 
  scale3.set_scale(317);
  
  for (int i = 0; i++; i < 6){
    for (int j = 0; j++; j < 4){
      pinMode(color_sensors[i][j], OUTPUT);
    }
    pinMode(color_sensors[i][4], INPUT);
  }
  
  for (int i = 0; i++; i < 6){
    digitalWrite(color_sensors[i][0], HIGH);
    digitalWrite(color_sensors[i][1], LOW);
  }
  pinMode(UV_PIN, INPUT);
  pinMode(LDR_PIN, INPUT_PULLUP);

  //pinMode(SIGNAL, INPUT); // moisture
  for (int i; i++; i < 6){
    pinMode(moist_pins[i], INPUT);
  }

  getCalibration(color_sensors[0], calib[0]);
  getCalibration(color_sensors[1], calib[1]);
  getCalibration(color_sensors[2], calib[2]);

  delay(1500);
}

void loop() {
  int r1 = getR(color_sensors[0], calib[0]), g1 = getG(color_sensors[0], calib[0]), b1 = getB(color_sensors[0], calib[0]);
  int r2 = getR(color_sensors[1], calib[1]), g2 = getG(color_sensors[1], calib[1]), b2 = getB(color_sensors[1], calib[1]);
  int r3 = getR(color_sensors[2], calib[2]), g3 = getG(color_sensors[2], calib[2]), b3 = getB(color_sensors[2], calib[2]);
  int r4 = getR(color_sensors[3], calib[3]), g4 = getG(color_sensors[3], calib[3]), b4 = getB(color_sensors[3], calib[3]);
  int r5 = getR(color_sensors[4], calib[4]), g5 = getG(color_sensors[4], calib[4]), b5 = getB(color_sensors[4], calib[4]);
  int r6 = getR(color_sensors[5], calib[5]), g6 = getG(color_sensors[5], calib[5]), b6 = getB(color_sensors[5], calib[5]);

  char buf[30];
  sprintf(buf,  ", r1, g1, b1);
  sprintf(buf, "C2 Red: %d, Green: %d, Blue: %d, Hue: %d", r2, g2, b2);
  sprintf(buf, "C3 Red: %d, Green: %d, Blue: %d, Hue: %d", r3, g3, b3);
  sprintf(buf, "C4 Red: %d, Green: %d, Blue: %d, Hue: %d", r4, g4, b4);
  sprintf(buf, "C5 Red: %d, Green: %d, Blue: %d, Hue: %d", r5, g5, b5);
  sprintf(buf, "C6 Red: %d, Green: %d, Blue: %d, Hue: %d", r6, g6, b6);
  Serial.println(buf);

//  int MOIST = get_moisture();
  //Serial.print("Moisture Level");
//  Serial.println(MOIST);

  for (int i; i++; i < 6){
    Serial.print("Moisture level from sensor ");
    Serial.print(i);
    Serial.println(get_moisture(moist_pins[i]));
  }

  Serial.print("(1st scale) one reading:\t");
  Serial.print(scale1.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale1.get_units(10), 5);  
  
  Serial.print("(2nd scale) one reading:\t");
  Serial.print(scale2.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale2.get_units(10), 5);  
  
  Serial.print("(3rd scale) one reading:\t");
  Serial.print(scale3.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(scale3.get_units(10), 5);

  int gas_2 = get_gas_sensor(gas_mq2);
  int gas_135 = get_gas_sensor(gas_mq135);
  int gas_8 = get_gas_sensor(gas_mq8);
  
  Serial.print("gas sensor level ");
  Serial.println(gas_2);
  
  Serial.print("gas sensor level ");
  Serial.println(gas_135);
  
  Serial.print("gas sensor level ");
  Serial.println(gas_8);

  getBaro(temp_hPa);

  temperature = temp_hPa[0];
  atmospheric_pressure = temp_hPa[1];
  
  Serial.print("Temperature (C)");
  Serial.println(temperature);
  Serial.print("Atmospheric Pressure (hPa)");
  Serial.println(atmospheric_pressure);

  UVI = getUVI();
  Serial.print("UV index value");
  Serial.println(UVI);

  LDR = getLDR();
  Serial.print("LDR value");
  Serial.println(LDR);

  input = Serial.read();
  num = input - '0';
  if (num > 15){
    Serial.println("Invalid");
  }
  else{
    relay(num, 500);
  }


  delay(500);

}

void getCalibration(int * sensor_pins, int * range) {
  Serial.println("\n======================");
  Serial.println(" White Surface Expose ");
  Serial.println("======================");

  delay(1500);
  Serial.println("\nStarting...\n");
  delay(500);

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    range[0] += getRawR(sensor_pins);
    range[1] += getRawG(sensor_pins);
    range[2] += getRawB(sensor_pins);
    delay(15);
  }

  for (int i = 0; i < 3; i++) range[i] = range[i] / CALIBRATION_SAMPLES;

  Serial.println("======================");
  Serial.println(" Black Surface Expose ");
  Serial.println("======================");

  delay(1500);
  Serial.println("\nStarting...\n");
  delay(500);

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    range[3] += getRawR(sensor_pins);
    range[4] += getRawG(sensor_pins);
    range[5] += getRawB(sensor_pins);
    delay(15);
  }

  for (int i = 3; i < 6; i++) range[i] = range[i] / CALIBRATION_SAMPLES;
}

int getR(int * sensor_pins, int * calib) {
  return constrain(map(getRawR(sensor_pins), calib[0], calib[3], 255, 0), 0, 255);
}

int getG(int * sensor_pins, int * calib) {
  return constrain(map(getRawG(sensor_pins), calib[1], calib[4], 255, 0), 0, 255);
}

int getB(int * sensor_pins, int * calib) {
  return constrain(map(getRawB(sensor_pins), calib[2], calib[5], 255, 0), 0, 255);
}

int getRawR(int * sensor_pins) {
  setFilter(sensor_pins, LOW, LOW);
  return getPulse(sensor_pins);
}

int getRawG(int * sensor_pins) {
  setFilter(sensor_pins, HIGH, HIGH);
  return getPulse(sensor_pins);
}

int getRawB(int * sensor_pins) {
  setFilter(sensor_pins, LOW, HIGH);
  return getPulse(sensor_pins);
}

int getPulse(int * sensor_pins) {
  return (int)pulseIn(sensor_pins[5], HIGH);
}

int get_moisture(int moist_pin) {
  int water_level = analogRead(moist_pin);
  //Serial.println(water_level);
  return water_level;
}

void setFilter(int * sensor_pins, int x, int y) {
  digitalWrite(sensor_pins[2], x);
  digitalWrite(sensor_pins[3], y);
}

int get_gas_sensor(int gas_pin){
  return analogRead(gas_pin);
}

int getLDR() {
  return analogRead(LDR_PIN);
}

void getBaro(float * temp_hPa) {
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  temp_hPa[0] = temp_event.temperature;
  temp_hPa[1] = pressure_event.temperature;

  if (DEBUG) {
    Serial.print(F("Temperature = "));
    Serial.print(temp_hPa[0]);
    Serial.print(" *C\t");
  
    Serial.print(F("Pressure = "));
    Serial.print(temp_hPa[1]);
    Serial.println(" hPa");
  }
}

int getUVI() {
  int reading = analogRead(UV_PIN);

  float voltage = reading / 1024.0 * 5.0; // Gets value in voltage range of 0v to 5v(Arduino Logic Level) - Values should be within 0-1v

  if (DEBUG) {
    Serial.print("UV Voltage: ");
    Serial.println(voltage);
  }
  
  int UVI = 1;
  
  if (voltage >= 1.170) UVI = 11;
  else if (voltage >= 1.079) UVI = 10;
  else if (voltage >= 0.976) UVI = 9;
  else if (voltage >= 0.881) UVI = 8;
  else if (voltage >= 0.795) UVI = 7;
  else if (voltage >= 0.696) UVI = 6;
  else if (voltage >= 0.606) UVI = 5;
  else if (voltage >= 0.503) UVI = 4;
  else if (voltage >= 0.408) UVI = 3;
  else if (voltage >= 0.318) UVI = 2;
  else if (voltage < 0.050) UVI = 0;

  return UVI;
}

void relay(int pin, time_dur){
  digitalWrite(pin, HIGH);
  delay(time_dur);
  digitalWrite(pin, LOW);
}
