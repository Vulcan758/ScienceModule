// Debug Signal Values

#define DEBUG true

// Signal Pin Defines

#define UV_PIN A0
#define LDR_PIN A1

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() {
  Serial.begin(9600);

  // ========================= Pin Modes =========================
  
  pinMode(UV_PIN, INPUT);
  pinMode(LDR_PIN, INPUT_PULLUP);

  // ========================= BMP 280 Stuff =========================
  
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
}

float temp_hPa[2];

void loop() {
  getBaro(temp_hPa);
  delay(1000);
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
