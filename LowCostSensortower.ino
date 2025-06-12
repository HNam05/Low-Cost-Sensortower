#include <stdio.h>
#include <Wire.h>
#include <Arduino.h>
#include "Adafruit_SHT4x.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <esp_sleep.h>
#include "ADS1115.h"
#include <sps30.h>

#include <Adafruit_ADS1X15.h>
#include <Time.h>

#include <SD.h>
#include <ArduinoJson.h>


//SPS 30
//#define SPS30_SDA 1
//#define SPS30_SCL 2

// Für den SHT
//#define SHT_SDA 21
//#define SHT_SCL 9


//Für SPS und SHT und ADC
#define I2C_SDA 21
#define I2C_SCL 9
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */


//Für den Luftdrucksensor BMP
#define BMP_SCK 35
#define BMP_SDI 36
#define BMP_SDO 37
#define BMP_CS 38

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

//Für SD_Karte
#define SD_CS_PIN 17    // Chip Select Pin
#define SD_SCK_PIN 3    // Serial Clock
#define SD_MISO_PIN 8   // Master In Slave Out
#define SD_MOSI_PIN 18  // Master Out Slave In




// Für den SHT
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
sensors_event_t humidity, temp;
uint32_t timestamp = millis();

//Deepsleep
#define WAKE_TIME 20000000   // 20 Sekunden
#define SLEEP_TIME 40000000  // 40 Sekunden


TwoWire shtWire = TwoWire(0);  // I2C Bus 0 für SHT40
TwoWire spsWire = TwoWire(1);  // I2C Bus 1 für SPS30



struct sps30_measurement m;
char serial[SPS30_MAX_SERIAL_LEN];
uint16_t data_ready;
int16_t ret;


void setup() {
  //Sync Time
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("wake up from deep-sleep");
  } else {
    Serial.println("reset");
    syncTime();  //resync the time after reset
  }







  //ADC
  Serial.begin(19200);//
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }



  //SD Karte
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  // SD-Karte initialisieren
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD-Karte konnte nicht initialisiert werden!");
    return;
  }
  Serial.println("SD-Karte erfolgreich initialisiert!");





  //shtWire.begin(SHT_SDA, SHT_SCL);
  //spsWire.begin(SPS30_SDA, SPS30_SCL);

  Wire.begin(I2C_SDA, I2C_SCL);

  //Wire.begin(SHT_SDA, SHT_SCL);
  //Für SHT 45
  Serial.begin(19200);
  delay(1000);

  Serial.println("Adafruit SHT4x test");
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);


  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
    case SHT4X_HIGH_PRECISION:
      Serial.println("High precision");
      break;
    case SHT4X_MED_PRECISION:
      Serial.println("Med precision");
      break;
    case SHT4X_LOW_PRECISION:
      Serial.println("Low precision");
      break;

      // Für den SHT 45
      sht4.setHeater(SHT4X_NO_HEATER);
      switch (sht4.getHeater()) {
        case SHT4X_NO_HEATER:
          Serial.println("No heater");
          break;
        case SHT4X_HIGH_HEATER_1S:
          Serial.println("High heat for 1 second");
          break;
        case SHT4X_HIGH_HEATER_100MS:
          Serial.println("High heat for 0.1 second");
          break;
        case SHT4X_MED_HEATER_1S:
          Serial.println("Medium heat for 1 second");
          break;
        case SHT4X_MED_HEATER_100MS:
          Serial.println("Medium heat for 0.1 second");
          break;
        case SHT4X_LOW_HEATER_1S:
          Serial.println("Low heat for 1 second");
          break;
        case SHT4X_LOW_HEATER_100MS:
          Serial.println("Low heat for 0.1 second");
          break;
      }
  }

  //Für BMP 390

  while (!Serial)
    ;
  Serial.println("BMP390 test");

  if (!bmp.begin_I2C()) {                                       // hardware I2C mode, can pass in address & alt Wire
    if (!bmp.begin_SPI(BMP_CS)) {                               // hardware SPI mode
      if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_SDO, BMP_SDI)) {  // software SPI mode
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        while (1)
          ;
      }
    }
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


  //SPS 30
  //Wire.begin(SPS30_SDA, SPS30_SCL);
  int16_t ret;
  uint8_t auto_clean_days = 4;
  uint32_t auto_clean;


  delay(2000);

  sensirion_i2c_init();

  while (sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
    delay(500);
  }

#ifndef PLOTTER_FORMAT
  Serial.print("SPS sensor probing successful\n");
#endif /* PLOTTER_FORMAT */

  ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
  if (ret) {
    Serial.print("error setting the auto-clean interval: ");
    Serial.println(ret);
  }

  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("error starting measurement\n");
  }

#ifndef PLOTTER_FORMAT
  Serial.print("measurements started\n");
#endif /* PLOTTER_FORMAT */


  delay(1000);




  //Deepsleep

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // **Hier Messung jede Sekunde während der Wachphase**:
  uint32_t startTime = millis();                     // Startzeit der Wachphase
  while (millis() - startTime < WAKE_TIME / 1000) {  // 10 Sekunden Wachphase




    //ADC
    int16_t adc0, adc1, adc2, adc3;
    float volts0, volts1, volts2, volts3;

    adc0 = ads.readADC_SingleEnded(0);  //Spannungsteiler
    adc1 = ads.readADC_SingleEnded(1);  //OP1
    adc2 = ads.readADC_SingleEnded(2);  //OP2
    //adc3 = ads.readADC_SingleEnded(3);

    volts0 = ads.computeVolts(adc0);
    volts1 = ads.computeVolts(adc1);
    volts2 = ads.computeVolts(adc2);
    //volts3 = ads.computeVolts(adc3);

    Serial.println("-----------------------------------------------------------");
    Serial.print("AIN0: ");
    Serial.print(adc0);
    Serial.print("  ");
    Serial.print(volts0);
    Serial.println("V");
    Serial.print("AIN1: ");
    Serial.print(adc1);
    Serial.print("  ");
    Serial.print(volts1);
    Serial.println("V");
    Serial.print("AIN2: ");
    Serial.print(adc2);
    Serial.print("  ");
    Serial.print(volts2);
    Serial.println("V");
    //Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");

    delay(1000);

    // Für den SHT
    sht4.getEvent(&humidity, &temp);
    timestamp = millis() - timestamp;

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degrees C");
    Serial.print("Humidity: ");
    Serial.print(humidity.relative_humidity);
    Serial.println("% rH");

    Serial.print("Read duration (ms): ");
    Serial.println(timestamp);

    delay(1000);

    //Für BMP 390

    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println();
    delay(1000);





    //SPS 30
    struct sps30_measurement m;
    char serial[SPS30_MAX_SERIAL_LEN];
    uint16_t data_ready;
    int16_t ret;

    do {
      ret = sps30_read_data_ready(&data_ready);
      if (ret < 0) {
        Serial.print("error reading data-ready flag: ");
        Serial.println(ret);
      } else if (!data_ready)
        Serial.print("data not ready, no new measurement available\n");
      else
        break;
      delay(100); /* retry in 100ms */
    } while (1);

    ret = sps30_read_measurement(&m);
    if (ret < 0) {
      Serial.print("error reading measurement\n");
    } else {

#ifndef PLOTTER_FORMAT
      Serial.print("PM  1.0: ");
      Serial.println(m.mc_1p0);
      Serial.print("PM  2.5: ");
      Serial.println(m.mc_2p5);
      Serial.print("PM  4.0: ");
      Serial.println(m.mc_4p0);
      Serial.print("PM 10.0: ");
      Serial.println(m.mc_10p0);

#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
      Serial.print("NC  0.5: ");
      Serial.println(m.nc_0p5);
      Serial.print("NC  1.0: ");
      Serial.println(m.nc_1p0);
      Serial.print("NC  2.5: ");
      Serial.println(m.nc_2p5);
      Serial.print("NC  4.0: ");
      Serial.println(m.nc_4p0);
      Serial.print("NC 10.0: ");
      Serial.println(m.nc_10p0);

      Serial.print("Typical particle size: ");
      Serial.println(m.typical_particle_size);
#endif

      Serial.println();

#else
      // since all values include particles smaller than X, if we want to create buckets we
      // need to subtract the smaller particle count.
      // This will create buckets (all values in micro meters):
      // - particles        <= 0,5
      // - particles > 0.5, <= 1
      // - particles > 1,   <= 2.5
      // - particles > 2.5, <= 4
      // - particles > 4,   <= 10

      Serial.print(m.nc_0p5);
      Serial.print(" ");
      Serial.print(m.nc_1p0 - m.nc_0p5);
      Serial.print(" ");
      Serial.print(m.nc_2p5 - m.nc_1p0);
      Serial.print(" ");
      Serial.print(m.nc_4p0 - m.nc_2p5);
      Serial.print(" ");
      Serial.print(m.nc_10p0 - m.nc_4p0);
      Serial.println();


#endif /* PLOTTER_FORMAT */
    }

    delay(1000);





    //SD Karte
    // JSON-Daten erstellen
    StaticJsonDocument<256> jsonData;
    Serial.println("unixtime:");
    Serial.println(String(getUnixTime()));
    // Werte in SD speichern
    jsonData["Timestamp"] = String(getUnixTime());
    jsonData["Temperature"] = temp.temperature;
    jsonData["Humidity"] = humidity.relative_humidity;
    jsonData["Altitude in m"] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    jsonData["Pressure in hPa"] = bmp.pressure / 100.0;
    jsonData["Typical Particle Size"] = m.typical_particle_size;
    jsonData["Spannungsteiler"] = volts0;
    jsonData["Channel 1"] = volts1;
    jsonData["Channel 2"] = volts2;
    

    // JSON-Daten auf SD-Karte speichern
    writeDataToSD(jsonData);
  }










  // Deep Sleep Phase
  Serial.println("ESP geht in den Deep Sleep-Modus.");
  digitalWrite(LED_BUILTIN, LOW);             // LED ausschalten (Schlafphase)
  esp_sleep_enable_timer_wakeup(SLEEP_TIME);  // Timer für den Deep Sleep
  esp_deep_sleep_start();
}






// Funktion: JSON-Daten auf SD-Karte speichern
void writeDataToSD(StaticJsonDocument<256>& jsonData) {
  File file = SD.open("/sensor_data.json", FILE_APPEND);
  if (file) {
    serializeJson(jsonData, file);  // JSON in Datei schreiben
    file.println();                 // Neue Zeile hinzufügen
    file.close();
    Serial.println("Daten gespeichert:");
    serializeJsonPretty(jsonData, Serial);  // Für Debugging
    Serial.println();
  } else {
    Serial.println("Datei konnte nicht geöffnet werden!");
  }
}



// Funktion: SD-Karte formatieren
void formatSDCard() {
  if (SD.exists("/")) {
    Serial.println("SD-Karte wird formatiert...");
    SD.remove("/sensor_data.json");
    delay(1000);
    Serial.println("Formatierung abgeschlossen.");
  }
}




// returns unix time
unsigned long long getUnixTime() {
  unsigned long long now = time(nullptr);
  return now;
}


// returns internal RTC time in "%Y-%m-%d %H:%M:%S" format
String getFormattedTime() {
  time_t now;
  struct tm timeinfo;
  time(&now);
  if (!localtime_r(&now, &timeinfo)) {
    return "Zeit konnte nicht abgerufen werden!";
  }
  char timeString[30];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeString);
}

// synchronizes RTCs
String syncTime() {


  int year = 2025;
  int month = 4;
  int day = 24;
  int hour = 11;
  int minute = 40;
  int second = 0;
  int offset = 1;
  struct tm timeInfo;
  timeInfo.tm_year = year - 1900;
  timeInfo.tm_mon = month - 1;
  timeInfo.tm_mday = day;
  timeInfo.tm_hour = hour - offset - 1;
  timeInfo.tm_min = minute;
  timeInfo.tm_sec = second;

  // update internal RTC
  struct timeval now = { mktime(&timeInfo), 0 };
  settimeofday(&now, NULL);  // Setzt die Systemzeit (RTC) auf die aktuelle Zeit



  return "";
}
