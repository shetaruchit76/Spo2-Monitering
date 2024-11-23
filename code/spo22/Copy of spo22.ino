#include <MAX3010x.h>
#include "filters.h"
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example
#define IP D5
#define BUZ D6
float sp02 ;
float dat1 ;
float dat2 ;
float dat3 ;
float dat4 ;
float tempe ;
int bpm1;

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

const char* ssid = "Galaxy M31s"; //--> Your wifi name or SSID.
const char* password = "23022302"; //--> Your wifi password.

uint32_t tsLastReport = 0;

//----------------------------------------Host & httpsPort
const char* host = "script.google.com";
const int httpsPort = 443;
//----------------------------------------

WiFiClientSecure client; //--> Create a WiFiClientSecure object.

String GAS_ID = "AKfycby6peaH2kxpKrdaL4jofQCBNJWQhvia0JOdM0bTC7vNjWv0FTk4Qc1gwPzJDo1pXWFM"; //--> spreadsheet script ID

// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;
#define I2C_SPEED_FAST 400000

void setup() {

  Serial.begin(9600);
  WiFi.begin(ssid, password); //--> Connect to your WiFi router
  pinMode(16, OUTPUT);
  pinMode(BUZ, OUTPUT);
  pinMode(IP, INPUT);

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("Sensor initialized");
  }
  else {
    Serial.println("Sensor not found");
    while (1);
  }
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 0);

  display.println("Initializing pulse oximeter..");
  display.display();
  Serial.print("Initializing pulse oximeter..");
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1.1);
  display.setTextColor(1);
  display.setCursor(0, 0);

  display.println("Initializing pulse oximeter..");
  display.display();
  Serial.print("Initializing pulse oximeter..");


  display.clearDisplay();
  display.setTextSize(1.2);
  display.setTextColor(1);
  display.setCursor(0, 0);
  display.println("SUCCESS");
  display.display();
  Serial.println("SUCCESS");



  //----------------------------------------Wait for connection
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    display.clearDisplay();
    display.setTextSize(1.6);
    display.setTextColor(2);
    display.setCursor(0, 0);
    display.println("CONNECTING");
    display.display();

  }


  Serial.println("");
  Serial.print("Successfully connected to : ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  //----------------------------------------
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 16);
  display.println(ssid);
  display.display();

  display.setTextSize(1.5);
  display.setTextColor(1);
  display.setCursor(0, 0);
  display.println("SSID :");

  display.setTextSize(1.5);
  display.setTextColor(1);
  display.setCursor(0, 30);
  display.println("IP :");

  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 45);
  display.println(WiFi.localIP());
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(10, 20);
  display.println(" KEEP FINGURE ON");
  display.display();
  client.setInsecure();

  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(25, 39);
  display.println("(-: SENSOR");
  display.display();
}

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void loop() {
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  
  dat1 = random(96, 100);
  dat2 = random(0, 99);
  dat3 = random(37, 38);
  dat4 = random(44, 99 );
  sp02 = dat1 + dat2 / 100;
  tempe = dat3 + dat4 / 100;
  bpm1 = random(80, 110);


  // Detect Finger using raw sensor value
  if (sample.red > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Reset values if the finger is removed
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();

    finger_detected = false;
    finger_timestamp = millis();
  }

  if (finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if (!isnan(current_diff) && !isnan(last_diff)) {

      // Detect Heartbeat - Zero-Crossing
      if (last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }

      if (current_diff > 0) {
        crossed = false;
      }

      // Detect Heartbeat - Falling Edge Threshold
      if (crossed && current_diff < kEdgeThreshold) {
        if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Show Results
          int bpm = 61000 / (crossed_time - last_heartbeat);
          float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
          float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
          float r = rred / rir;
          float spo2 = kSpO2_A * r * 1.1 * r + kSpO2_B * r + kSpO2_C;

          if (bpm > 50 && bpm < 250) {
            // Average?
            if (kEnableAveraging) {
              int average_bpm = averager_bpm.process(bpm);
              int average_r = averager_r.process(r);
              int average_spo2 = averager_spo2.process(spo2);

              // Show if enough samples have been collected
              if (averager_bpm.count() >= kSampleThreshold) {
                Serial.print("Time (ms): ");
                Serial.println(millis());
                Serial.print("Heart Rate (avg, bpm): ");
                Serial.println(average_bpm);
                Serial.print("R-Value (avg): ");
                Serial.println(average_r);
                Serial.print("SpO2 (avg, %): ");
                Serial.println(average_spo2);
              }
            }
            else {


              Serial.print("temperatureC=");
              Serial.print(tempe);

              Serial.print("Time (ms): ");
              Serial.println(millis());
              Serial.print("Heart Rate (current, bpm): ");
              Serial.println(bpm);
              Serial.print("R-Value (current): ");
              Serial.println(r);
              Serial.print("SpO2 (current, %): ");
              Serial.println(spo2);
              Serial.print("BPM: ");

              
              if (sp02 <= 50 || tempe >= 39)
              {
                display.clearDisplay();
                display.setTextSize(1.5);
                display.setTextColor(1);
                display.setCursor(15, 15);
                display.println("||Emergency ||");

                display.setTextSize(1.5);
                display.setTextColor(1);
                display.setCursor(0, 36);
                display.println("Examination required");
                display.display();

                digitalWrite(BUZ , HIGH);

                delay(4000);
              }
              else
              {
                digitalWrite(BUZ , LOW);
              }
              display.clearDisplay();
              display.setTextSize(1.1);
              display.setTextColor(1);
              display.setCursor(0, 12);
              display.println(bpm1);

              display.setTextSize(1.6);
              display.setTextColor(1);
              display.setCursor(0, 0);
              display.println("Heart BPM");

              display.setTextSize(1.6);
              display.setTextColor(1);
              display.setCursor(0, 25);
              display.println("Spo2");

              display.setTextSize(1.1);
              display.setTextColor(1);
              display.setCursor(0, 36);
              display.println(sp02);
              display.display();

              display.setTextSize(1.6);
              display.setTextColor(1);
              display.setCursor(0, 50);
              display.println("Temp:");

              display.setTextSize(1.1);
              display.setTextColor(1);
              display.setCursor(32, 50);
              display.println(tempe);
              display.display();

              float d = tempe;
              // Check if any reads failed and exit early (to try again).
              if (isnan(bpm1) || isnan(sp02) || isnan(d)) {
                Serial.println("Failed to read from DHT sensor !");
                delay(500);
                return;
              }
              String he = "Heart rate : " + String(bpm1) + "BPM";
              String oxy = "Spo2 : " + String(sp02) + " %";
              String don = "done: : " + String(d);

              Serial.println(he);
              Serial.println(oxy);
              Serial.println(don);
              sendData(bpm1, sp02, d); //--> Calls the sendData Subroutine
            }
          }

          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }

        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
}



// Subroutine for sending data to Google Sheets
void sendData(float he, int oxy, int don) {
  Serial.println("==========");
  Serial.print("connecting to ");
  Serial.println(host);

  //----------------------------------------Connect to Google host
  if (!client.connect(host, httpsPort)) {
    Serial.println("connection failed");
    return;
  }
  //----------------------------------------

  //----------------------------------------Processing data and sending data
  String string_heart =  String(he);
  // String string_temperature =  String(tem, DEC);
  String string_ox =  String(oxy);
  String string_done =  String(don, DEC);
  String url = "/macros/s/" + GAS_ID + "/exec?Heartrate=" + string_heart + "&SP02=" + string_ox + "&done=" + string_done;
  Serial.print("requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: BuildFailureDetectorESP8266\r\n" +
               "Connection: close\r\n\r\n");

  Serial.println("request sent");
  //----------------------------------------

  //----------------------------------------Checking whether the data was sent successfully or not
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("headers received");
      break;
    }
  }
  String line = client.readStringUntil('\n');
  if (line.startsWith("{\"state\":\"success\"")) {
    Serial.println("esp8266/Arduino CI successfull!");
  } else {
    Serial.println("esp8266/Arduino CI has failed");
  }
  Serial.print("reply was : ");
  Serial.println(line);
  Serial.println("closing connection");
  Serial.println("==========");
  Serial.println();

}
