#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <DHT.h>
#include <MHZ19.h>
#include <SoftwareSerial.h>

// WLAN-Daten
const char* ssid = "Jupa 2,4";
const char* password = "YouCrazyCat5454";

// Serveradresse
const char* serverURL = "home.it-horizon.de";

// DHT Sensor
#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Staubsensor
int messPin = A0;
int abtastZeit = 280;
int verzogerungZeit = 40;
int schlafZeit = 9680;

// CO2 Sensor (MH-Z19)
#define RX_PIN 12
#define TX_PIN 13
#define BAUDRATE 9600
MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);

void connectToWiFi(const char* ssid, const char* password) {
  Serial.print("Verbinde mit WLAN...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWLAN verbunden.");
  Serial.print("IP-Adresse: ");
  Serial.println(WiFi.localIP());
}

void initSensors() {
  dht.begin();
  mySerial.begin(BAUDRATE);
  myMHZ19.begin(mySerial);
  myMHZ19.autoCalibration();
}

float readDustVoltage() {
  delayMicroseconds(abtastZeit);
  float messwert = analogRead(messPin);
  delayMicroseconds(verzogerungZeit);
  delayMicroseconds(schlafZeit);
  return messwert * (3.3 / 1024.0);
}

float calculateDustDensity(float voltage) {
  return 170 * voltage - 0.1;
}

void sendSensorData(float humidity, float temperature, float dustVoltage, float dustDensity, int co2, int co2Temp) {
  WiFiClientSecure client;
  client.setInsecure();

  String url = String("/log_data?humidity=") + humidity +
               "&temperature=" + temperature +
               "&dustVoltage=" + dustVoltage +
               "&dustDensity=" + dustDensity +
               "&co2=" + co2 +
               "&co2temp=" + co2Temp;

  if (client.connect(serverURL, 443)) {
    Serial.println("Verbunden mit Server");

    client.print("GET " + url + " HTTP/1.1\r\n");
    client.print("Host: " + String(serverURL) + "\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");

    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
  } else {
    Serial.println("Verbindung fehlgeschlagen");
  }
}

void setup() {
  Serial.begin(9600);
  connectToWiFi(ssid, password);
  initSensors();
}

void loop() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float dustVoltage = readDustVoltage();
  float dustDensity = calculateDustDensity(dustVoltage);
  int co2 = myMHZ19.getCO2();
  int co2Temp = myMHZ19.getTemperature();

  Serial.println("--- Messwerte ---");
  Serial.print("Temperatur: "); Serial.println(temperature);
  Serial.print("Feuchtigkeit: "); Serial.println(humidity);
  Serial.print("Staubspannung: "); Serial.println(dustVoltage);
  Serial.print("Staubdichte: "); Serial.println(dustDensity);
  Serial.print("CO2: "); Serial.println(co2);
  Serial.print("CO2 Temp: "); Serial.println(co2Temp);

  sendSensorData(humidity, temperature, dustVoltage, dustDensity, co2, co2Temp);
  delay(10000);
}
