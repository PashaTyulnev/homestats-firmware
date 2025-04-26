#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <DHT.h>
#include <MHZ19.h>
#include <SoftwareSerial.h>

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

// Reset Button und LED
#define RESET_BUTTON_PIN 0 // Beispiel: GPIO0
#define LED_PIN 2 // Beispiel: GPIO2 (eingebaute LED auf vielen Boards)

WiFiManager wifiManager;
bool portalActive = false;
unsigned long lastBlinkTime = 0;
bool ledState = false;

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

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    Serial.println("Reset-Taste gedrückt beim Start - WLAN-Daten löschen!");
    wifiManager.resetSettings();
    delay(1000);
  }

  // Jetzt versuchen, automatisch zu verbinden:
  if (!wifiManager.autoConnect("HomeStats")) {
    Serial.println("Verbindung fehlgeschlagen! Starte neu...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("WLAN verbunden!");
  Serial.print("IP-Adresse: ");
  Serial.println(WiFi.localIP());

  portalActive = false;
  initSensors();
}


void handleButtonAndPortal() {
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    Serial.println("Reset-Taste gedrückt - WLAN neu konfigurieren!");
    wifiManager.resetSettings();
    delay(500);
    portalActive = true;
    wifiManager.startConfigPortal("HomeStats");
    ESP.restart();
  }
}

void handleBlinking() {
  if (portalActive) {
    if (millis() - lastBlinkTime > 500) {
      lastBlinkTime = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else {
    digitalWrite(LED_PIN, HIGH); // LED aus
  }
}

void loop() {
  handleButtonAndPortal();
  handleBlinking();

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
