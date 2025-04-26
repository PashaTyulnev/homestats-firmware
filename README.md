# HomeStats mit ESP8266
Projekt von Pavel Tyulnev


## Wichtige Hinweise
### ESP8266 Upload Geschwindigkeit erhöhen

Arduino IDE ->
Suche den Punkt Upload Speed oder Upload-Geschwindigkeit.
Stelle ihn auf 921600 Baud (statt 115200 oder 57600).


### Flash-Modus optimieren

Ebenfalls unter Werkzeuge (Tools):

Flash Mode: Wähle DIO (Double I/O) oder manchmal QIO (Quad I/O)
DIO ist sicherer bei günstigen Modulen.


### Verbindung mit dem WLAN
ESP9288 Firmware hochladen.

1. ESP8266 öffnet Acces Point "HomeStats"
2. Dort WLAN Netz auswählen und Passwort eingeben.
3. Mit Reset Button kann man die Credentials zurücksetzen.


### TODO Pinout

### Sensoren
1. DHT22 Temperatur + Luftfeuchtigkeit
2. GP2Y10 Staubsensor
3. MH-Z19 co2 Sensor
4. GY-68 Luftdrucksensor





