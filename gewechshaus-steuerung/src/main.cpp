/*****************************************************
 * Date: 15.06.2020
 * Written by: Thomas Huesing
 * ***************************************************/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

//WIFI / MQTT
#define WIFI_SSID "DEINE_SSID"
#define WIFI_PASSWORD "DEIN_PASSWORT"
#define MQTT_HOST IPAddress(192, 168, 123, 240)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

//BME
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
float temperature, humidity, pressure, altitude;
unsigned long now = 0;
uint16_t updatetimer = 60 * 1000; //Upadte alle 60000 Millisekunden

//Motor
//#define STEPSPERREVOLUTION  200  // Schritte die der Motor für eine Umdrehung braucht
#define DIRPIN  12 // D6
#define STEPPIN 14 // D5
int stepstomake; 
int zu =0;

//Referenztaster
#define SW1 13 // D7



void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub=mqttClient.subscribe("gw/in/command", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload_in, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    uint8_t * payload = (uint8_t *) malloc(len + 1);
    memcpy(payload, payload_in, len);
    payload[len] = 0;
    Serial.printf("MQTT: Recieved [%s]: %s\r\n", topic, payload);
    if (strcmp(topic, "gw/in/command") == 0) {
     
      if (strcmp((char*)payload, "zu") == 0) {
        stepstomake = 50;
        Serial.println("Mache zu");
      }
        if (strcmp((char*)payload, "stop") == 0) {
        stepstomake = 0;
        Serial.println("STOP");
      }
        if (strcmp((char*)payload, "auf") == 0) {
        stepstomake = -19200;
        Serial.println("Mache auf");
      }
        if (strcmp((char*)payload, "service") == 0) {
        stepstomake = -12800;
        Serial.println("Service");
      }
    }  
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


void update(void) {
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.println("Sensor gelesen.");
    now = millis();
    char result[10];
    dtostrf(temperature, 8, 2, result);
    mqttClient.publish("gw/out/temp", 0, true, result);
    dtostrf(humidity, 8, 2, result);
    mqttClient.publish("gw/out/hum", 0, true, result);
    dtostrf(pressure, 8, 2, result);
    mqttClient.publish("gw/out/pres", 0, true, result);
}
 
void setup() {
  //BME
  bme.begin(0x76);

  pinMode(DIRPIN, OUTPUT);
  pinMode(STEPPIN, OUTPUT);
  Serial.begin(115200);
  delay(1);
  
 //Referenzfahrt    
  Serial.println("Referenzfahrt");      
  digitalWrite(DIRPIN, HIGH);   
  while ( digitalRead(SW1) != LOW ) {
        digitalWrite(STEPPIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEPPIN, LOW);
        delayMicroseconds(1000);
        yield();
  //delay(10);
  }
  
//MQTT  
  //Serial.begin(115200);
  Serial.println();
  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  update();

}



void loop() {

//BME LOOP alle 5 sec
  if (millis() > now + updatetimer) {
    update();
  }

//Motor auf und zu fahren
  if (stepstomake != 0) {
    if (stepstomake > 0) {
      digitalWrite(DIRPIN, HIGH);
      //stepstomake = stepstomake - 1
      if (digitalRead(SW1) != LOW) {
        digitalWrite(STEPPIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEPPIN, LOW);
        delayMicroseconds(1000);
      } else {
        stepstomake = 0;
        //zustand="geschlossen";
      }    
    }
    if (stepstomake < 0) {
      digitalWrite(DIRPIN, LOW);
      digitalWrite(STEPPIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(STEPPIN, LOW);
      delayMicroseconds(1000);
      stepstomake = stepstomake + 1;
      //zustand= "geoeffnet";
      if (stepstomake == 0) {
      }
    }
    
  }

//Debug
  //delay(10);
  //Serial.print("stepstomake: ");
  //Serial.println(stepstomake);
}