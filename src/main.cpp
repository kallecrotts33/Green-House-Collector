#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
#include <string>
#include <secrets.h>

// generate client ID based on MAC address for uniqueness
String clientId = String(WiFi.macAddress());

// Wifi credentials in include/secrets.h, see example file include/secrets.example.h
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;
const char *mqtt_server = MQTT_BROKER_IP;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
unsigned long lastReconnectAttempt = 0;

// Initialize AM2320 sensor
Adafruit_AM2320 am2320 = Adafruit_AM2320();

// LED pin
const int ledPinR = 16;
const int ledPinG = 2;
const int ledPinB = 4;
unsigned long blinkInterval = 0;
unsigned long lastBlinkTime = 0;
bool ledTurnedON;
int ledState = -1;

// Buzzer pin
const int buzzerPin = 26;
unsigned long lastBlipTime = 0;
bool blipActive = false;
const unsigned long blipLength = 20;     // length of blip in ms (super short)
const unsigned long blipInterval = 1000; // one blip every 1 second
int globalBuzzerState = 0;

// Motion sensor pin
const int motionSensorPin = 15;
bool motionSensorState;
bool lastMotionSensorState;

// Fan pin
const int fanPin = 23;
bool fanState = false;

// Nominal temperature
float nominalTemp = 25.0;

// Stop Mode
bool stopMode = false;

boolean reconnect()
{
  if (client.connect(clientId.c_str()))
  {
    Serial.println("Connected to MQTT broker");
    clientId.replace(":", "");
    // subscriptions
    String ledStateString = "esp32/" + String(clientId) + "/ledState";
    client.subscribe(ledStateString.c_str());

    String buzzerStateString = "esp32/" + String(clientId) + "/buzzerState";
    client.subscribe(buzzerStateString.c_str());

    // String stopModeString = "esp32/" + String(clientId) + "/STOP";
    String stopModeString = "esp32/STOP";
    client.subscribe(stopModeString.c_str());

    String resetString = "esp32/reset";
    client.subscribe(resetString.c_str());

    String nominalTempString = "esp32/" + String(clientId) + "/nominalTemp";
    client.subscribe(nominalTempString.c_str());
  }
  return client.connected();
}

void setColor(int redValue, int greenValue, int blueValue)
{
  ledcWrite(0, redValue);
  ledcWrite(1, greenValue);
  ledcWrite(2, blueValue);
}

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setLedState()
{
  if (stopMode)
  {
    setColor(0, 0, 255);
    return;
  }
  unsigned long now = millis();
  if (ledState == 0)
  { // Solid green
    setColor(0, 255, 0);
  }
  else if (ledState == 1)
  { // 2Hz yellow
    if (now - lastBlinkTime >= 1000)
    {
      lastBlinkTime = now;
      if (ledTurnedON)
      {
        setColor(0, 0, 0);
        ledTurnedON = false;
        lastBlinkTime = millis();
      }
      else
      {
        setColor(255, 255, 0);
        ledTurnedON = true;
        lastBlinkTime = millis();
      }
    }
  }
  else if (ledState == 2)
  { // Solid yellow
    setColor(255, 255, 0);
  }
  else if (ledState == 3)
  { // 4Hz red
    if (now - lastBlinkTime >= 500)
    {
      lastBlinkTime = now;
      if (ledTurnedON)
      {
        setColor(0, 0, 0);
        ledTurnedON = false;
        lastBlinkTime = millis();
      }
      else
      {
        setColor(255, 0, 0);
        ledTurnedON = true;
        lastBlinkTime = millis();
      }
    }
  }
  else if (ledState == 4)
  { // Solid red
    setColor(255, 0, 0);
  }
  else if (ledState == -1)
  { // Led off
    setColor(0, 0, 0);
  }
}

void setBuzzerState(int buzzerState)
{
  if (stopMode)
  {
    digitalWrite(buzzerPin, HIGH);
    return;
  }

  unsigned long now = millis();

  if (buzzerState == 1)
  {
    if (!blipActive && (now - lastBlipTime >= blipInterval))
    {
      blipActive = true;
      lastBlipTime = now;
      digitalWrite(buzzerPin, LOW); 
    }

    // End blip
    if (blipActive && (now - lastBlipTime >= blipLength))
    {
      blipActive = false;
      digitalWrite(buzzerPin, HIGH); 
    }
  }
  else
  {
    blipActive = false;
    digitalWrite(buzzerPin, HIGH);
  }
}

void setFanState(int fanState)
{
  if (stopMode)
  {
    digitalWrite(fanPin, LOW);
    return;
  }
  if (fanState == 1)
  {
    digitalWrite(fanPin, HIGH);
  }
  else if (fanState == 0)
  {
    digitalWrite(fanPin, LOW);
  }
}

void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  // LED Color based on temperature
  String ledStateString = "esp32/" + String(clientId) + "/ledState";
  if (String(topic) == ledStateString)
  {
    if (-1 <= messageTemp.toInt() && messageTemp.toInt() <= 4 && !stopMode)
    {
      ledState = (messageTemp.toInt());
      Serial.print("LED state changed to: ");
      Serial.println(messageTemp.toInt());
    }
  }


  String buzzerStateString = "esp32/" + String(clientId) + "/buzzerState";
  if (String(topic) == buzzerStateString)
  {
    if (0 <= messageTemp.toInt() && messageTemp.toInt() <= 1)
    {
      globalBuzzerState = messageTemp.toInt(); // save state
      Serial.print("Buzzer state changed to: ");
      Serial.println(messageTemp.toInt());
    }
  }

  // Stop mode
  // String stopModeString = "esp32/" + String(clientId) + "/STOP"; //use this line to have stop mode per device
  String stopModeString = "esp32/STOP";
  if (String(topic) == stopModeString)
  {
    if (messageTemp.toInt() == 1)
    {
      stopMode = true;
      Serial.print("STOP mode activated");
      setFanState(0);
      setLedState();
      setBuzzerState(0);
    }
    else if (messageTemp.toInt() == 0)
    {
      stopMode = false;
      Serial.print("STOP mode de-activated");
    }
  }

  // Reset command
  String resetString = "esp32/reset";
  if (String(topic) == resetString)
  {
    Serial.print("reset activated");
    setup_wifi();
    client.disconnect();
    client.connect(clientId.c_str());
    setup();
  }

  // Nominal temperature
  String nominalTempString = "esp32/" + String(clientId) + "/nominalTemp";
  if (String(topic) == nominalTempString)
  {
    nominalTemp = messageTemp.toFloat();
    Serial.print("Nominal temperature set to: ");
    Serial.println(nominalTemp);
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  am2320.begin(); // Initialize AM2320 sensor

  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);

  ledcAttachPin(ledPinR, 0);
  ledcAttachPin(ledPinG, 1);
  ledcAttachPin(ledPinB, 2);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);
  pinMode(motionSensorPin, INPUT);
  pinMode(fanPin, OUTPUT);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void sendData()
{
  if (!client.connected())
  {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000)
    {
      lastReconnectAttempt = now;
      if (reconnect())
        lastReconnectAttempt = 0;
    }
  }
  else
  {
    if (!stopMode)
    {
      long now = millis();
      if (now - lastMsg > 2000)
      {
        lastMsg = now;

        Serial.println(clientId);

        // Temp and humidity
        float temperature = am2320.readTemperature();
        float humidity = am2320.readHumidity();
        // Control fan based on temperature
        if (temperature > nominalTemp * 1.3)
        {
          if (!fanState)
          {
            setFanState(1);
            fanState = true;
          }
        }
        else if (temperature < nominalTemp * 0.9)
        {
          if (fanState)
          {
            setFanState(0);
            fanState = false;
          }
        }
        // Print values to Serial Monitor and send via MQTT
        char tempString[8];
        char humString[8];
        dtostrf(temperature, 1, 2, tempString);
        dtostrf(humidity, 1, 2, humString);
        Serial.print("Temperature: ");
        Serial.print(tempString);
        Serial.print(" °C  |  Humidity: ");
        Serial.print(humString);
        Serial.println(" %");
        // Publish values
        String temperatureTopic = "esp32/" + String(clientId) + "/temperature";
        client.publish(temperatureTopic.c_str(), tempString);
        String humidityTopic = "esp32/" + String(clientId) + "/humidity";
        client.publish(humidityTopic.c_str(), humString);

        // motion sensor
        String motionTopic = "esp32/" + String(clientId) + "/motion";
        motionSensorState = digitalRead(motionSensorPin);
        if (motionSensorState != lastMotionSensorState)
        {
          if (motionSensorState)
          {
            Serial.println("HUMAN PRESENT!!");
            client.publish(motionTopic.c_str(), "1");
          }
          else
          {
            Serial.println("NO HUMAN");
            client.publish(motionTopic.c_str(), "0");
          }
          lastMotionSensorState = motionSensorState;
        }
      }
    }
  }
}

void loop()
{
  client.loop();
  sendData();
  setLedState();
  setBuzzerState(globalBuzzerState);
  delay(10);
  yield();
}
