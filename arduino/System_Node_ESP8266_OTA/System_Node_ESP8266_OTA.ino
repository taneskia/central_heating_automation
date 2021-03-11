#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

//DEVICE VARIABLES
#define OTAHOSTNAME "esp8226_name"
#define OTAPASS ""
#define SUBTOPIC_SISTEM "rooms/room_name/state"
#define SUBTOPIC_PUMP "rooms/room_name/pump/state"
#define PUBTOPIC "rooms/room_name/current_state"
#define POLLTOPIC "rooms/room_name/poll"
byte willQoS = 0;
const char* willTopic = "rooms/room_name/connection";
const char* willMessage = "disconnected";
boolean willRetain = true;
//END

#ifndef STASSID
#define STASSID ""
#define STAPSK  ""
#endif

#define ONE_WIRE_BUS 5
#define RELAYPIN 4
#define IRPIN 14

const char* ssid = STASSID;
const char* password = STAPSK;

const char* mqttServer = "192.168.1.100";
const int mqttPort = 1883;
//const char* mqttUser = "otfxknod";
//const char* mqttPassword = "nSuUc1dDLygF";

WiFiClient espClient;

PubSubClient client(espClient);

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

DeviceAddress deviceAddress;

IRsend irsend(IRPIN);

float t = 0;

unsigned long previousMillis = 0;

const long interval = 2000;

void OTA_Setup();
void callback(char* topic, byte* payload, unsigned int length);
bool mqtt_reconnect();
bool wifi_reconnect();

void setup() {
  OTA_Setup();
  dallasTempSensorSetup();
  ir_setup();

  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, HIGH);

  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  mqtt_reconnect();
}

int total_reconnect_fails = 5;

bool sistem_state = false;
bool pump_state = false;

void loop() {
  ArduinoOTA.handle();

  bool connection_ok = true;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI DISCONNECTED!");
    connection_ok = connection_ok && wifi_reconnect();
  }

  if (!client.connected()) {
    Serial.println("MQTT DISCONNECTED!");
    connection_ok = connection_ok && mqtt_reconnect();
  }

  if (!connection_ok) {
    //================== !!WARNING!! IF CONNECTION IS LOST TURN ON THE PUMP !!WARNING!!==================
    //maybe fix it with a temp sensor that turns on the pump if below zero and off if not
    digitalWrite(RELAYPIN, LOW);
    total_reconnect_fails--;
    if (total_reconnect_fails == 0)
      ESP.restart();
  }
  else {
    total_reconnect_fails = 5;
  }

  client.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

  }
}

void publishState() {
  String message = String("System state: ") + String(sistem_state ? "on" : "off") + String(" Pump state: ") +  String(pump_state ? "on" : "off");
  client.publish(PUBTOPIC, message.c_str());
}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String message;

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message += (char)payload[i];
  }
  Serial.println();

  if (String(topic) == SUBTOPIC_PUMP) {
    Serial.print("Changing output to ");
    if (message == "on") {
      Serial.println("on");
      digitalWrite(RELAYPIN, LOW);
      pump_state = true;
    }
    else if (message == "off") {
      Serial.println("off");
      digitalWrite(RELAYPIN, HIGH);
      pump_state = false;
    }
  } else if (String(topic) == SUBTOPIC_SISTEM) {
    Serial.print("Changing output to ");
    if (message == "on") {
      Serial.println("on");
      ir_send("on");
      sistem_state = true;
    }
    else if (message == "off") {
      Serial.println("off");
      ir_send("off");
      sistem_state = false;
    }
  } else if (String(topic) == POLLTOPIC) {
    publishState();
  }
}

void ir_send(String state) {
  if (state == "on") {
    uint16_t rawData[73] = {9074, 4430,  680, 512,  680, 486,  704, 1624,  678, 1624,  706,
                            512,  680, 484,  706, 486,  678, 514,  678, 512,  678, 514,  676,
                            538,  652, 1626,  702, 512,  678, 488,  702, 512,  678, 488,  704,
                            512,  680, 488,  702, 512,  678, 488,  704, 486,  678, 538,  652,
                            514,  676, 514,  676, 514,  678, 512,  678, 512,  678, 512,  678,
                            1624,  704, 488,  702, 1624,  680, 486,  704, 486,  704, 1598,
                            704, 512,  678
                           };
    irsend.sendRaw(rawData, 73, 38);
  }
  else if (state == "off") {
    uint16_t rawData[73] = {9048, 4406,  702, 512,  678, 512,  678, 1650,  652, 512,  678,
                            512,  680, 512,  678, 512,  678, 538,  652, 488,  702, 512,  680,
                            486,  704, 1650,  680, 486,  704, 512,  678, 486,  678, 538,  652,
                            512,  678, 514,  678, 538,  652, 514,  678, 512,  678, 538,  652,
                            512,  680, 512,  678, 486,  704, 512,  680, 512,  678, 486,  706,
                            1652,  652, 512,  678, 1650,  680, 512,  678, 512,  680, 1624,
                            678, 512,  678
                           };
    irsend.sendRaw(rawData, 73, 38);
  }
}

bool mqtt_reconnect() {
  int retries = 5;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", willTopic, willQoS, willRetain, willMessage)) {
      Serial.println("connected");
      client.subscribe(SUBTOPIC_SISTEM);
      client.subscribe(SUBTOPIC_PUMP);
      client.subscribe(POLLTOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(10000);  // Wait 10 seconds before retrying
    }

    retries--;
    if (retries == 0)
      return false;
  }
  client.publish(willTopic, "connected", true);
  return true;
}

bool wifi_reconnect() {
  int i = 60; // wait 30s while trying to reconect (500ms delay)
  Serial.print("Reconnecting...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    i--;
    if (i == 0)
      return false;
  }
  Serial.println("Connected!");
  return true;
}

void dallasTempSensorSetup() {
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  if (!sensors.getAddress(deviceAddress, 0)) Serial.println("Unable to find address for Device 0");

  Serial.print("Device 0 Address: ");
  printAddress(deviceAddress);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(deviceAddress, 11);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress), DEC);
  Serial.println();
}

void ir_setup() {
  irsend.begin();
#if ESP8266
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
#else  // ESP8266
  Serial.begin(115200, SERIAL_8N1);
#endif  // ESP8266
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void OTA_Setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OTAHOSTNAME);

  // No authentication by default
  ArduinoOTA.setPassword(OTAPASS);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
