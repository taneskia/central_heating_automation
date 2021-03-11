#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//DEVICE VARIABLES
#define OTAHOSTNAME "esp8226_name"
#define OTAPASS ""
#define SUBTOPIC "rooms/room_name/state"
#define PUBTOPIC "rooms/room_name/temp"
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

void loop() {
  ArduinoOTA.handle();

  bool connection_ok = true;
  
  if (WiFi.status() != WL_CONNECTED){
    Serial.println("WIFI DISCONNECTED!");
    connection_ok = connection_ok && wifi_reconnect();    
  }
  
  if (!client.connected()) {
    Serial.println("MQTT DISCONNECTED!");
    connection_ok = connection_ok && mqtt_reconnect();    
  }    

  if(!connection_ok){
    //digitalWrite(RELAYPIN, HIGH); //turn off the RELAY
    pinMode(RELAYPIN, INPUT);
    total_reconnect_fails--;
      if(total_reconnect_fails == 0)
        ESP.restart();    
  }
  else{
    total_reconnect_fails = 5;
  }
  
  client.loop();
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sensors.requestTemperatures();
    t = sensors.getTempC(deviceAddress);
    Serial.println(t);
    client.publish(PUBTOPIC, String(t).c_str()); //Topic name
  }
}

void OTA_Setup(){
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

void callback(char* topic, byte* payload, unsigned int length) {
   
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  if (String(topic) == SUBTOPIC) {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      //digitalWrite(RELAYPIN, LOW);
      pinMode(RELAYPIN, OUTPUT);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      //digitalWrite(RELAYPIN, HIGH);
      pinMode(RELAYPIN, INPUT);
    }
  } 
}

bool mqtt_reconnect() {
  int retries = 5;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(OTAHOSTNAME, willTopic, willQoS, willRetain, willMessage)) {
      Serial.println("connected");
      client.subscribe(SUBTOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");      
      delay(10000);  // Wait 10 seconds before retrying
    }

    retries--;
    if(retries == 0)
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
        if(i == 0)
          return false;
    }  
    Serial.println("Connected!");
    return true;
} 

void dallasTempSensorSetup(){
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

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
