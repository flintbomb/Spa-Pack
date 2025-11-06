#include <Arduino.h>
#include "main.h"
#include <AsyncTCP.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h> 
//#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <arduinoJson.h>
#include <WebSerial.h>
#include <FastLED.h>

extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}

//TODO
//Pump logic on button press
//send temperature and heater state to mqtt periodically
//format mqtt messages as json

//NVM Preferances 
Preferences preferences;

// mqtt declarations
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

//wifi connection variables
unsigned long previousMillis = 0;
AsyncWebServer server(80);

// Setup oneWire instances to communicate with temperature sensors
OneWire oneWire1(ONE_WIRE_BUS_WATER_PIN);
OneWire oneWire2(ONE_WIRE_BUS_HIGH_LIMIT_PIN);

// Pass our oneWire reference to the Dallas Temperature sensor
DallasTemperature waterSensor(&oneWire1);
DallasTemperature highLimitSensor(&oneWire2);

// FastLED setup for built-in LED WIFI status indication
CRGB leds[NUM_LEDS]; // Creates an array to hold the LED data

void setup() {
  Serial.begin(115200);

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, OFF);
  pinMode(PUMP1_LOW_PIN, OUTPUT);
  digitalWrite(PUMP1_LOW_PIN, OFF); 
  pinMode(PUMP1_HIGH_PIN, OUTPUT);
  digitalWrite(PUMP1_HIGH_PIN, OFF);
  pinMode(PUMP2_LOW_PIN, OUTPUT);
  digitalWrite(PUMP2_LOW_PIN, OFF);
  pinMode(PUMP2_HIGH_PIN, OUTPUT);
  digitalWrite(PUMP2_HIGH_PIN, OFF);
  pinMode(LIGHTS_PIN, OUTPUT);
  digitalWrite(LIGHTS_PIN, OFF);  
  pinMode(CIRCULATION_PUMP_PIN, OUTPUT);
  digitalWrite(CIRCULATION_PUMP_PIN, OFF); 
  pinMode(OZONE_PIN, OUTPUT);
  digitalWrite(OZONE_PIN, OFF);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);

  // Configure Spa Light
  ledcSetup(channel, freq, resolution);
  // Attach the pin to the channel
  ledcAttachPin(LIGHTS_PIN, channel);
   
  // get preferences
  outputPrintln(F("Reading NVM"));
  preferences.begin("states", false);
  TEMP_SETPOINT = preferences.getInt("setPoint", TEMP_SETPOINT);
  FILTER_START_HOUR = preferences.getInt("filterHour", FILTER_START_HOUR);
  FILTER_START_MINUTE = preferences.getInt("filterMinute", FILTER_START_MINUTE);
  FILTER_DURATION = preferences.getInt("filterDuration", FILTER_DURATION);
  outputPrint(F("Stored Setpoint: "));
  outputPrintln(String(TEMP_SETPOINT));
  outputPrint(F("Stored Filter Time: "));
  outputPrint(String(FILTER_START_HOUR));
  outputPrint(F(":"));
  outputPrintln(String(FILTER_START_MINUTE));
  outputPrint(F("Stored Filter Duration: "));
  outputPrint(String(FILTER_DURATION));
  preferences.end();

  //Set initial LED state
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  // Start up the DallasTemperature library
  highLimitSensor.begin();
  waterSensor.begin();

  //Activate and check circulation pump, if bad flow sensor or no flow detected sets error state TRUE
  checkCirculationPump(); //Check if circulation pump is running

  //Setup MQTT Server and Callbacks
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWiFi));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);

  //connect to WiFi 
  WiFi.onEvent(WiFiEvent);
  wifiConnectTimer = millis() - wifiConnectTimeOut;  // reset connection timeout
  connectToWiFi();

  //setup Webserial
  WebSerial.begin(&server);
    /* Attach Message Callback */
  WebSerial.onMessage([&](uint8_t *data, size_t len) {
    Serial.write(data, len);
    WebSerial.println("Received Data...");
    String d = "";
    for(size_t i=0; i < len; i++){
      d += char(data[i]);
    }
    WebSerial.println(d);
    if (d.startsWith("r")) {
      outputPrintln("Restarting...");
      ESP.restart();
    }
  });
  server.begin();

  //Arduino OTA Setup
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      outputPrintln("Start updating " + type);
    })
    .onEnd([]() {
      outputPrintln("\nEnd");
      ESP.restart();
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //outputPrintln("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      //outputPrintf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        outputPrintln("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        outputPrintln("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        outputPrintln("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        outputPrintln("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        outputPrintln("End Failed");
      }
    });

  ArduinoOTA.begin();

  //Stabilize temperature sensors
  stabilizeSensors();

  //Turn on OZONE 
  if (ERROR_STATE = 0) digitalWrite(OZONE_PIN, ON);

}

void loop() {
  
  if (WiFi.status() != WL_CONNECTED && millis() > (wifiConnectTimer + wifiConnectTimeOut)) {
      connectToWiFi();
  }
  //Handle Arduino OTA
  ArduinoOTA.handle();

  //HOT TUB LOGIC **************************************************************************
  
  // send Error state 
  publishErrorState();

  // Check high limit sensor
  checkHighLimitSensor();

  // Check water temperature
  checkWaterTemperature();

  if (ERROR_STATE == 0 && !flowDetected && jet1State == 0) 
  {
    ERROR_STATE = 1;
    digitalWrite(HEATER_PIN, OFF);
    heaterState = OFF;
    outputPrintln(F("ERROR: No flow detected from circulation pump!"));
    
  }
  
  // If error state is false, check temperature and control heater
  if (ERROR_STATE == 0) {
    
    //only run heater if no jet pumps are on and flow is detected
    if (flowDetected && jet1State == OFF && jet2State == OFF) {
      // check temperature and turn heater on or off as needed
      checkTemperature();
    } else if (jet1State == ON || jet2State == ON) {
      if (heaterState != 2) {
        heaterState = JETS_RUNNING;
        digitalWrite(HEATER_PIN, OFF);  
        publishHeaterState();
        outputPrintln(F("Heater turned off due to pump operation."));
      }
    }else if (!flowDetected) { 
      ERROR_STATE = 1;
      publishErrorState();
      if (heaterState != OFF) {
        heaterState = OFF;
        digitalWrite(HEATER_PIN, OFF); 
        digitalWrite(CIRCULATION_PUMP_PIN, OFF); 
        outputPrintln(F("No flow detected, heater turned off.")); //display every 10 seconds
        publishHeaterState();
      }
    }

  }
  // END HOT TUB LOGIC ********************************************************************8

}

void outputPrintln(const String &msg) {
  String msgOut = msg;
  if (ERROR_STATE > 0) {
    msgOut = "Error State: ";
    msgOut = msgOut + msg;
  }
  Serial.println(msgOut);
  WebSerial.println(msgOut);
}

void outputPrint(const String &msg) {
  String msgOut = msg;
  if (ERROR_STATE > 0) {
    msgOut = "Error State: ";
    msgOut = msgOut + msg;
  }
  Serial.print(msgOut);
  WebSerial.print(msgOut);
}

void connectToWiFi() {
    // Red led while connecting to WiFi
  WiFi.disconnect();
  leds[0] = CRGB::Red;
  FastLED.show();
  WiFi.mode(WIFI_STA);
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      outputPrintln("STA Failed to configure");
    }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiConnectTimer = millis();  //reset connection attempt timer
  outputPrint(F("Connecting to WiFi .."));




//   while (WiFi.status() != WL_CONNECTED) {
//     Serial.print(F("."));
//     delay(1000);
//   }
}

void connectToMqtt() {
  outputPrintln("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    String output = "[WiFi-event] event: " + String(event);
    outputPrintln(output);
    IPAddress ip = WiFi.localIP();
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        // Blue led while connected to WiFi
        leds[0] = CRGB::Blue;
        FastLED.show();
        outputPrintln("WiFi connected");
        outputPrintln("IP address: ");
        outputPrintln(ip.toString());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        // Red led while connecting to WiFi
        leds[0] = CRGB::Red;
        FastLED.show();
        outputPrintln("WiFi lost connection");
        wifiConnectTimer = millis() + 2000; //reset connection timer timeout with additinal 2 sec delay  
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  outputPrintln("Connected to MQTT.");
  outputPrint("Session present: ");
  outputPrintln(String(sessionPresent));
  
  // Subscribe to topics
  uint16_t packetIdSub = mqttClient.subscribe(jet1buttonTopic, 1);
  outputPrint("Subscribing at QoS 1, packetId: ");
  outputPrintln(String(packetIdSub));

  packetIdSub = mqttClient.subscribe(jet2buttonTopic, 1);
  outputPrint("Subscribing at QoS 1, packetId: ");
  outputPrintln(String(packetIdSub));

  packetIdSub = mqttClient.subscribe(upButtonTopic, 1);
  outputPrint("Subscribing at QoS 1, packetId: ");
  outputPrintln(String(packetIdSub));

  packetIdSub = mqttClient.subscribe(downButtonTopic, 1);
  outputPrint("Subscribing at QoS 1, packetId: ");
  outputPrintln(String(packetIdSub));

    packetIdSub = mqttClient.subscribe(lightButtonTopic, 1);
  outputPrint("Subscribing at QoS 1, packetId: ");
  outputPrintln(String(packetIdSub));

  // Publish initial setpoint temperature
  publishSetpointTemperature();
  publishCurrentTemperature();
  publishHeaterState();
  publishErrorState();
  mqttClient.publish(lightStateTopic, 1, true, "OFF");
  mqttClient.publish(jet1StateTopic, 1, true, "OFF");
  mqttClient.publish(jet2StateTopic, 1, true, "OFF");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  outputPrintln("Disconnected from MQTT.");
  outputPrint("Reason: ");
  outputPrintln(String(static_cast<uint8_t>(reason)));
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  outputPrintln("Subscribe acknowledged.");
  // outputPrint("  packetId: ");
  // outputPrintln(packetId);
  // outputPrint("  qos: ");
  // outputPrintln(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  outputPrintln("Unsubscribe acknowledged.");
  outputPrint("  packetId: ");
  outputPrintln(String(packetId));
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  outputPrintln("Publish received.");
  // outputPrint("  topic: ");
  // outputPrintln(topic);
  // outputPrint("  qos: ");
  // outputPrintln(properties.qos);
  // outputPrint("  dup: ");
  // outputPrintln(properties.dup);
  // outputPrint("  retain: ");
  // outputPrintln(properties.retain);
  //outputPrint("  len: ");
  //outputPrintln(String(len));
  // outputPrint("  index: ");
  // outputPrintln(index);
  // outputPrint("  total: ");
  // outputPrintln(total);
    // Serial.print(".");
    // Serial.print(payload);
    // Serial.print(".");
  //Logic to handle received messages
  if (String(topic) == jet1buttonTopic) {
    if (strncmp(payload, "OFF",3)  == 0) {
      jet1State = 0;
      setJet1State();
    } else if (strncmp(payload, "LOW",3) == 0) {
      jet1State = 1;
      setJet1State();
    } else if (strncmp(payload, "HIGH",4) == 0) {
      jet1State = 2;
      setJet1State();
    }
  } else if (String(topic) == jet2buttonTopic) {
    if (strncmp(payload, "OFF",3) == 0) {
      jet2State = 0;
      setJet2State();
    } else if (strncmp(payload, "LOW",3) == 0) {
      jet2State = 1;
      setJet2State();
    } else if (strncmp(payload, "HIGH",4) ==0) {
      jet2State = 2;
      setJet2State();
    }
  } else if (String(topic) == upButtonTopic) {
    // Increase temperature setpoint
    if (TEMP_SETPOINT < 104) {
      TEMP_SETPOINT += 1;
      //Store setpoint in NVM
      preferences.begin("states", false);
      preferences.putInt("setPoint", TEMP_SETPOINT); 
      preferences.end(); 
      //outputPrint(F("Increased temperature setpoint to:"));
      //outputPrintln(String(TEMP_SETPOINT));
      publishSetpointTemperature();
    }
  } else if (String(topic) == downButtonTopic) {
    // Decrease temperature setpoint
    if (TEMP_SETPOINT > 50) {
      TEMP_SETPOINT -= 1;
      //Store Setpoint in NVM
      preferences.begin("states", false);
      preferences.putInt("setPoint", TEMP_SETPOINT); 
      preferences.end(); 
      //outputPrint(F("Decreased temperature setpoint to: "));
      //outputPrintln(String(TEMP_SETPOINT));
      publishSetpointTemperature();
    }
  } else if (String(topic) == lightButtonTopic) {
    // Toggle lights    
    if (strncmp(payload, "LOW",3) == 0) {
      lightState = 1;
      setSpaLight(lightLow); // Set to medium brightness
    } else if (strncmp(payload, "MEDIUM",6) == 0) {
      lightState = 2;
      setSpaLight(lightMedium); // Set to medium brightness
    } else if (strncmp(payload, "HIGH",4) == 0) {
      lightState = 3;
      setSpaLight(lightHigh); // Set to high brightness
    } else if (strncmp(payload, "OFF",3) == 0) {
      lightState = 0;
      setSpaLight(OFF); // Turn off lights
    }
  }
}

void onMqttPublish(uint16_t packetId) {
  outputPrintln("Publish acknowledged.");
  // outputPrint("  packetId: ");
  // outputPrintln(packetId);
}

void stabilizeSensors() {
  bool stabil = false;
  outputPrint(F("Wating for sensors to stabilize..."));
  while (!stabil) {
  outputPrint(F("."));
  highLimitSensor.requestTemperatures(); // Send the command to get temperatures
  float highLimitTemp = highLimitSensor.getTempFByIndex(0);
  waterSensor.requestTemperatures(); // Send the command to get temperatures
  currentTemp = waterSensor.getTempFByIndex(0);
  if (highLimitTemp == DEVICE_DISCONNECTED_F || currentTemp == DEVICE_DISCONNECTED_F) {
    outputPrintln(F("Error: Could not read temperature data from sensors!"));
    stabil = false;
  } else {
    outputPrintln(F("Sensors stabilized."));
    stabil = true;
  }
  delay(1000);
  } 
}

void checkCirculationPump() {
  // CHeck if circulation pump and flow sensor are working
  if (!flowDetected()) {
    digitalWrite(CIRCULATION_PUMP_PIN, ON);
    delay(5000); // wait 5 seconds to see if flow is detected
    if (!flowDetected()) {
      //digitalWrite(CIRCULATION_PUMP_PIN, OFF);
      outputPrintln(F("ERROR: No flow detected from circulation pump!"));
      ERROR_STATE = 1; // ERROR_STATE = true
    } else {
      outputPrintln(F("Circulation pump and flow sensor are working."));
      ERROR_STATE = 0; // ERROR_STATE = false
    }
  } else {
    outputPrintln(F("ERROR: Bad Flow Sensor - Flow detected before ciculation pump activated!"));
    ERROR_STATE = 2; // ERROR_STATE = true
  }
} 

bool flowDetected() {
  if (digitalRead(FLOW_SENSOR_PIN) == LOW) {
    return true;
  } else {
    return false;
  }
}

void checkTemperature() {
  if (millis() % 3000 != 0) return; // Check every 3 seconds
  //Serial.print(TEMP_SETPOINT - TEMP_HYSTERESIS);
  //Serial.print(" < ");
  //Serial.print(currentTemp);
  //Serial.print(" < ");
  //Serial.println(heaterState);
  if (heaterState != ON && currentTemp < (TEMP_SETPOINT - TEMP_HYSTERESIS)) {
    digitalWrite(HEATER_PIN, ON);
    heaterState = ON;
    outputPrintln(F("Heater ON"));
    publishHeaterState();
  } else if (heaterState != OFF && currentTemp > (TEMP_SETPOINT + TEMP_HYSTERESIS)) {
    digitalWrite(HEATER_PIN, OFF);
    heaterState = OFF;
    outputPrintln(F("Heater OFF"));
    publishHeaterState();
  }
}

void checkHighLimitSensor() {
  if (ERROR_STATE > 0) return; // If already in error state, skip checking
  if (millis() % 10000 != 0) return; // Check every 10 seconds

  highLimitSensor.requestTemperatures(); // Send the command to get temperatures
  highLimitTemp = highLimitSensor.getTempFByIndex(0);
  if (highLimitTemp >= HIGH_LIMIT) {
    digitalWrite(HEATER_PIN, OFF);
    heaterState = OFF;
    ERROR_STATE = 3;
    outputPrintln(F("ERROR: High limit temperature exceeded! Heater turned off."));
    publishHeaterState();
  }
}

void checkWaterTemperature() {
  if (millis() % 15000 != 0) return; // Check every 15 seconds
  waterSensor.requestTemperatures(); // Send the command to get temperatures
  currentTemp = waterSensor.getTempFByIndex(0);
  //outputPrint(F("Current water temperature: "));
  //outputPrintln(String(currentTemp));
  publishCurrentTemperature();
}

void publishSetpointTemperature() {
  JsonDocument doc;
  doc["temperature"] = String(TEMP_SETPOINT).c_str();
  char output[256];
  serializeJson(doc, output);
  outputPrint(F("Publishing setpoint temperature to MQTT..."));
  outputPrint(output);
  mqttClient.publish(setpointTempTopic, 1, true, output);
}

void publishCurrentTemperature() {
  JsonDocument doc;
  char charArray[10];
  doc["temperature"] = String(currentTemp).c_str();
  char output[256];
  serializeJson(doc, output);
  outputPrint(F("Publishing current temperature to MQTT..."));
  outputPrint(output);
  
  // convert high limit temp to char array
  snprintf(charArray, sizeof(charArray), "%.1f", highLimitTemp);

  // publish to mqtt clients
  mqttClient.publish(highLimitTopic, 1, true, charArray);
  mqttClient.publish(waterTempTopic, 1, true, output);
}

void publishHeaterState() {
  JsonDocument doc;
  if (heaterState == ON) {
    doc["state"] = "ON";
  } else if (heaterState == OFF || heaterState == JETS_RUNNING) {
    doc["state"] = "OFF";
  }
  char output[256];
  serializeJson(doc, output);
  outputPrint(F("Publishing heater state to MQTT..."));
  outputPrint(output);
  mqttClient.publish(heaterStateTopic, 1, true, output);
}

void publishErrorState() {
   if (millis() % 30000 != 0) return;
  char output[4];
  sprintf(output,"%d",ERROR_STATE);
  outputPrint(F("Publishing error state to MQTT..."));
  outputPrint(output);
  mqttClient.publish(errorStateTopic, 1, true, output);
}

void setJet1State() {
  if (jet1State == 1) {
    digitalWrite(PUMP1_HIGH_PIN, OFF);
    delay(250);
    digitalWrite(PUMP1_LOW_PIN, ON);
    outputPrintln(F("Pump 1 set to LOW"));
  } else if (jet1State == 2) {
    digitalWrite(PUMP1_LOW_PIN, OFF);
    delay(250);
    digitalWrite(PUMP1_HIGH_PIN, ON);
    outputPrintln(F("Pump 1 set to HIGH"));
  } else {
    digitalWrite(PUMP1_HIGH_PIN, OFF);
    digitalWrite(PUMP1_LOW_PIN, OFF);
    outputPrintln(F("Pump 1 turned OFF"));
  }
  
  //publish to mqtt
  JsonDocument doc;
  if (jet1State == 1) doc["state"] = "LOW";
   else if (jet1State == 2) doc["state"] = "HIGH";
   else doc["state"] = "OFF";
  char output[256];
  serializeJson(doc, output);
  outputPrint(F("Publishing pump 1 state to MQTT..."));
  outputPrint(output);
  mqttClient.publish(jet1StateTopic, 1, true, output);
}

void setJet2State() {
  if (jet2State == 1) {
    digitalWrite(PUMP2_HIGH_PIN, OFF);
    delay(500);
    digitalWrite(PUMP2_LOW_PIN, ON);
    outputPrintln(F("Pump 2 set to LOW"));
  } else if (jet2State == 2) {
    digitalWrite(PUMP2_LOW_PIN, OFF);
    delay(500);
    digitalWrite(PUMP2_HIGH_PIN, ON);
    outputPrintln(F("Pump 2 set to HIGH"));
  } else {
    digitalWrite(PUMP2_HIGH_PIN, OFF);
    digitalWrite(PUMP2_LOW_PIN, OFF);
    outputPrintln(F("Pump 2 turned OFF"));
  }
  
  //publish to mqtt
  JsonDocument doc;
  if (jet2State == 1) doc["state"] = "LOW";
   else if (jet2State == 2) doc["state"] = "HIGH";
   else doc["state"] = "OFF";
  char output[256];
  serializeJson(doc, output);
  outputPrint(F("Publishing pump 2 state to MQTT..."));
  outputPrint(output);
  mqttClient.publish(jet2StateTopic, 1, true, output);
}

// Set spa light brightness (0-255)
void setSpaLight(int brightness) {
  ledcWrite(channel, brightness);
  outputPrint(F("Setting spa light brightness to: "));
  outputPrint(String(brightness));
   
  //publish to mqtt
  JsonDocument doc;
  if (lightState == 1) doc["state"] = "LOW";
   else if (lightState == 2) doc["state"] = "MEDIUM";
   else if (lightState == 3) doc["state"] = "HIGH";
   else if (lightState == OFF) doc["state"] = "OFF1";
  char output[256];
  serializeJson(doc, output);
  outputPrint(F("Publishing light state to MQTT..."));
  outputPrint(output);
  mqttClient.publish(heaterStateTopic, 1, true, output);
}