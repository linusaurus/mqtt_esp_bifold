

/*
Design Synthesis- r.young
BiFold relay control with Autoconnect and MQTT controls
for bifolding doors [linear control with limits]
capacitive switch
*/
#include <FS.h>
#include <ArduinoJson.h>    
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <AceButton.h>
using namespace ace_button;
#include <AdjustableButtonConfig.h>
#include <ButtonConfig.h>
#include <Atm_esp8266.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>  
//-----------------------------------------------------------------------------------
// WiFI network values
//-----------------------------------------------------------------------------------
const char* ssid = "MonkeyRanch";
const char* password = "12345678";
const char* mqtt_server = "192.168.10.62";
WiFiClient espClient;
PubSubClient client(espClient);
//-----------------------------------------------------------------------------------
long lastMsg = 0;
char msg[50];
int value = 0;
unsigned long   BEACON_DELAY = 2000;
unsigned long   INTERVAL_DURATION = 500;
unsigned long   RELAY_FIRED;
unsigned long   ONLINE_START;
unsigned long   TOP_RANGE;
const int BUTTON_PIN = D0;


Atm_led led;
Atm_timer timer;
int DOOR_STATE = 0;
int LAST_STATE = 0;
int LAST_COMMAND = 0;
bool STARTUP= false;

AceButton button(BUTTON_PIN);
AdjustableButtonConfig adjustableButtonConfig;
void handleEvent(AceButton*, uint8_t, uint8_t);

//flag for saving data
bool shouldSaveConfig = false;

//define your default values here, if there are different values in config.json, they are overwritten.
//=====================================================================================================
#define mqtt_server  "192.168.10.62"
#define mqtt_port    "1883"
#define mqtt_user    ""
#define mqtt_pass    ""
#define outTopic =   "STATUS"
#define inTopic =    "TEST"
#define USE_PULSED_RELAY
#define TOKEN "mnwc5ijPmSaiyTejTpji"
//====================================================================================================

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}



void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  //clean FS, for testing
  //SPIFFS.format();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
 //--------------PINS-------------------------------
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(D5,OUTPUT);
  pinMode(D6,OUTPUT);
  pinMode(D7,OUTPUT);
  digitalWrite(D5,HIGH);
  digitalWrite(D6,HIGH);
  digitalWrite(D7,LOW);

 
  //-------------------------------------------------
  EEPROM.begin(512); 
  LAST_STATE = EEPROM.read(0);
  LAST_COMMAND=EEPROM.read(1);
  Serial.print("LAST_STATE=");
  Serial.println(LAST_STATE);
  Serial.print("LAST_COMMAND=");
  Serial.println(LAST_COMMAND);
  
 // Override the System ButtonConfig with our own AdjustableButtonConfig
  // instance instead.
  button.setButtonConfig(&adjustableButtonConfig);

 // Configure the ButtonConfig with the event handler and enable LongPress.
 // SupressAfterLongPress is optional since we don't do anything if we get it.
  adjustableButtonConfig.setEventHandler(handleEvent);
  adjustableButtonConfig.setFeature(ButtonConfig::kFeatureLongPress);
  adjustableButtonConfig.setLongPressDelay(2000);
 
  void handleEvent(AceButton*, uint8_t, uint8_t);


  

   if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, 20);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // Reset Wifi settings for testing  
  //wifiManager.resetSettings();

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
 
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimum quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connection Success :)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());


  //save the custom parameters to FS
  if (shouldSaveConfig) {
    
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    //json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  
//client.setServer(mqtt_server, 12025);
// this strange assignment of port needed to be changed
  const uint16_t mqtt_port_x = 1883; 
  client.setServer(mqtt_server, mqtt_port_x);
  client.setCallback(callback);

#ifdef MQTT_CLIENT

#endif

led.begin(BUILTIN_LED).blink(1500,40);
led.trigger(led.EVT_BLINK);

ONLINE_START = millis();


}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//---------------------------------------------------------------------------------------------------------------
// MQTT Callback logic
//---------------------------------------------------------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  
  Serial.println();

  if ((char)payload[0] == '1') {
   
    DOOR_STATE=1;
    LAST_STATE=2;
    Serial.println("DOOR STATE = 1");
    Serial.println(LAST_STATE);
    Serial.println("MQTT_OPEN");
   //client.publish("outTopic", "BiDOOR-OPEN");
   
      
  } 
  if ((char)payload[0] == '2') {

    DOOR_STATE=2;
    LAST_STATE=1;
    Serial.println("DOOR STATE = 2");
    Serial.println(LAST_STATE);
    Serial.println("MQTT_CLOSE");
    
  }

}
//--------------------------------------------------------------------------------------------------------------
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("BIFOLDSTATUS", "hello world");
      // ... and resubscribe
      client.subscribe("TEST");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  button.check();
  //OnSwitchChanged();
  OnStateChanged();
  
 
    
  long now = millis();
  
  if (now - RELAY_FIRED > INTERVAL_DURATION) {
      digitalWrite(D5,HIGH);
      digitalWrite(D6,HIGH);
  }
  if (now - ONLINE_START > 3000) {
      digitalWrite(D7,HIGH);
      STARTUP=false;
  }
  
  //snprintf (msg, 75, "door state=%ld", DOOR_STATE);
   automaton.run();
}

//-------------------------------------------------------------------------------------------------------------------
// Events
//-------------------------------------------------------------------------------------------------------------------

void OnSwitchChanged()
{
  if((DOOR_STATE==0 || DOOR_STATE==2) && digitalRead(D0)==HIGH){
    DOOR_STATE=1;
    LAST_COMMAND=1;
    EEPROM.write(1,LAST_COMMAND);
    EEPROM.commit();
    Serial.println("SWITCH_OPEN");

  }
  
  if(DOOR_STATE==1 && digitalRead(D0)==LOW){
    DOOR_STATE=2;
    LAST_COMMAND=2;
    Serial.println("SWITCH_CLOSE");
    EEPROM.write(1,LAST_COMMAND);
    EEPROM.commit();
  }
}

void OnSwitchStateChanged(){
  
}

void OnStateChanged()
{
  
  
  if(DOOR_STATE==1 && LAST_STATE==2){
      digitalWrite(D5,LOW);
      RELAY_FIRED = millis();
      Serial.println("HIGH");
      LAST_STATE=1;
      EEPROM.write(1,LAST_STATE);
      EEPROM.commit();
    }
  if(DOOR_STATE==2 && LAST_STATE==1){
      digitalWrite(D6,LOW);
      RELAY_FIRED = millis();
      Serial.println("LOW");
      LAST_STATE=2;
      EEPROM.write(1,LAST_STATE);
      EEPROM.commit();
    }
  
}

void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t /* buttonState */) {
  switch (eventType) {

    case AceButton::kEventPressed:
   
      if(DOOR_STATE==1){
        DOOR_STATE=2;
        LAST_STATE=1;     
        Serial.print("Door State=");
        Serial.println(DOOR_STATE);
      }else{
        DOOR_STATE=1;
        LAST_STATE=2;
        Serial.print("Door State=");
        Serial.println(DOOR_STATE);        
      }
 
      break;
      
    
  }
}


//---------------------------------------------------------------------------------------------------------
