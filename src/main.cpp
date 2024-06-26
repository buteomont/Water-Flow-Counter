/*
 * ESP8266 firmware to measure the flow rate of the water system and send it to an MQTT 
 * broker.
 * Configuration is done via serial connection.  
 * Press ENTER on an empty line to get the current values.
 * The code is written for an ESP8266-01S processor. The sensor
 * should be connected between pin 3 (GPIO2) and ground (pin 1).
 *  
 */
const char* VERSION = "24.04.18.0";  //remember to update this after every change! YY.MM.DD.REV

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h> 
#include <EEPROM.h>
#include "mqtt_flow_counter.h"

// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate 
//    flashed in the WiFi module.

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//mqtt stuff
unsigned long lastMessageSent = 0;
int messageCount = 0;
unsigned long lastReportTime=0;  // This is the time of the last report
unsigned long lastReportCount=0; // This is the pulse count at the last report
boolean finalReportSent=true;
boolean startup=true;           // set to false after everything is working

boolean restartNeeded=false;    // Sent as a reminder when settings change

//flow stuff
boolean lastTick=false;
unsigned long pulseCount=0;
float liters=0.0;
unsigned long lastPulseTime=0;
volatile bool pulseDetected = false; //interrupt sets this, loop resets it

// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve from EEPROM.
typedef struct 
  {
  unsigned int validConfig=0; 
  char ssid[SSID_SIZE] = "";
  char wifiPassword[PASSWORD_SIZE] = "";
  char mqttBrokerAddress[ADDRESS_SIZE]=""; //default
  int mqttBrokerPort=1883;
  char mqttUsername[USERNAME_SIZE]="";
  char mqttPassword[PASSWORD_SIZE]="";
  char mqttTopicRoot[MQTT_TOPIC_SIZE]="";
  char mqttClientId[MQTT_CLIENT_ID_SIZE]="";
  float pulsesPerLiter=DEFAULT_PULSES_PER_LITER;
  unsigned int reportInterval=DEFAULT_REPORT_INTERVAL; //seconds
  int sensorType=INPUT; // INPUT, INPUT_PULLUP, INPUT_PULLDOWN_16
  } conf;

conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

int pulseCountStorage=sizeof(settings); //use the EEPROM location after settings to store the pulse counter occasionally

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed

void reboot()
  {
  storePulseCount(); //just in case
  delay(500);
  ESP.restart();
  }

//Convert the input type code to human-readable
char* getSensorTypeText(char* buf)
  {
  int st=settings.sensorType;
  if (st==INPUT) strcpy(buf,"INPUT"); 
  else if (st==INPUT_PULLUP) strcpy(buf,"INPUT_PULLUP");
  else if (st==INPUT_PULLDOWN_16) strcpy(buf,"INPUT_PULLDOWN");
  else strcpy(buf,"NOT SET");
  return buf;
  }


void setup() 
  {
  //Initialize the serial port and wait for it to open 
  Serial.begin(115200);

//  Serial.setTimeout(10000);
  
  // while (!Serial) 
  //   {
  //   delay(1000); // wait for serial port to connect. Needed for native USB port only
  //   }

  //Set up hardware
  pinMode(SENSOR_PIN,INPUT);  //may change after we load the settings object

#ifdef BOARD_D1_MINI
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); //turn on the LED to show we are booting
#endif
  
    
  Serial.println(); //first print
  delay(500);
  Serial.println("\n***************************************************");
  Serial.print("MQTT flow counter version ");
  Serial.print(getVersion());
  Serial.println(" starting up...");
  Serial.println("***************************************************\n");

  EEPROM.begin(sizeof(settings)+sizeof(pulseCount)); //fire up the eeprom section of flash
  // Serial.print("Settings object size=");
  // Serial.println(sizeof(settings));

    
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string

  loadSettings(); //set the values from eeprom

  Serial.println("\nConfiguration is done via serial connection.  You can enter:\n");
  showSettings(); //show them
  
  if (settings.mqttBrokerPort < 0) //then this must be the first powerup. Fix the random stuff in there
    {
    Serial.println("\n*********************** Resetting All EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    resetPulseCounter(); //store the zeroed pulse counter too
    delay(2000);
    reboot();
    }
  loadPulseCount(); //restore the pulse count value from before power loss

  if (settingsAreValid)
    {
    pinMode(SENSOR_PIN,settings.sensorType);

    if (connectToWifi())
      {
      Serial.println("\nConnected to network.");
      Serial.println();

      // ********************* Initialize the MQTT connection
      mqttClient.setServer(settings.mqttBrokerAddress, settings.mqttBrokerPort);
      mqttClient.setCallback(incomingMqttHandler);
      mqttClient.setBufferSize(JSON_STATUS_SIZE); //that's the biggest one
        
      delay(2000);  //give wifi a chance to warm up
      reconnectToBroker();
      }    
    else
      {
      Serial.println("\nUnable to connect to network.");
      Serial.println();
      }
    }

  char tempbuf[15]="Unknown";
  Serial.print("Sensor pin is ");
  Serial.print(SENSOR_PIN);
  Serial.print(" (");
  Serial.print(getSensorTypeText(tempbuf));
  Serial.println(")");
  
#ifdef BOARD_D1_MINI
  digitalWrite(LED_BUILTIN, HIGH); //turn off the LED
#endif

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), handleInterrupt, FALLING); // Attach interrupt handler to sensor pin with falling edge detection
  }

boolean connectToWifi()
  {
  if (settingsAreValid)
    {
    while (WiFi.status() != WL_CONNECTED)
      {
      Serial.print("WiFi status is ");
      Serial.println(WiFi.status());

      // ********************* attempt to connect to Wifi network
      Serial.print("Attempting to connect to WPA SSID \"");
      Serial.print(settings.ssid);
      Serial.println("\"");

      //Serial.println(WiFi.status());

      WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world
      WiFi.begin(settings.ssid, settings.wifiPassword);
//      WiFi.waitForConnectResult(); //hang around for a bit

      //if we're still not connected, wait a little longer
      int waitCount=0;
      while (WiFi.status() != WL_CONNECTED) 
        {
        Serial.print(".");
        if (waitCount++ >= MAX_WIFI_WAIT_COUNT)
          break;

        checkForCommand(); // Check for input in case it needs to be changed to work
        delay(2000);
        }
      if (WiFi.status() != WL_CONNECTED)
        Serial.println("Unable to connect.  Retrying...");
      }

    }
  return WiFi.status() == WL_CONNECTED;
  }

/**
 * Handler for incoming MQTT messages.  The payload is the command to perform. The MQTT message topic sent is the  
 * topic root plus the command.
 * Implemented commands are: 
 * MQTT_PAYLOAD_SETTINGS_COMMAND: sends a JSON payload of all user-specified settings
 * MQTT_PAYLOAD_RESET_PULSE_COMMAND: Reset the pulse counter to zero
 * MQTT_PAYLOAD_REBOOT_COMMAND: Reboot the controller
 * MQTT_PAYLOAD_VERSION_COMMAND Show the version number
 * MQTT_PAYLOAD_STATUS_COMMAND Show the most recent flow values
 */
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) 
  {
  payload[length]='\0'; //this should have been done in the caller code, shouldn't have to do it here
  boolean rebootScheduled=false; //so we can reboot after sending the reboot response
  char charbuf[100];
  sprintf(charbuf,"%s",payload);
  char response[JSON_STATUS_SIZE];
  
  if (strcmp((char*)payload,MQTT_TOPIC_COMMAND_REQUEST)==0)
    {
    return; //someone's messing with us, ignore them
    }  
  else if (strcmp(charbuf,MQTT_PAYLOAD_SETTINGS_COMMAND)==0)
    {
    strcpy(response,getMqttSettings());// send all of the settings
    Serial.println(response);
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_RESET_PULSE_COMMAND)==0)
    {
    resetPulseCounter();
    strcpy(response,"OK");
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_VERSION_COMMAND)==0) //show the version number
    {
    strcpy(response,getVersion());
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_STATUS_COMMAND)==0) //show the latest flow values
    {
    if (getMqttStatus())
      strcpy(response,"Status report complete");
    }
  else if (strcmp(charbuf,MQTT_PAYLOAD_REBOOT_COMMAND)==0) //reboot the controller
    {
    strcpy(response,"REBOOTING");
    rebootScheduled=true;
    }
  else if (std::strchr(charbuf, '=') != nullptr) //we have a setting to change
    {
    processCommand(charbuf);
    strcpy(response,getMqttSettings());
    }
  else
    {
    strcpy(response,"(empty)");
    }
    
  char topic[MQTT_TOPIC_SIZE];
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,charbuf); //the incoming command becomes the topic suffix

  if (!publish(topic,response))
    Serial.println("************ Failure when publishing status response!");
  
  if (rebootScheduled)
    {
    delay(2000); //give publish time to complete
    reboot();
    }
  }

void showSettings()
  {
  char buf[15];
  Serial.print("broker=<MQTT broker host name or address> (");
  Serial.print(settings.mqttBrokerAddress);
  Serial.println(")");
  Serial.print("port=<port number>   (");
  Serial.print(settings.mqttBrokerPort);
  Serial.println(")");
  Serial.print("topicRoot=<topic root> (");
  Serial.print(settings.mqttTopicRoot);
  Serial.println(")  Note: must end with \"/\"");  
  Serial.print("user=<mqtt user> (");
  Serial.print(settings.mqttUsername);
  Serial.println(")");
  Serial.print("pass=<mqtt password> (");
  Serial.print(settings.mqttPassword);
  Serial.println(")");
  Serial.print("ssid=<wifi ssid> (");
  Serial.print(settings.ssid);
  Serial.println(")");
  Serial.print("wifipass=<wifi password> (");
  Serial.print(settings.wifiPassword);
  Serial.println(")");
  Serial.print("pulsesPerLiter=<number of pulses generated per liter of flow>   (");
  Serial.print(settings.pulsesPerLiter,4);
  Serial.println(")");
  Serial.print("reportInterval=<number of seconds between MQTT reports>   (");
  Serial.print(settings.reportInterval);
  Serial.println(")");
  Serial.print("pulseCount=<cumulative pulse count>   (");
  Serial.print(pulseCount);
  Serial.println(")");
  Serial.print("sensorType=<INPUT, INPUT_PULLUP, or INPUT_PULLDOWN>   (");
  Serial.print(getSensorTypeText(buf));
  Serial.println(")");
  Serial.print("Generated Client Id = ");
  Serial.println(settings.mqttClientId);
  Serial.print("IP Address is ");
  Serial.print(WiFi.localIP().toString().c_str());
  Serial.println("\n");

  Serial.println("\"reboot=yes\" to reboot the controller");
  Serial.println("\"resetPulses=yes\" to reset the pulse counter to zero");
  Serial.println("*** Use \"factorydefaults=yes\" to reset all settings ***\n");

  if (restartNeeded)
    Serial.println("\n***** Warning: Reboot is needed *****");
  }

void generateMQTTClientId()
  {
  // Create a random client ID
  String clientId = MQTT_CLIENT_ROOT;
  clientId += String(random(0xffff), HEX);
  strcpy(settings.mqttClientId,clientId.c_str());
  }

/*
 * Reconnect to the MQTT broker
 */
void reconnectToBroker() 
  {
  if (WiFi.status() != WL_CONNECTED)
    {
    connectToWifi();
    return;
    }
  
  // Loop until we're reconnected
  while (!mqttClient.connected()) 
    {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (mqttClient.connect(settings.mqttClientId,settings.mqttUsername,settings.mqttPassword))
      {
      Serial.println("connected to MQTT broker.");

      //resubscribe to the incoming message topic
      char topic[MQTT_TOPIC_SIZE];
      strcpy(topic,settings.mqttTopicRoot);
      strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
      mqttClient.subscribe(topic);
      }
    else 
      {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Will try again in 5 seconds");
      
      // Wait 5 seconds before retrying
      // In the meantime check for input in case something needs to be changed to make it work
      checkForCommand(); 
      
      delay(5000);
      }
    }
  if (startup)
    {
    char topic[MQTT_TOPIC_SIZE];
    char message[18];
    boolean success=false;

    //Inform the world that we rebooted
    strcpy(topic,settings.mqttTopicRoot);
    strcat(topic,MQTT_TOPIC_INFO);
    sprintf(message,"%s","Startup Complete");
    success=publish(topic,message);
    if (!success)
      Serial.println("************ Failed publishing startup message!");
    startup=false;
    }
  }

ICACHE_RAM_ATTR void handleInterrupt() //interrupts must be in IRAM
  {
  pulseDetected = true;
  }

void handlePulse()
  {
  static unsigned long lastDebounceTime = 0;
  unsigned long currentMillis = millis();
  pulseDetected = false;

  // See if enough time has passed since the last transition
  if (currentMillis - lastDebounceTime >= DEBOUNCE_DELAY) 
    {
    pulseCount++;
    lastDebounceTime = currentMillis;
    
    Serial.println("Pulse!");
    lastPulseTime=millis();
    lastTick=!lastTick; // used to flash the light 
    liters=pulseCount/settings.pulsesPerLiter;
#ifdef BOARD_D1_MINI
    digitalWrite(LED_BUILTIN, lastTick?LOW:HIGH); //HIGH is LED OFF
#endif
    }
  }

void sendReport()
  {
  static bool pulseSaved=true; //On first run, assume pulse was saved so we don't overwrite it

  unsigned long ts=millis();
  if (ts-lastReportTime>settings.reportInterval*1000)
    {
    report();
    lastReportTime=ts;

    // If the last report was the same as this report, and we haven't done it yet,
    // then the valve must be closed and the tank is full.
    // Store the pulse count into flash in case we lose power.  We don't want to 
    // do this very often because it will wear out the flash.
    if (lastReportCount==pulseCount && !pulseSaved)
      {
      storePulseCount(); //save the pulse count value in case we lose power
      pulseSaved=true;
      }
    else if (lastReportCount!=pulseCount)
      {
      pulseSaved=false;
      }
    lastReportCount=pulseCount; //If they stay equal then the water has stopped flowing
#ifdef BOARD_D1_MINI
    digitalWrite(LED_BUILTIN, HIGH); //turn off the LED when water stops
#endif
    }
  }


/*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    Serial.println(commandString);
    String newCommand=commandString;

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

void processCommand(char* str)
  {
  // Serial.print("Received this line:");
  // Serial.println(str);
  char *val=NULL;
  char *nme=strtok((char *)str,"=");
  if (nme!=NULL)
    val=strtok(NULL,"=");

  //Get rid of the carriage return
  if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
    val[strlen(val)-1]=0; 

  if (nme==NULL || val==NULL || strlen(nme)==0 || strlen(val)==0)
    {
    showSettings();
    
    return;  
    }
  else if (strcmp(nme,"broker")==0)
    {
    strcpy(settings.mqttBrokerAddress,val);
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
  else if (strcmp(nme,"port")==0)
    {
    settings.mqttBrokerPort=atoi(val);
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
  else if (strcmp(nme,"topicRoot")==0)
    {
    // Trim whitespace from the end
    char* end = val + strlen(val) - 1;
    while (end > val && isspace(*end)) 
      *end-- = '\0';

    // make sure the string ends with "/"
    if (val[strlen(val) - 1] != '/')
      { 
      const char* slash = "/";
      strcat(val, slash);
      }
    strcpy(settings.mqttTopicRoot,val);
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
  else if (strcmp(nme,"user")==0)
    {
    strcpy(settings.mqttUsername,val);
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
  else if (strcmp(nme,"pass")==0)
    {
    strcpy(settings.mqttPassword,val);
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
  else if (strcmp(nme,"ssid")==0)
    {
    strcpy(settings.ssid,val);
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
  else if (strcmp(nme,"wifipass")==0)
    {
    strcpy(settings.wifiPassword,val);
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
  else if (strcmp(nme,"pulsesPerLiter")==0)
    {
    settings.pulsesPerLiter=atof(val);
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
  else if (strcmp(nme,"pulseCount")==0) //might need to manually update it
    {
    pulseCount=atoi(val);
    liters=pulseCount/settings.pulsesPerLiter;
    storePulseCount();
    Serial.print("Pulse count set to ");
    Serial.println(pulseCount);
    }
  else if (strcmp(nme,"reportInterval")==0)
    {
    // Serial.print(val);
    // Serial.print(":");
    // Serial.println(atoi(val));
    settings.reportInterval=atoi(val);
//    Serial.println(settings.reportInterval);
    saveSettings();
    }
  else if (strcmp(nme,"sensorType")==0)
    {
    if (strcmp(val,"INPUT")==0)
      settings.sensorType=INPUT;
    else if (strcmp(val,"INPUT_PULLUP")==0)
      settings.sensorType=INPUT_PULLUP;
    else if (strcmp(val,"INPUT_PULLDOWN")==0)
      settings.sensorType=INPUT_PULLDOWN_16;
    saveSettings();
    if (settingsAreValid)
      restartNeeded=true;
    }
else if ((strcmp(nme,"reboot")==0) && (strcmp(val,"yes")==0)) //reboot the controller
    {
    Serial.println("\n*********************** Rebooting! ************************");
    delay(2000);
    reboot();
    }
  else if ((strcmp(nme,"resetPulses")==0) && (strcmp(val,"yes")==0)) //reset the pulse counter
    {
    Serial.println("\n*********** Resetting pulse counter ************");
    resetPulseCounter();
    }
  else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
    {
    Serial.println("\n*********************** Resetting EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    resetPulseCounter(); //store the zeroed pulse counter too
    delay(2000);
    reboot();
    }
  else
    showSettings();
  return;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  strcpy(settings.ssid,"");
  strcpy(settings.wifiPassword,"");
  strcpy(settings.mqttBrokerAddress,""); //default
  settings.mqttBrokerPort=1883;
  strcpy(settings.mqttUsername,"");
  strcpy(settings.mqttPassword,"");
  strcpy(settings.mqttTopicRoot,"");
  settings.pulsesPerLiter=DEFAULT_PULSES_PER_LITER;
  settings.reportInterval=DEFAULT_REPORT_INTERVAL;
  pulseCount=0;
  generateMQTTClientId();
  }

void loop() 
  {
  // serialEvent is not interrupt driven on the ESP32 for some reason. Do it here.
  checkForCommand();
          
  // call loop() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker.  Don't call if not set up yet
  // because the WDT on the ESP8266 will reset the processor. Not a problem on ESP32.
  if (settingsAreValid)
    {
    connectToWifi(); //should already be connected, but check anyway
    reconnectToBroker();  //won't do anything unless there's a problem
    mqttClient.loop();

    //See if a new pulse has arrived. If so, handle it.
    if (pulseDetected)
      {
      handlePulse();
      }
    sendReport(); //checks to see if any pulses haven't been timely reported
    }

  if (millis()%60000==0 && restartNeeded)
    {
    Serial.println("\n***** Warning: Reboot is needed *****");
    delay(20); //to keep it from repeating several times
    }
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    incomingSerial();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      char *str=(char*)cmd.c_str();
      processCommand(str);
      }
    }
  }

/************************
 * Do the MQTT thing
 ************************/
void report()
  {  
  char topic[MQTT_TOPIC_SIZE];
  char reading[18];
  boolean success=false;

  //publish the raw pulse count
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_RAW);
  sprintf(reading,"%lu",pulseCount);
  success=publish(topic,reading);
  if (!success)
    Serial.println("************ Failed publishing pulse count!");

  //publish the liter count
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_LITERS);
  sprintf(reading,"%.1f",liters);
  success=publish(topic,reading);
  if (!success)
    Serial.println("************ Failed publishing liter count!");

  // //publish the raw milliseconds between ticks
  // strcpy(topic,settings.mqttTopicRoot);
  // strcat(topic,MQTT_TOPIC_PERIOD);
  // sprintf(reading,"%lu",pulsePeriod);
  // success=publish(topic,reading);
  // if (!success)
  //   Serial.println("************ Failed publishing tick rate!");
  }

boolean publish(char* topic, char* reading)
  {
  Serial.print(messageCount++);
  Serial.print("\t");
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(reading);
  return mqttClient.publish(topic,reading);
  }

  
/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);
  if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    Serial.println("Loaded configuration values from EEPROM");
//    showSettings();
    }
  else
    {
    Serial.println("Skipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (strlen(settings.ssid)>0 &&
    strlen(settings.wifiPassword)>0 &&
    strlen(settings.mqttBrokerAddress)>0 &&
    settings.mqttBrokerPort!=0 &&
//    strlen(settings.mqttUsername)>0 &&
//    strlen(settings.mqttPassword)>0 &&
    strlen(settings.mqttTopicRoot)>0 &&
    settings.pulsesPerLiter!=0 &&
    (settings.sensorType==INPUT 
      || settings.sensorType==INPUT_PULLUP 
      || settings.sensorType==INPUT_PULLDOWN_16) &&
    settings.reportInterval>0)
    {
    Serial.println("Settings deemed valid");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    if (strncmp(settings.mqttClientId, MQTT_CLIENT_ROOT,sizeof(MQTT_CLIENT_ROOT) - 1)!=0)
      generateMQTTClientId();
    }
  else
    {
    Serial.println("Settings deemed incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }
    
  EEPROM.put(0,settings);
  return EEPROM.commit();
  }

  
/*
 * Save the pulse counter to eeprom
 * 
 */
 boolean storePulseCount()
  {
  EEPROM.put(pulseCountStorage,pulseCount);
  boolean saved=EEPROM.commit();

  char topic[MQTT_TOPIC_SIZE];
  char message[35];

  //publish the event of saving the pulse count
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_SAVE_PULSE_COUNT);
  sprintf(message,"%lu",pulseCount);
  publish(topic,message);

  return saved;
  }
  
/*
 * Read the pulse counter from eeprom
 * 
 */
 void loadPulseCount()
  {
  Serial.print("Restoring pulse counter (");
  EEPROM.get(pulseCountStorage,pulseCount);
  lastReportCount=pulseCount;
  Serial.print(pulseCount);
  Serial.println(")");
  liters=pulseCount/settings.pulsesPerLiter;
//  pulseCount=0; //removed storage of pulse counter due to flash write endurance
  }

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void incomingSerial() {
  while (Serial.available()) 
    {
    // get the new byte
    char inChar = (char)Serial.read();
    Serial.print(inChar); //echo it back

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n') 
      {
      commandComplete = true;
      }
    else
      {
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }

void resetPulseCounter()
  {
  pulseCount=0;
  storePulseCount(); //store the zeroed pulse counter
  }


//**********************************************************************
//********************** MQTT Command Handlers *************************
//**********************************************************************

char* getMqttSettings()
  {
  char tempbuf[15]; //for converting numbers to strings
  static char jsonStatus[JSON_STATUS_SIZE];

  strcpy(jsonStatus,"{");
  strcat(jsonStatus,"\"broker\":\"");
  strcat(jsonStatus,settings.mqttBrokerAddress);
  strcat(jsonStatus,"\", \"port\":");
  sprintf(tempbuf,"%d",settings.mqttBrokerPort);
  strcat(jsonStatus,tempbuf);
  strcat(jsonStatus,", \"topicRoot\":\"");
  strcat(jsonStatus,settings.mqttTopicRoot);
  strcat(jsonStatus,"\", \"user\":\"");
  strcat(jsonStatus,settings.mqttUsername);
  strcat(jsonStatus,"\", \"pass\":\"");
  strcat(jsonStatus,settings.mqttPassword);
  strcat(jsonStatus,"\", \"ssid\":\"");
  strcat(jsonStatus,settings.ssid);
  strcat(jsonStatus,"\", \"wifipass\":\"");
  strcat(jsonStatus,settings.wifiPassword);
  strcat(jsonStatus,"\", \"pulsesPerLiter\":");
  sprintf(tempbuf,"%.4f",settings.pulsesPerLiter);
  strcat(jsonStatus,tempbuf);
  strcat(jsonStatus,", \"reportInterval\":");
  sprintf(tempbuf,"%d",settings.reportInterval);
  strcat(jsonStatus,tempbuf);
  strcat(jsonStatus,", \"pulseCount\":");
  sprintf(tempbuf,"%lu",pulseCount);
  strcat(jsonStatus,tempbuf);
  strcat(jsonStatus,", \"sensorType\":");
  getSensorTypeText(tempbuf);
  strcat(jsonStatus,tempbuf);
  strcat(jsonStatus,", \"clientId\":");
  strcat(jsonStatus,settings.mqttClientId);
  strcat(jsonStatus,", \"Address\":\"");
  strcat(jsonStatus,WiFi.localIP().toString().c_str());
  strcat(jsonStatus,"\"");
  strcat(jsonStatus,"}");
  return jsonStatus;
  }

const char* getVersion()
  {
  return VERSION;
  }

boolean getMqttStatus()
  {
  if (liters==0.0) //happens when status is requested immediately after reboot
  liters=pulseCount/settings.pulsesPerLiter;
  report();

  return true;
  }
