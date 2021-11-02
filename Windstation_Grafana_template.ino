#define SIM800L_IP5306_VERSION_20200811

// Select your modem:
#define TINY_GSM_MODEM_SIM800

#include "utilities.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <esp_task_wdt.h>

// Set serial for AT commands (to the module SIM800L)
#define SerialAT Serial1

// Set timer Watchdog
#define WDT_TIMEOUT 300

//Wind Speed Serial
SoftwareSerial Windspeed_Serial;
// Wind Direction Serial
SoftwareSerial Winddirection_Serial;

//Define TX and RX Pin for connnected Wind Speed Sensor
#define TXWindspeed 4
#define RXWindspeed 5
//Define TX and RX Pin for connnected Wind Direction Sensor 
#define TXWinddirection 18
#define RXWinddirection 19

#define SW 15

// Address for calling Data from all sensor
byte request[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39};

float WindSpeed;
float WindSpeed_hr;
String Direction;
float DirectionDegree;
String Time;
String TIME;
byte puretime;
int count = 0;
float adcsum = 0;
float speedsum = 0;
int countwindspeed = 0;

// Define how you're planning to connect to the internet
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// Network Detail
const char apn[]      = "internet-name";
const char gprsUser[] = "userinternet";
const char gprsPass[] = "passwordinternet";

// MQTT details
const char* broker = "broker-name";
const char* topicOut = "publish-topic";
const char* topicIn = "subscribe-topic";

StaticJsonDocument<256> doc;
char message[256];

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// timer function
hw_timer_t * timer = NULL;
volatile bool state = false;

// timer function Read Windspeed
hw_timer_t * timerWindspeed = NULL;
volatile bool stateWindspeed = false;

//ISR Function Send Data
void IRAM_ATTR onTimer(){
   state = !state;
}

//ISR Function Read Windspeed
void IRAM_ATTR _onTimer(){
   stateWindspeed = !stateWindspeed;
}

void setup() {
  
  Serial.begin(115200);
  Serial.setTimeout(50);
  SerialAT.setTimeout(50);
  
  delay(100);
  
  setupModem();

  Serial.println("wait . . .");

//Serial begin Module SIM800L 
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

//Serial begin Wind Speed Sensor
  Windspeed_Serial.begin(9600, SWSERIAL_8N1, RXWindspeed, TXWindspeed);
  
//Serial begin Wind Direction Sensor
  Winddirection_Serial.begin(9600, SWSERIAL_8N1, RXWinddirection, TXWinddirection);
  
  delay(5000);
  Serial.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  while(!modem.waitForNetwork()){
    Serial.println("Fail, Try to connected");
    delay(1000);
    ESP.restart();
  }
  
  Serial.print("Connected to telecom : ");
  Serial.println("Signal Quality: " + String(modem.getSignalQuality()));

//Connected GPRS  
  Serial.println("Connecting to GPRS network.");
  while(!modem.gprsConnect(apn, gprsUser, gprsPass)){
    Serial.println("Fail, Try to connected");
    delay(500);
    continue;
  }
  Serial.println("Connected to GPRS: " + String(apn));
  mqtt.setServer(broker, 8883);
  mqtt.setCallback(mqttCallback);
  Serial.println("Connecting to MQTT Broker: " + String(broker));
  while(mqttConnect()==false){
    Serial.println("Try connected to MQTT Broker !!!");
    continue;
  }
  Serial.println();

  delay(500);
  
  SerialAT.println("AT+CLTS=1");
  SerialAT.flush();
  Serial.println(SerialAT.readString());
  
//Watchdog init
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
//Timer Read Date and Send Data
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 60000000, true);
  timerAlarmEnable(timer);

//Timer Read Date Windspeed
  timerWindspeed = timerBegin(2, 80, true);
  timerAttachInterrupt(timerWindspeed, &_onTimer, true);
  timerAlarmWrite(timerWindspeed, 5000000, true);
  timerAlarmEnable(timerWindspeed);
  
}

void loop() {

//  Restart ESP32 when ESP32 works about 1 hr.
  if(millis() == 14400000){
    ESP.restart();
  }

  int stateswitch = analogRead(SW);
  float _stateswitch = stateswitch * 3.3 / 4095.0;

  if(_stateswitch >= 0.6){
    delay(5000);
    Serial.println("Config Mode");

    WindDirection();
    
    //Check Status of GPRS
      while(!modem.isGprsConnected()){
        Serial.println("Try Connect GPRS");
        modem.gprsConnect(apn, gprsUser, gprsPass);
        continue;
      }
        
      //Check status of mqtt
      if(!mqtt.connected()){
        while(mqttConnect() == false) continue;
      }
        
      //Check status of mqtt
      if(mqtt.connected()){
        mqtt.loop();
      }

    doc["WDD2"] = DirectionDegree;
    serializeJson(doc, message);
    Serial.print("MQTT pub = ");
    Serial.println(mqtt.publish(topicOut, message, sizeof(message)));
    TIME = "";
    doc.clear();
    mqtt.disconnect();
 
  }
  
  if(state == true && _stateswitch == 0){

//    Call Function Read Wind Direction
      WindDirection();
    
//    AT Command call Time with SIM800L 
      SerialAT.println("AT+CCLK?");
      SerialAT.flush();
      Time = SerialAT.readString();
      Time.trim();
      if(Time.startsWith("+CCLK")){
        for(byte i = 8; i < Time.length(); i++){
          if(Time[i] == '+'){
            break;
          }
          TIME += Time[i];
        }
      }

//    Convert value of signal to RSSI
      int value_signal = modem.getSignalQuality();
      int RSSI_GPRS = (value_signal + (value_signal - 1)) - 112;

//    Add member to document before serialize 
      WindSpeed = (speedsum / countwindspeed) / 10.0;
      doc["T2"] = TIME;
      doc["W2"] = WindSpeed;
      doc["WHR2"] = WindSpeed * 3.6;
      doc["WD2"] = Direction;
      doc["WDD2"] = DirectionDegree;
      doc["lat2"] = 17.92844;
      doc["long2"] = 102.79028;
      doc["S2"] = RSSI_GPRS;
      doc["batt2"] = ((adcsum / count) * 4.2) / 4230;
      
      serializeJson(doc, message);
      
      //Check Status of GPRS
      while(!modem.isGprsConnected()){
        Serial.println("Try Connect GPRS");
        modem.gprsConnect(apn, gprsUser, gprsPass);
        continue;
      }
        
      //Check status of mqtt
      if(!mqtt.connected()){
        while(mqttConnect() == false) continue;
      }
        
      //Check status of mqtt
      if(mqtt.connected()){
        mqtt.loop();
      }

  //MQTT and GPRS is connected
    if(modem.isGprsConnected() && mqtt.connected() && TIME != ""){
      Serial.print("MQTT pub = ");
      Serial.print(mqtt.publish(topicOut, message, sizeof(message)));
      Serial.print(" MQTT con = ");
      Serial.print(mqtt.connected());
      Serial.print(" MQTT st = ");
      Serial.print(mqtt.state());
      Serial.print(" GPRS = ");
      Serial.println(modem.isGprsConnected());
      Serial.println(message);
      
//    flip state and wait 1 minute for read and send data again
      state = !state;
      
//    Reset variable "TIME" before pack time for send data and time of read data
      TIME = "";

//    Clear StaticJSONDocument for Pack new data before send data again
      doc.clear();

//    Clear windspeed and battery data
      count = 0;
      adcsum = 0;
      speedsum = 0;
      countwindspeed = 0;

      mqtt.disconnect();

    }
    
  }
  
  // stage read data
  if(stateWindspeed == true){
    
  //    Call Function Read Wind Speed
      windSpeed_Battery();
      
      stateWindspeed = !stateWindspeed;
  }
  
//  watchdog reset
    esp_task_wdt_reset();
    
}

boolean mqttConnect()
{
  if(!mqtt.connect("windstation_2"))
  {
    Serial.print(".");
    return false;
  }
  mqtt.subscribe(topicIn);
  return mqtt.connected();
}

void mqttCallback(char* topic, byte* payload, unsigned int len)
{
  Serial.print("Message receive: ");
  Serial.write(payload, len);
  Serial.println();
}

//Function Read Windspeed
void windSpeed_Battery(){
  
   //Wind Speed Coding
      Windspeed_Serial.write(request, sizeof(request));
      Windspeed_Serial.flush();
  
      byte windspeeddatacallback[8];
      Windspeed_Serial.readBytes(windspeeddatacallback, 8);
  
//      Serial.print("Wind Calculation : ");
      int WindSpeedData = (windspeeddatacallback[3] + windspeeddatacallback[4]);
      speedsum += WindSpeedData;
      countwindspeed++;
      
//      Wind speed unit m/s
//      WindSpeed = WindSpeedData / 10.0;
//      Wind speed unit km/hr
//      WindSpeed_hr = WindSpeed * 3.6;
//      Serial.print(WindSpeed);
//      Serial.println(" m/s");
//      Serial.println(WindSpeed_hr);
//      Serial.println(" km/hr");

//    Battery
      int adc = analogRead(33) + 938;
      adcsum += adc;
      count++;
      
}

//Function Read Winddirection
void WindDirection(){

  //Wind Direction Coding
      Winddirection_Serial.write(request, sizeof(request));
      Winddirection_Serial.flush();
  
      byte winddirectiondatacallback[8];
      Winddirection_Serial.readBytes(winddirectiondatacallback, 8);
  
//      Serial.print("Wind Direction : ");
      int WinddirectionDEC = (winddirectiondatacallback[3] + winddirectiondatacallback[4]);
//      Serial.print(WinddirectionDEC, DEC);
//      Serial.print(" : ");
  
      switch(WinddirectionDEC){
        case 0:
//          Direction = "North";
          Direction = "N";
          DirectionDegree = WinddirectionDEC * 22.5;
        break;
        case 1:
//          Direction = "Northeast by north";
          Direction = "NNE";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 2:
//          Direction = "Northeast";
          Direction = "NE";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 3:
//          Direction = "Northeast by east";
          Direction = "ENE";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 4:
//          Direction = "East";
          Direction = "E";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 5:
//          Direction = "Southeast by east";
          Direction = "ESE";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 6:
//          Direction = "Southeast";
          Direction = "SE";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 7:
//          Direction = "Southeast by south";
          Direction = "SSE";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 8:
//          Direction = "South";
          Direction = "S";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 9:
//          Direction = "Southwest by south";
          Direction = "SSW";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 10:
//          Direction = "Southwest";
          Direction = "SW";
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 11:
//          Direction = "Southwest by west";    
          Direction = "WSW";
          DirectionDegree = WinddirectionDEC * 22.5; 
          break;
        case 12:
//          Direction = "West";
          Direction = "W";     
          DirectionDegree = WinddirectionDEC * 22.5;
          break;
        case 13:
//          Direction = "Northwest by west";
          Direction = "WNW";
          DirectionDegree = WinddirectionDEC * 22.5;     
          break;
        case 14:
//          Direction = "Northwest";
          Direction = "NW";
          DirectionDegree = WinddirectionDEC * 22.5;     
          break;
        default:
//          Direction = "Northwest by north";
          Direction = "NNW";
          DirectionDegree = WinddirectionDEC * 22.5;     
          break;
      }

//      Serial.println(Direction);

}
