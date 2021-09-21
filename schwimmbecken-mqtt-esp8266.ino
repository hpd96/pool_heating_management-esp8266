/*
 * temperatur-Messung mit ESP8266 und DS18b18
 * 
 * features:
 * http web page
 * schwimmbecken & solarmatte -> mqtt
 * 
 * hpd
 * 
 * 
 * 
 * 
 * GNU General Public License v2.0
 * 
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <stdio.h>
#include <stdlib.h>
#include <ESP8266HTTPClient.h>

#include "timerclass.h"

#include <OneWire.h>
#include <DallasTemperature.h>

// Uses Ticker to do a nonblocking loop every 1 second to check status
#include <Ticker.h>

#define TEMP_DELTA_MEHR_AN      4.5
#define TEMP_DELTA_WENIGER_AUS  2.5

#define TEMP_MATTEN_MIN_PUMPEN 20.0
#define TEMP_BECKEN_MAX_PUMPEN 38.0

#define MIN_FILTERUNG_PRO_TAG 120                      // 120 [min]


#define UHRZEIT_PUMPE_START_MINS ( 9 * 60 + 30 )
#define UHRZEIT_PUMPE_ENDE_MINS  ( 18 * 60 + 30 )

// #define UHRZEIT_PUMPE_START_MINS ( 23 * 60 + 41 )
// #define UHRZEIT_PUMPE_ENDE_MINS  ( 23 * 60 + 42 )

#define NACHLAUF_UV_LAMPE_MIN 30                       // [min]


// STM ==========================================================
#include <StateMachine.h>

const int STATE_DELAY = 2000;


Timer myUVLampeTimerOFF;

StateMachine machine = StateMachine();

State* S0 = machine.addState(&state0); 
State* S1 = machine.addState(&state1);
State* S2 = machine.addState(&state2);
State* S3 = machine.addState(&state3);
State* S4 = machine.addState(&state4);


String sListStates[] = { "INIT","AUS","FILTERN","WÄRMEN", "DEAKTIV" };


// STM ==========================================================

unsigned int iSekundenFilterung=0;

unsigned long lastmillis;

const char* sVersion = "0.9.95";

/* Configuration of NTP */
#define MY_NTP_SERVER "de.pool.ntp.org"           
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"   

#include <time.h>                   // time() ctime()

char sNTPUhrzeit[50];


#include "my-defines.h"

//Your Wifi SSID
const char* ssid = WIFI_SSID;
//Your Wifi Key
const char* password = WIFI_PASSWORD;


#define REQUIRESNEW false
#define REQUIRESALARMS false

#define PLAUSI_MIN 4.0
#define PLAUSI_MAX 80

#include <PubSubClient.h>
//Your MQTT Broker
const char* mqtt_server = MQTT_SERVER;    // name address for MQTT broker (using DNS)

const char* mqtt_topic_temp_becken = "garten/schwimmbecken/temp_becken";
const char* mqtt_topic_temp_matten = "garten/schwimmbecken/temp_matten";
const char* mqtt_topic_temp_delta = "garten/schwimmbecken/temp_delta";

const char* mqtt_topic_min_filter = "garten/schwimmbecken/minuten_filterung";

const char* mqtt_topic_text = "garten/schwimmbecken/meldung";


// MQTT client is also host name for WIFI
const char* mqtt_client_id = "schwimmbecken";

const char* mqtt_topic_cmd = "garten/schwimmbecken/cmd";
const char* mqtt_topic_status = "garten/schwimmbecken/status";

const char* mqtt_topic_version = "garten/schwimmbecken/version";
const char* mqtt_topic_wifi = "garten/schwimmbecken/wifi";
const char* mqtt_topic_time = "garten/schwimmbecken/time";


//28FE9C2918200679   5m  solar matten
#define DS18_ADDRESS_SOLAR_MATS 0x28, 0xfe, 0x9c, 0x29, 0x18, 0x20, 0x06, 0x79
//28CD051D070000C6   15m   becken
#define DS18_ADDRESS_POOL_SIDE 0x28, 0xcd, 0x05, 0x1d, 0x07, 0x00, 0x00, 0xc6



const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";

// ############################################################################################################

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);


#include <OneWire.h>
#include <DallasTemperature.h>


DeviceAddress pool_sensor_matten = { DS18_ADDRESS_SOLAR_MATS };
DeviceAddress pool_sensor_becken = { DS18_ADDRESS_POOL_SIDE  };



bool SendUpdate = true;

Ticker ticker1;
Ticker ticker2;

int bSensorCheckOK = false;
int bTempDeltaValid = false;
float  fTempBecken, fTempMatten, fTempDelta;  

// Data wire is plugged into pin D1 on the ESP8266 = GPIO5
#define ONE_WIRE_BUS 5

// D2 defekt!
//// Data wire is plugged into pin D2 on the ESP8266 = GPIO4
//#define ONE_WIRE_BUS 4
 
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// https://werner.rothschopf.net/201802_arduino_esp8266_ntp.htm

uint32_t sntp_startup_delay_MS_rfc_not_less_than_60000 ()
{
  return 30 * 1000UL; // 30 sec
}

uint32_t sntp_update_delay_MS_rfc_not_less_than_15000 ()
{
  return 6 * 60 * 60 * 1000UL; // 6 hours
}


int getTimeOfDayMinutes() 
{
  time_t now;                         // this is the epoch
  tm tm;                              // the structure tm holds time information in a more convient way
  time(&now);                       // read the current time
  localtime_r(&now, &tm);           // update the structure tm with the current time
  return tm.tm_hour * 60 + tm.tm_min;
}


bool isNTPtimeSet()
{
  time_t now;                         // this is the epoch
  tm tm;                              // the structure tm holds time information in a more convient way
  time(&now);                       // read the current time
  localtime_r(&now, &tm);           // update the structure tm with the current time
  if ( 1970 < (tm.tm_year + 1900) ) return true;

  Serial.println("NTP internet date not set by now...");
  mqtt_client.publish(mqtt_topic_text, "NTP internet date not set by now...");
  return false;
}


void showTime() 
{
  time_t now;                         // this is the epoch
  tm tm;                              // the structure tm holds time information in a more convient way

  time(&now);                       // read the current time
  localtime_r(&now, &tm);           // update the structure tm with the current time
  Serial.print("year:");
  Serial.print(tm.tm_year + 1900);  // years since 1900
  Serial.print("\tmonth:");
  Serial.print(tm.tm_mon + 1);      // January = 0 (!)
  Serial.print("\tday:");
  Serial.print(tm.tm_mday);         // day of month
  Serial.print("\thour:");
  Serial.print(tm.tm_hour);         // hours since midnight  0-23
  Serial.print("\tmin:");
  Serial.print(tm.tm_min);          // minutes after the hour  0-59
  Serial.print("\tsec:");
  Serial.print(tm.tm_sec);          // seconds after the minute  0-61*
  Serial.print("\twday");
  Serial.print(tm.tm_wday);         // days since Sunday 0-6
  if (tm.tm_isdst == 1)             // Daylight Saving Time flag
    Serial.print("\tDST");
  else
    Serial.print("\tstandard");
  Serial.println();

  sprintf(sNTPUhrzeit, "Uhrzeit = %02d:%02d", tm.tm_hour, tm.tm_min );
  mqtt_client.publish(mqtt_topic_time, sNTPUhrzeit);

    // tageswechesel
    if ( ( tm.tm_min <= 1 ) && ( tm.tm_hour == 0 ) )
    {
      Serial.print("Tageswechsel: Minuten Filterung auf null");
      mqtt_client.publish(mqtt_topic_text, "Tageswechsel: Minuten Filterung auf null");
      iSekundenFilterung = 0;
    }

}



void sendVersionInfo()
{
    mqtt_client.publish(mqtt_topic_version, sVersion);
    long lRssi = WiFi.RSSI();
    int iQuality=0;
    char sBuffer[50];
    // sprintf(sBuffer, "RSSI = %d dBm", lRssi);
    
    // dBm to Quality:
    if(lRssi <= -100)
        iQuality = 0;
    else if(lRssi >= -50)
        iQuality = 100;
    else
        iQuality = 2 * (lRssi + 100);
        
    sprintf(sBuffer, "%d", iQuality);
    mqtt_client.publish(mqtt_topic_wifi, sBuffer );
}



void sendStatusInfo()
{
  char sStatus[15];
  
  sListStates[ machine.currentState ].toCharArray(sStatus, sizeof(sStatus)-1 );
  mqtt_client.publish(mqtt_topic_status, sStatus);
  Serial.print(" status ");
  Serial.println( sStatus );

  showTime();  
}



void MqttCallback(char* topic, byte* payload, unsigned int length) 
{
  char sStatus[15];
  
  payload[length] = 0;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if ( strstr( (const char *)payload, "version") != NULL) {
    sendVersionInfo();
  }
  if ( strstr( (const char *)payload, "status") != NULL) {
    sendStatusInfo();
  }
}



void MqttReconnect() {
  while (!mqtt_client.connected()) {
    Serial.print("Connect to MQTT Broker "); Serial.println( mqtt_server );
    delay(1000);
    if (mqtt_client.connect(mqtt_client_id)) {
      Serial.print("connected: topic ");  Serial.println( mqtt_topic_cmd );

      // Once connected, publish an announcement...
      sendVersionInfo();

      // ... and resubscribe
      mqtt_client.subscribe(mqtt_topic_cmd);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds...");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  Serial.println(" ok...");
}




//=======================================

//Your Domain name with URL path or IP address with path
const char* serverNamePumpe = "http://conil.fritz.box:8080/rest/items/StromGartenSchwimmbeckenPumpe";
const char* serverName3Hahn = "http://conil.fritz.box:8080/rest/items/StromGartenSchwimmbecken3Hahn";
const char* serverNameUVFilter = "http://conil.fritz.box:8080/rest/items/StromGartenSchwimmbeckenUVFilter";

//=======================================

void schalteUVFilter(int iStatus)
{
  if (iStatus == 1)
  {
    schalteOpenhab( iStatus, serverNameUVFilter );
    myUVLampeTimerOFF.unset();
  }
  else
  {
    myUVLampeTimerOFF.set();
         mqtt_client.publish(mqtt_topic_text, "timer: UV lampe start 6min");
  }
}


void schalte3Hahn(int iStatus)
{
  schalteOpenhab( iStatus, serverName3Hahn );  
}


void schaltePumpe(int iStatus)
{
  schalteOpenhab( iStatus, serverNamePumpe );
}

// $ curl -X GET --header "Content-Type: text/plain"  "http://conil:8080/rest/items/StromGartenSchwimmbeckenPumpe/state"
// OFF

void schalteOpenhab(int iStatus, const char* serverName )
{

char cstr[16];

      HTTPClient http;
     int httpResponseCode;
     
      // Your Domain name with URL path or IP address with path
      http.begin(espClient, serverName);

      // If you need an HTTP request with a content type: text/plain
      http.addHeader("Content-Type", "text/plain");
      // http.addHeader("Accept:", "application/json");
//      String httpRequestData = iStatus == 1 ? "ON" : "OFF" ;
//      int httpResponseCode = http.POST( httpRequestData );
       if ( iStatus == 1 ) 
         httpResponseCode = http.POST( "ON" );
       else
         httpResponseCode = http.POST( "OFF" );
      
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      
      http.end();


}


//=======================================





void state0(){
  Serial.println("State 0 INIT");
  if(machine.executeOnce){
     Serial.println("Execute Once"); 
      Serial.println("PUMPE AUS");
      schaltePumpe(0);
        schalte3Hahn(0);
       schalteUVFilter(0);
      sendStatusInfo();
  }
}


bool transitionS0S4(){
  // if shelly erreichbar
  // if PV hat strom
  // if pumpe enabled
  if ( (bSensorCheckOK == true ) && ( bTempDeltaValid == true ) && ( isNTPtimeSet() == true ) )
  return true;
  else
  return false;
  
}


void state1()
{
  Serial.println("State AUS");
  if(machine.executeOnce)
    {
            mqtt_client.publish(mqtt_topic_text, "Pumpzeit!");
      Serial.println("PUMPE AUS");
      schaltePumpe(0);
      schalte3Hahn(0);
      schalteUVFilter(0);
      sendStatusInfo();
    }
}

bool transitionS1S2()
{
  if ( iSekundenFilterung / 60 < MIN_FILTERUNG_PRO_TAG )  // x min Filterung pro Tag
  {
    return true;
  }
  else
  {
    return false;
  }
}


// transition zu SOLAR-MATTEN Pumpen ?
bool transitionS1S3()
{
  if ( ( fTempDelta >= TEMP_DELTA_MEHR_AN ) && ( fTempMatten >= TEMP_MATTEN_MIN_PUMPEN  )  )
    return true;
  else
    return false;

}

//-------------------------
void state2(){
  Serial.println("State 2 PUMPEN_FILTER");
  if(machine.executeOnce){
    schaltePumpe(1);
    schalte3Hahn(0);
    schalteUVFilter(1);
    sendStatusInfo();
  }
  iSekundenFilterung += (STATE_DELAY/1000);
}

bool transitionS2S1(){

  if ( iSekundenFilterung / 60 >= MIN_FILTERUNG_PRO_TAG )  // x min Filterung pro Tag
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

// transition zu SOLAR-MATTEN Pumpen ?
bool transitionS2S3(){
  if ( ( fTempDelta >= TEMP_DELTA_MEHR_AN ) && ( fTempMatten >= TEMP_MATTEN_MIN_PUMPEN  )  )
  return true;
  else
  return false;
}


//------------------------
void state3(){
  Serial.println("State 3 PUMPE_WAERME");
  if(machine.executeOnce){
    Serial.println("PUMPE AN");  
  schaltePumpe(1);
    schalte3Hahn(1);
    schalteUVFilter(1);
  sendStatusInfo();
  }
  iSekundenFilterung += (STATE_DELAY/1000);

}

bool transitionS3S1(){
  if (  ( fTempDelta < TEMP_DELTA_WENIGER_AUS ) ||  ( fTempBecken > TEMP_BECKEN_MAX_PUMPEN ) )
  return true;
  else
  return false;
}



//------------------------
void state4(){
  Serial.println("State 4 deaktiv");
  if(machine.executeOnce){
      mqtt_client.publish(mqtt_topic_text, "außerhalb Pumpzeit 9:30h-18:30h!");
         schaltePumpe(0);
       schalte3Hahn(0);
       schalteUVFilter(0);
      sendStatusInfo();
  }
}

// true = außerhalb
bool testZeitAusserhalbPumpen()
{
  if  ( ( getTimeOfDayMinutes() < UHRZEIT_PUMPE_START_MINS )  ||  ( getTimeOfDayMinutes() > UHRZEIT_PUMPE_ENDE_MINS ) )
    return true;
  else
    return false;
}

bool transitionS4S1(){
  // Zeit mit PV?
    return ! testZeitAusserhalbPumpen();
  }
  
bool transitionS1S4(){
  // Zeit mit PV?
    return testZeitAusserhalbPumpen();
  }

  
//=======================================


// the setup function runs once when you press reset or power the board
void setup(void) {

  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // start serial port
  Serial.begin(115200);
  while(!Serial) { ; }
  Serial.println("    Dallas Temperature One Wire DS18b18 -> MQTT");
  Serial.println();
  Serial.println(sVersion);
  Serial.println();

  lastmillis = millis();
  

  
  // Start up the library
  sensors.begin();
  sensors.setResolution(12);
    sensors.setWaitForConversion(true);        // makes it async
  delay(250);

// https://werner.rothschopf.net/201802_arduino_esp8266_ntp.htm
  configTime(MY_TZ, MY_NTP_SERVER); // --> Here is the IMPORTANT ONE LINER needed in your sketch!


//  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);


  while(WiFi.waitForConnectResult() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.println("WiFi failed, retrying.");
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  MDNS.begin(mqtt_client_id);

  httpUpdater.setup(&httpServer, update_path, update_username, update_password);

  httpServer.begin();

  MDNS.addService("http", "tcp", 80);

  Serial.printf("HTTPUpdateServer ready! \n  Open http://%s.local%s in your browser and\n login with username '%s' and password '%s'\n", mqtt_client_id, update_path, update_username, update_password);
  Serial.printf("\nSketch version: %s\n", sVersion);


  mqtt_client.setServer(mqtt_server, MQTT_SERVER_PORT);
  mqtt_client.setCallback(MqttCallback);
  MqttReconnect();

  mqtt_client.publish(mqtt_topic_text, "starting...");
  
  showInfo();


  httpServer.on("/", []() {
      char sStatus[20];
      char sBuffer[100];
      sListStates[ machine.currentState ].toCharArray(sStatus, sizeof(sStatus)-1 );
     // sprintf(sBuffer, "schwimmbecken status ist %s \r\ntemp delta %.1f\r\n<p>!!!</p>", sStatus, mqtt_topic_temp_delta);
    //  httpServer.send(200, "text/html", sBuffer );
    httpServer.send(200, "text/html", "test" );
      httpServer.send ( 302, "text/plain", "");
  });


  httpServer.on("/status", []() {
    long lRssi = WiFi.RSSI();
    char sBuffer[100];
    sprintf(sBuffer, "WiFi  RSSI = %d dBm\r\nVersion = %s\r\r\r\n Zeit %s \r\n", lRssi, sVersion, sNTPUhrzeit);
    Serial.println( sBuffer );

    httpServer.send(200, "text/plain", sBuffer);
 
    httpServer.send ( 302, "text/plain", "");
  });



  httpServer.on("/info", []() {
    char sBuffer1[300],sBuffer2[100],sBuffer3[250],sBuffer[100];
    DeviceAddress Thermometer;
    int i;
    float fTemp=-96;

    // report parasite power requirements
    sprintf(sBuffer1, "Parasite power is: %s\r\n", (sensors.isParasitePowerMode()) ? "ON" : "OFF");
    sprintf(sBuffer2, "\r\nFound %d devices\r\n", sensors.getDeviceCount() );

    strcpy(sBuffer3,"");
    for(i=0; i<sensors.getDeviceCount(); i++)
    {
      sensors.getAddress(Thermometer, i);
      sprintf(sBuffer, "%d: adr %02X %02X %02X %02X %02X %02X %02X %02X res: %d \r\n", i, Thermometer[0], Thermometer[1], Thermometer[2], Thermometer[3], Thermometer[4], Thermometer[5], Thermometer[6], Thermometer[7],
         sensors.getResolution(Thermometer));
      strcat(sBuffer3, sBuffer);

      fTemp = sensors.getTempCByIndex(i);
      sprintf( sBuffer, " =: %.2f gradC\r\n", fTemp);
      strcat(sBuffer3, sBuffer);

    }
    strcat (sBuffer1, sBuffer2);
    strcat (sBuffer1, sBuffer3);
    httpServer.send(200, "text/plain", sBuffer1);
    httpServer.send ( 302, "text/plain", "");
    Serial.printf( sBuffer1 );

  });



 // blinking start
  for ( int iLoop=0; iLoop<12; iLoop++) {
    digitalWrite(LED_BUILTIN, HIGH );
    delay(250);
    digitalWrite(LED_BUILTIN, LOW );
    delay(250);
  }

  myUVLampeTimerOFF.set_max_delay( NACHLAUF_UV_LAMPE_MIN * 60000UL ); // 16 x  1 minute   
  ticker1.attach(15*60, sendVersionInfo);
  ticker2.attach(60, sendStatusInfo);


  S0->addTransition(&transitionS0S4,S4);    // Transition init->deaktiv
  
  S1->addTransition(&transitionS1S2,S2);  // S1 transition to S2
  S1->addTransition(&transitionS1S3,S3);  // S1 transition to S3

  S2->addTransition(&transitionS2S3,S3);  // S2 transition to S3
  S2->addTransition(&transitionS2S1,S1);  // S2 transition to S1

  S3->addTransition(&transitionS3S1,S1);  // S3 transition to S1

// pumpzeit
  S1->addTransition(&transitionS1S4,S4);  // S1 transition to S4
  S2->addTransition(&transitionS1S4,S4);  // S2 transition to S4
  S3->addTransition(&transitionS1S4,S4);  // S3 transition to S4
  S4->addTransition(&transitionS4S1,S1);  // S4 transition to S1

  Serial.println(machine.currentState);
  
  machine.transitionTo(S0);
  Serial.println(machine.currentState);
  sendStatusInfo();
}



// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}




void showInfo(void)
{
  uint8_t i;
  // locate devices on the bus
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements

  DeviceAddress insideThermometer={ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }, outsideThermometer={ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  if (!sensors.getAddress(outsideThermometer, 1)) Serial.println("Unable to find address for Device 1"); 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();


  Serial.println("found:");
  printAddress(insideThermometer);
  printAddress(outsideThermometer);

  Serial.println("wanted:");
  printAddress( pool_sensor_matten);
  printAddress(pool_sensor_becken);

  if ( sensors.getDeviceCount() != 2 ) {
    Serial.println("###### nicht 2 Sensoren ! ################"); 
      mqtt_client.publish(mqtt_topic_text, "nicht 2 Temp Sensoren");
    bSensorCheckOK=false;
  }
  else
  {
    // compare 2 addresses
    if ( insideThermometer[0] == pool_sensor_matten[0] && 
    insideThermometer[1] == pool_sensor_matten[1] && 
    insideThermometer[2] == pool_sensor_matten[2] && 
    insideThermometer[3] == pool_sensor_matten[3] && 
    insideThermometer[4] == pool_sensor_matten[4] && 
    insideThermometer[5] == pool_sensor_matten[5] && 
    insideThermometer[6] == pool_sensor_matten[6] && 
    insideThermometer[7] == pool_sensor_matten[7] )
    {
      Serial.println("sensor matte ok"); 
      mqtt_client.publish(mqtt_topic_text, "sensor matte OK");

    // compare 2 addresses
    if ( outsideThermometer[0] == pool_sensor_becken[0] && 
    outsideThermometer[1] == pool_sensor_becken[1] && 
    outsideThermometer[2] == pool_sensor_becken[2] && 
    outsideThermometer[3] == pool_sensor_becken[3] && 
    outsideThermometer[4] == pool_sensor_becken[4] && 
    outsideThermometer[5] == pool_sensor_becken[5] && 
    outsideThermometer[6] == pool_sensor_becken[6] && 
    outsideThermometer[7] == pool_sensor_becken[7] )
    {
      Serial.println("sensor matte & becken ok"); 
      mqtt_client.publish(mqtt_topic_text, "sensor matte & becken OK");  
      bSensorCheckOK=true;
      }
      else
      { 
        Serial.println("sensor becken falsch"); 
        mqtt_client.publish(mqtt_topic_text, "sensor becken falsch");
        bSensorCheckOK=false;
      }
    
    
    }  // matte 
    else
    { 
      Serial.println("sensor matte falsch"); 
      mqtt_client.publish(mqtt_topic_text, "sensor matte falsch");
      bSensorCheckOK=false;
    }
  } // 2 sensoren ?
}

//void DallasTemperature::shutdown(void) {
//    _wire->depower();
//}



void loop(void) {
  httpServer.handleClient();

  char sBUFFER[20];

  if (!mqtt_client.connected()) { MqttReconnect(); }
  mqtt_client.loop();


  if (millis() - lastmillis >  30 * 1000) {
    Serial.print(" running since ");
    Serial.print(millis()/60000);
    Serial.println(" minutes");
    lastmillis = millis();
    SendUpdate = true;
  }
  
  
  if (SendUpdate)
  {
    Serial.print("mqtt status update: ");

    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("\n Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    Serial.println("DONE");

    float fTemp;

    itoa(iSekundenFilterung / 60, sBUFFER, 10);
    mqtt_client.publish( mqtt_topic_min_filter, sBUFFER );

    int bTempValid=false;

      sensors.requestTemperatures();
      fTempDelta = -96;
     
      fTemp = sensors.getTempC(pool_sensor_becken);
      Serial.print(mqtt_topic_temp_becken);
      Serial.print(" ");
      Serial.print(fTemp);
      Serial.print(" °C\n");
      if ( (fTemp >= PLAUSI_MIN) && (fTemp <= PLAUSI_MAX)  &&  (fTemp != -127.00) )
      {
        mqtt_client.publish( mqtt_topic_temp_becken, dtostrf(fTemp, 3, 1, sBUFFER) );
        fTempBecken=fTemp;
        bTempValid=true; 
      }
      else
      { Serial.print("pool_sensor_becken out of range!\n"); bTempValid=false; 
        mqtt_client.publish(mqtt_topic_text, "pool_sensor_becken out of range!");
      }


      fTemp = sensors.getTempC(pool_sensor_matten);
      Serial.print(mqtt_topic_temp_matten);
      Serial.print(" ");
      Serial.print(fTemp);
      Serial.print(" °C\n");
      if ( (fTemp >= PLAUSI_MIN) && (fTemp <= PLAUSI_MAX)  &&  (fTemp != -127.00) )
      {
        mqtt_client.publish( mqtt_topic_temp_matten, dtostrf(fTemp, 3, 1, sBUFFER) );
        fTempMatten=fTemp;
        if ( bTempValid == true )
        {  
          fTempDelta = fTempMatten - fTempBecken;
          mqtt_client.publish( mqtt_topic_temp_delta, dtostrf(fTempDelta, 3, 1, sBUFFER) ); 
          bTempDeltaValid = true;
        }
      }
      else
      {
        Serial.print("pool_sensor_matten out of range!\n");  bTempValid=false;
        mqtt_client.publish(mqtt_topic_text, "pool_sensor_matten out of range!");
      }


      oneWire.depower(); //???

      SendUpdate = false;
    }

    if (myUVLampeTimerOFF.check())
    {
      mqtt_client.publish(mqtt_topic_text, "timer: UV lampe OFF!");
      schalteOpenhab( 0, serverNameUVFilter );
      Serial.print("timer: UV lampe OFF  ");
      myUVLampeTimerOFF.unset();
    }
  

  machine.run();
  delay(STATE_DELAY);
  
}
