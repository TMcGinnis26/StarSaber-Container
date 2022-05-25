/////
//STAR SABER CONTAINER SOURCE
//By: Tristan Mcginnis
/////
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include "RTClib.h"
#include <math.h>
#include <TinyGPSPlus.h>

#define GPSSerial Serial3
#define TESTLED 9
#define TeamID 1092


//Telemetry Values
float voltage, altitude, temp, pError, GPS_Lat, GPS_Lon, GPS_alt;//voltage draw, altitude, temperature, pointing error
int packets = 0, mh = 0, mm = 0;//Packet Count, Flight State(moved to operation vars), Mission Hours, Mission Minutes 
int GPS_Sats, GPS_hour, GPS_min, CX = 1;//GPS Sattelites, GPS: Hour, GPS:Minutes, TelemetryState; 
float ms, GPS_sec;//Mission seconds + milliseconds
bool tpRelease = false; bool cx = false;//Release state of tethered payload, telemetry state (on/off)
char mode = 'F';//F for flight, S for simulation
String echo = "TMP", curPacket;//Last command with its arguments, no space

//Operation Values
enum states
{
  Stby,
  Asc,
  Desc1,
  Desc2,
  Desc3,
  Grnd
};

float lastAlt, simPres = 0.0, seaLvlPres, curPres;
unsigned int lastReadAlt, lastCPoll = 0, lastTPoll = 0;//last time readingAltitude, sampling Cont. sensors, polling payload 
states prev_state, state;
String cmd,tempString;


Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_BME280 bme;
RTC_PCF8523 rtc;
TinyGPSPlus gps;


//***Recovery Functions***//
bool recover()
{
  //if data is present, recover from the EEPROM, return true
  return false;//return false if doesn't/cant recover
}
  



////////////////////////////////////////
//***Primary Flight State Functions***//
////////////////////////////////////////
void Standby()
{
    //read sensors with sample_sensors()
  
  if(get_altitude() > 3)//switch states when ascended ~3m
  {
    state = Asc;
  }
 return; 
}

void Ascend()
{
    
    //read sensors with sample_sensors()
  
  if(1)//if altitude has decreased
  {
    state = Desc1;
  }
  return;
}

void Stage1Desc()
{
    //read sensors with sample_sensors()
  
  if(get_altitude() <= 400)//if altitude is or < 400m
  {
    //deploy the larger parachute
    state = Desc2;
  }
  return;
}

void Stage2Desc()
{

  //read sensors with sample_sensors()

  if(get_altitude() <= 300)//if altitude is or < 300m
  {
    //lower the tethered payload
    state = Desc3;
  }
  return;
}

void Stage3Desc()
{
    //read sensors with sample_sensors()
    //poll payload with poll_payload()
  
    if(check_landing())//if landed = true
  {
    state = Grnd;
  }
  return;
}

void Grounded()//grounded loop
{
    //buzzer
    //LED
  return;
}



/////////////////////////////////////
//***Secondary Flight Functions***///
/////////////////////////////////////
void I2C_Sensor_Init()
{
    digitalWrite(TESTLED, HIGH);
    //begin sensors
    ina260.begin();
    bme.begin();
    rtc.begin();

    for (int i = 0; i < 5; i++)
        curPres =  bme.readPressure()/100.0;
        delay(5);
    seaLvlPres = curPres;//set sea level at current altitude

    digitalWrite(TESTLED, LOW);
    Serial1.println("System Ready");
    return;
}

void read_serial()
{
  
  return;
}

void sample_sensors()
{
    altitude = get_altitude();
    temp = bme.readTemperature();
    voltage = round(ina260.readBusVoltage()) * .001;

    //Read GPS
    if (GPSSerial.available() > 0) {
        if (gps.encode(GPSSerial.read()))
        {
            GPS_hour = gps.time.hour();
            GPS_min = gps.time.minute();
            GPS_sec = gps.time.second();
            GPS_Lat = round(gps.location.lat() * 10000) * .0001;
            GPS_Lon = round(gps.location.lng() * 10000) * .0001;
            GPS_alt = gps.altitude.meters();
            GPS_Sats = gps.satellites.value();
        }
        else {
            GPS_Lat = 999;
            GPS_Lon = 999;
            GPS_Sats = 999;
        }
    }
    else {
        GPS_Lat = 888;
        GPS_Lon = 888;
        GPS_Sats = 888;
    }
   
    return;
}

void poll_payload()
{
    //poll payload if been 250ms
    return;
}

void downlink_telem()
{
    if ((millis() - lastCPoll) >= 980)//slightly faster than 1 sec
    {
        sample_sensors();
        curPacket += TeamID + ',' + mh + ':' + mm + ':' + String(ms) + ',' + packets + ',' + mode + ',' + tpRelease + ',' + altitude + ',' + temp + ',' + voltage + ',' + GPS_hour + ':' + GPS_min + ':' + GPS_sec + ',' + GPS_Lat + ',' + GPS_Lon + ',' + GPS_Sats + ',' + state + ',' + echo;
        Serial4.println(curPacket);
        if (CX == 1)
        {
            Serial1.println(curPacket);
            packets++;
        }
        curPacket = "";
        lastCPoll = millis();
    }
    else {
        return;
    }
}

bool check_landing()
{
    if (millis() - lastReadAlt >= 1000)//check every 1 sec
    {
        altitude = get_altitude();
        if (altitude < lastAlt +1 && altitude > lastAlt -1)
        {
            return true;
        }
        return false;
    }
    return false;
}

void updateEEPROM()
{
    //save all current data for recovery
    return;
}


float get_altitude()
{
  if(mode == 'S')//if in simulation mode
  {
      return simPres;
  }else
  {
      return bme.readAltitude(seaLvlPres);
  }
}




void setup() {
pinMode(TESTLED, OUTPUT);

  //Start All Sensors
Serial.begin(9600);
Serial1.begin(9600);//ground xbee
Serial2.begin(9600);//payload xbee
Serial4.begin(9600);//Open Log
GPSSerial.begin(9600);

I2C_Sensor_Init();//Init I2C Sensors

Serial1.println("Start Altitude: " + (String)bme.readAltitude(seaLvlPres));

    

  
  /*Temporarily remove the recovery feature
 if(!recover())
 {
  state = Stby; //default standby state
  //sample the barometer to get the sea level pressure
 }
*/


state = Stby;//temporary
//sample barom for sea level pressure
}

void loop() {
  switch(state)
  {
    case Stby:
      Standby();
      break;
    case Asc:
      Ascend();
      break;
    case Desc1:
      Stage1Desc();
      break;
    case Desc2:
      Stage2Desc();
      break;
    case Desc3:
      Stage3Desc();
      break;
    case Grnd:
      Grounded();
      break;
    default:
      while(1)
      {
          Serial1.println("STATE MACHINE ERROR");
      }
  }
  downlink_telem();//send data to ground if time and telem on
  read_serial();//Read incomming serial data (xbees)
  //check for commands after each flight function, parse_packet() if serial recieved

}