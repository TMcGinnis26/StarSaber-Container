/////
//STAR SABER CONTAINER SOURCE
//By: Tristan Mcginnis
/////
//#include <Adafruit_BME280.h>
#include "Adafruit_BMP3XX.h"
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



//Telemetry Values
float voltage, altitude, temp, pError, GPS_Lat, GPS_Lon, GPS_alt;//voltage draw, altitude, temperature, pointing error
int packets = 1, mish = 0, mismin = 0;//Packet Count, Flight State(moved to operation vars), Mission Hours, Mission Minutes 
int GPS_Sats, GPS_hour, GPS_min, CX = 1;//GPS Sattelites, GPS: Hour, GPS:Minutes, TelemetryState; 
float missec = 0.0, GPS_sec;//Mission seconds + milliseconds
bool cx = false;//Release state of tethered payload, telemetry state (on/off)
char mode = 'F';//F for flight, S for simulation
String echo = "TMP", curPacket = "";//Last command with its arguments, no space
int TeamID = 1092;
bool ledstat = false;
char tpRelease = 'N';

//Operation Values
enum states
{
    Stby = 0,
    Asc = 1,
    Desc1 = 2,
    Desc2 = 3,
    Desc3 = 4,
    Grnd = 5
};

float lastAlt = -5.0, simPres = 0.0, seaLvlPres, curPres;
unsigned int lastReadAlt, lastCPoll = 0, lastTPoll = 0, lastSampleTime, RTCStartMillis, lastBlink;//last time readingAltitude, sampling Cont. sensors, polling payload 
states prev_state, state;
String cmd, tempString;
DateTime now;


Adafruit_INA260 ina260 = Adafruit_INA260();
//Adafruit_BME280 bme;
Adafruit_BMP3XX bmp;
RTC_PCF8523 rtc;
TinyGPSPlus gps;



//***Recovery Functions***//
bool recover()
{
    //if data is present, recover from the EEPROM, return true
    return false;//return false if doesn't/cant recover
}








/////////////////////////////////////
//***Secondary Flight Functions***///
/////////////////////////////////////
void I2C_Sensor_Init()
{
    rtc.begin();
    rtc.adjust(DateTime(2022,1,1,0,0,0));
    RTCStartMillis = millis();
    digitalWrite(TESTLED, HIGH);
    //begin sensors
    ina260.begin();
    //bme.begin();
    bmp.begin_I2C();
    if (!rtc.begin())
    {
        Serial1.println("RTC NOT STARTED");
    }
    Serial1.println("RTC STARTED");

    for (int i = 0; i < 5; i++)
        bmp.performReading();
        curPres = bmp.pressure / 100.0;
        altitude = bmp.readAltitude(curPres);
        delay(10);
    seaLvlPres = curPres;//set sea level at current altitude
    altitude = get_altitude();

    digitalWrite(TESTLED, LOW);
    Serial1.println("System Ready");
    return;
}

void read_serial()
{
    if (Serial1.available() > 0)
    {
        while (Serial1.available() > 0)
        {
            cmd += Serial1.readString();
        }
        if (cmd.substring(0,2) == "ST")
        {
            rtc.begin();
            Serial1.println(cmd);
            
            String Hrs = cmd.substring(2,cmd.indexOf(":"));
            int HrsI = Hrs.toInt();
            Serial1.println(String(HrsI));
            rtc.adjust(DateTime(2022,1,1, HrsI, 30, 30));
            //rtc.adjust(DateTime("Jul 21 2015", "02:44:20"));
            now = rtc.now();
            Serial1.println("SetTime: "+String(now.hour()) + ':' + String(now.minute())+':'+String(now.second()));
        }
        
    }
    return;
}

void sample_sensors()
{
    if (millis() - lastSampleTime >= 50)
    {
        now = rtc.now();
        bmp.performReading();
    
    altitude = get_altitude();
    temp = bmp.temperature;
    voltage = ina260.readBusVoltage()*0.001;
    mish = now.hour();
    mismin = now.minute();
    missec = now.second();

    

    if (ledstat)
    {
        digitalWrite(TESTLED, LOW);
        ledstat = false;
    }
    else
    {
        digitalWrite(TESTLED, HIGH);
        ledstat = true;
    }

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
    lastSampleTime = millis();
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
        //Serial1.println(String(get_altitude()));
        curPacket = String(String(TeamID) + ',' + mish + ':' + mismin + ':' + String(missec) + ',' + packets + ",C," + mode + ',' + tpRelease + ',' + String(altitude) + ',' + temp + ',' + voltage + ',' + GPS_hour + ':' + GPS_min + ':' + GPS_sec + ',' + GPS_Lat + ',' + GPS_Lon + ',' + GPS_alt + ',' + GPS_Sats + ',' + String(state) + ',' + echo);
        Serial4.println(String(curPacket));
        if (CX == 1)
        {
            Serial1.println(String(curPacket));
            packets++;
        }
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
        if (altitude < lastAlt + 1 && altitude > lastAlt - 1)
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
    if (mode == 'S')//if in simulation mode
    {
        return simPres;
    }
    else
    {
        //Serial1.println();
        return bmp.readAltitude(seaLvlPres);
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

    //Serial1.println("Start Altitude: " + (String)bmp.readAltitude(seaLvlPres));




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

////////////////////////////////////////
//***Primary Flight State Functions***//
////////////////////////////////////////
void Standby()
{

  
    return;
}

void Ascend()
{
    //read sensors with sample_sensors()
    if (TRUE)
    {

    }
    if (get_altitude() < lastAlt)//if altitude has decreased
    {
        state = Desc1;
    }
    return;
}

void Stage1Desc()
{
    //read sensors with sample_sensors()

    if (get_altitude() <= 400)//if altitude is or < 400m
    {
        //deploy the larger parachute
        state = Desc2;
    }
    return;
}

void Stage2Desc()
{

    //read sensors with sample_sensors()

    if (get_altitude() <= 300)//if altitude is or < 300m
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

    if (check_landing())//if landed = true
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

void loop() {
    switch (state)
    {
    case Stby:
        if (millis() - lastReadAlt > 500)//check every half second
        {
            if (altitude > 5.0)
            {
                lastReadAlt == millis();
                state = Asc;
            }
        }
        break;
    case Asc:
        if (millis() - lastReadAlt > 500)
        {
            if (altitude < lastAlt && altitude > 600)
            {
                state = Desc1;
            }
            else
            {
                lastReadAlt = millis();
                lastAlt = altitude;
            }
        }
        break;
    case Desc1:
        if (millis() - lastReadAlt > 500)
        {
            if (altitude <= 400)
            {
                state = Desc2;
            }
            else
            {
                lastReadAlt = millis();
            }
        }
        break;
    case Desc2:
        if (millis() - lastReadAlt > 1000)
        {
            if (altitude <= 300)
            {
                state = Desc3;
                tpRelease = 'R';
                
            }
            else
            {
                lastReadAlt = millis();
            }
        }
        break;
    case Desc3:
        if (millis() - lastReadAlt > 2500)
        {
            if (altitude < lastAlt+1 && altitude >lastAlt-1)
            {
                state = Grnd;

            }
            else
            {
                lastReadAlt = millis();
                lastAlt = altitude;
            }
        }
        break;
    case Grnd:
        delay(200);
        break;
    default:
        while (1)
        {
            Serial1.println("STATE MACHINE ERROR");
        }
    }
    sample_sensors();
    downlink_telem();//send data to ground if time and telem on
    read_serial();//Read incomming serial data (xbees)
    //check for commands after each flight function, parse_packet() if serial recieved

}