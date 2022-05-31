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
#define TeamID 1092



//Telemetry Values
float voltage, temp, pError, GPS_Lat, GPS_Lon, GPS_alt;//voltage draw, altitude, temperature, pointing error
int packets = 1, mish = 0, mismin = 0;//Packet Count, Flight State(moved to operation vars), Mission Hours, Mission Minutes 
int GPS_Sats, GPS_hour, GPS_min, CX = 1;//GPS Sattelites, GPS: Hour, GPS:Minutes, TelemetryState; 
float missec = 0.0, GPS_sec;//Mission seconds + milliseconds, gps time seconds
char mode = 'F';//F for flight, S for simulation
String echo = "NUL", curPacket = "";//Last command with its arguments, no space
bool ledstat = false;
char tpRelease = 'N';//Default payload not released 'T' for released?
float altitude;

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

float lastAlt = -5.0, simAlt = 0.0, seaLvlPres = 988.0, curPres;
unsigned int lastReadAlt, lastCPoll = 0, lastTPoll = 0, lastSampleTime, RTCMillis,lastBlink;//last time readingAltitude, sampling Cont. sensors, polling payload 
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
    if (!rtc.begin())
    {
        Serial1.println("FAILED TO START RTC");
    }
    rtc.adjust(DateTime(2022,1,1,0,0,0));
    digitalWrite(TESTLED, HIGH);
    //begin sensors
    if (!ina260.begin())
    {
        Serial1.println("FAILED TO START INA");
    }
    
    if (!bmp.begin_I2C())
    {
        Serial1.println("BMP FAILED TO START");
    }



    for (int i = 0; i < 5; i++)
    {
        bmp.performReading();
        curPres = bmp.pressure / 100.0;
        altitude = bmp.readAltitude(curPres);
        delay(100);
    }
    seaLvlPres = curPres;//set sea level at current altitude

   
    if (!rtc.begin())
    {
        Serial1.println("RTC NOT STARTED");
    }
    
   

    digitalWrite(TESTLED, LOW);
    Serial1.println("System Ready");
    return;
}

void read_serial()
{
    if (Serial2.available() > 0)
    {
        while (Serial2.available() > 0 && Serial2.peek()!='\n')
        {
            curPacket += Serial2.read();
        }
        if (Serial2.peek()=='\n')
        {
            tempString = Serial2.read();
        }

        if(curPacket.substring(0, 6) == "1092,T")//if team matches && is Payload packet
        {
            
            sample_sensors();

            curPacket = curPacket.substring(4);//remove front of packet
            cmd = String(TeamID) + ',' + String(mish) + ":" + String(mismin) + ":" + String(missec+RTCMillis)+","+packets;
            cmd += curPacket;

            Serial4.println(cmd);//record to open log
            Serial1.println(cmd);//send to GND
            packets++;
        }
        cmd = "";
        curPacket = "";
    }



    if (Serial1.available() > 0)//if GND serial data is available
    {
        while (Serial1.available() > 0 && Serial1.peek() != '\n')
        {
            cmd += Serial1.read();
        }
        if (Serial1.peek() == '\n')
        {
            tempString = Serial1.read();
        }

        if (cmd.substring(0,2) == "ST")//set RTC time
        {
            rtc.begin();
            //Serial1.println(cmd); // return CMD to GND
            
            String Hrs = cmd.substring(2,cmd.indexOf(":"));
            int HrsI = Hrs.toInt();
            Serial1.println(String(HrsI));
            rtc.adjust(DateTime(2022,1,1, HrsI, 30, 30));
            //rtc.adjust(DateTime("Jul 21 2015", "02:44:20"));
            now = rtc.now();
            Serial1.println("SetTime: "+String(now.hour()) + ':' + String(now.minute())+':'+String(now.second()));
        }
        if (cmd.substring(0, 4) == "SIMP")//set the current SIM pressure
        {
            double simPress;
            //pull simPress from the packet

            simAlt = 44330 * (1.0 - pow(simPress / seaLvlPres, 0.1903));//calculate altitude, given ground pressure
        }
        
    }
    return;
}

void sample_sensors()
{
    //if (millis() - lastSampleTime >= 75)
    //{
    
    update_time();
    bmp.performReading();
    temp = bmp.temperature;
    
    voltage = ina260.readBusVoltage()*0.001;
    
    altitude = get_altitude();

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
   // }
    return;
}

void update_time()
{
    now = rtc.now();
    mish = now.hour();
    mismin = now.minute();
    RTCMillis = millis() % 1000;//calculate milliseconds
    missec = now.second() + RTCMillis;
    return;
}


void poll_payload()
{
    //poll CMD to payload
    return;
}

void downlink_telem()
{
    if ((millis() - lastCPoll) >= 900)//slightly faster than 1 sec
    {
        curPacket = String(String(TeamID) + ',' + mish + ':' + mismin + ':' + String(missec) + ',' + packets + ",C," + mode + ',' + tpRelease + ',' + String(altitude) + ',' + temp + ',' + voltage + ',' + GPS_hour + ':' + GPS_min + ':' + GPS_sec + ',' + GPS_Lat + ',' + GPS_Lon + ',' + GPS_alt + ',' + GPS_Sats + ',' + String(state) + ',' + echo);
        Serial4.println(String(curPacket));//write telemetry to open log
        if (CX == 1)
        {
            Serial1.println(String(curPacket));
            
        }
        packets++;//increment packet count
        lastCPoll = millis();
        curPacket = "";
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
    //Serial1.println("Test: " + String(bmp.readAltitude(seaLvlPres)));
   // return bmp.readAltitude(seaLvlPres);
    if (mode == 'S')//if in simulation mode
    {
        return simAlt;
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

    

    
    //Serial1.println("Ground pressure: " + String(curPres) + " ALT: " + String(bmp.readAltitude(seaLvlPres)) + " Function: " + altitude);
    

    state = Stby;//temporary
    //sample barom for sea level pressure
}

////////////////////////////////////////
//***Primary Flight State Functions***//
////////////////////////////////////////


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
        if (millis() - lastReadAlt > 2000)//state check
        {
            //Serial1.println(String(altitude));
            if (altitude >= 20.0)
            {
                state = Asc;
            }
            else
            {
                lastReadAlt == millis();
                lastAlt = altitude;
            }
        }
        break;
    case Asc:
        if (millis() - lastReadAlt > 500)//state check
        {
            if (altitude < lastAlt && altitude > 400)
            {
                state = Desc1;
            }
            else
            {
               // Serial1.println(String(altitude) + "from " + String(lastAlt));
                lastReadAlt = millis();
                lastAlt = altitude;
            }
        }
        break;
    case Desc1:
        if (millis() - lastReadAlt >= 500)//state check
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
        if (millis() - lastReadAlt >= 500)//state check
        {
            if (altitude <= 300)
            {
                state = Desc3;
                tpRelease = 'R';
                //send CMD to payload to turn on 
            }
            else
            {
                lastReadAlt = millis();
            }
        }
        break;
    case Desc3:
        if(millis() - lastTPoll >=240)//if time to poll payload
        {
            poll_payload();
        }

        if (millis() - lastReadAlt > 1000)//state check
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