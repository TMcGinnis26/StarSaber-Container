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
#define TESTLED 13
#define TeamID 1092
#define flightLowerTime 1000 //lower time for flight mode (ms)
#define simLowerTime 1000 //lower time for simulation mode (ms)
#define serialTimeout 300 //Timeout for waiting on serial input (ms)
#define SERIAL2TIMEOUT 1500 //Timeout for serial line 2 (ms)


//Telemetry Values
float voltage, temp, pError, GPS_Lat, GPS_Lon, GPS_alt;//voltage draw, altitude, temperature, pointing error
int packets = 1, mish = 0, mismin = 0;//Packet Count, Flight State(moved to operation vars), Mission Hours, Mission Minutes 
int GPS_Sats, GPS_hour, GPS_min, GPS_sec, SimMode;//GPS Sattelites, GPS: Hour, GPS:Minutes, TelemetryState; 
float missec = 0.0;//Mission seconds + milliseconds, gps time seconds
char mode = 'F';//F for flight, S for simulation
String echo = "NUL", curPacket = "", payloadCurPacket = "";//Last command with its arguments, no space
bool ledstat = false, CX = false, lowerStat = false;
char tpRelease = 'N', inChar ;//Default payload not released 'T' for released?
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

float lastAlt = -5.0, simAlt = 0.0, seaLvlPres = 988.0, curPres, simPress;
unsigned int lastReadAlt, lastCPoll = 0, lastTPoll = 0, lastSampleTime,lastBlink, releaseStart, serialWait;//last time readingAltitude, sampling Cont. sensors, polling payload 
double RTCMillis;
int HrsI, MinI, SecI, lastDelim, Payload_Packets;
states prev_state, state;
String cmd, tempString, payloadCmd;
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
    //Serial1.println("System Ready");
    return;
}

void read_serial()
{
    
    if (Serial1.available() > 0)//if GND serial data is available
    {
        cmd = "";
        inChar = ' ';
        serialWait = millis();
        while (millis() - serialWait < serialTimeout)
        {
            if (Serial1.available())
            {
                inChar = Serial1.read();
                if (inChar == '\n')
                    break;
                cmd += String(inChar);
            }
            
        }
       // while (Serial1.available() || millis() - serialWait < serialTimeout)//take in the command until NL character
       // {
           
       // }

        //Serial1.println(String(cmd));

        if (cmd.substring(0, 8) == "CMD,1092")//if packet is CMD for 1092
        {
            if (cmd.substring(9, 11) == "ST")//set RTC time
            {
                rtc.begin();

                tempString = cmd.substring(12);
                HrsI = tempString.substring(0, tempString.indexOf(':')).toInt();
                lastDelim = tempString.indexOf(':');
                MinI = tempString.substring(lastDelim + 1, tempString.indexOf(':', lastDelim + 1)).toInt();
                lastDelim = tempString.indexOf(':', lastDelim + 1);
                SecI = tempString.substring(lastDelim + 1).toInt();

                rtc.adjust(DateTime(2022, 1, 1, HrsI, MinI, SecI));
                now = rtc.now();
                echo = "ST";
                return;
            }

            if (cmd.substring(9, 13) == "SIMP")//set the current SIMP altitude
            {
                
                simPress = cmd.substring(14).toFloat();
                simPress = simPress / 100.0;//convert to hPa

                simAlt = 44330 * (1.0 - pow(simPress / seaLvlPres, 0.1903));//calculate altitude, given ground pressure
                
                echo = "SIMP"+(String)simPress;
                return;
            }
            
            if (cmd.substring(9, 12) == "SIM")//interpret SIM command
            {
                if (cmd.substring(13)=="ENABLE")
                {
                    echo = "SIMENABLE";
                    SimMode = 1;
                    return;
                }
                else if (SimMode == 1 && cmd.substring(13) == "ACTIVATE")
                {
                    echo = "SIMACTIVATE";
                    SimMode = 2;
                    mode = 'S';
                    return;
                }
                else if (cmd.substring(13)=="DISABLE")
                {
                    echo = "SIMDISABLE";
                    SimMode = 0;
                    mode = 'F';
                    return;
                }
                
                return;
            }

            if (cmd.substring(9,11)=="CX")//enable or disable Telemetry transmission
            {
                if (cmd.substring(12,14)=="ON")
                {
                    CX = true;
                    echo = "CXON";
                    return;
                }
                else if (cmd.substring(12,15)=="OFF")
                {
                    CX = false;
                    echo = "CXOFF";
                    return;
                }
                return;
            }
        }
    }
    return;
}

void sample_sensors()
{
    //if (millis() - lastSampleTime >= 75)//for limiting the sample speed
    //{
    update_time();
    bmp.performReading();
    temp = bmp.temperature;
    voltage = ina260.readBusVoltage()*0.001;
    altitude = get_altitude();

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
    RTCMillis = ((millis() % 1000)) / 1000.0;//calculate milliseconds
    missec = now.second() + RTCMillis;
    return;
}


void poll_payload()
{
    Serial2.println("CMD,1092,POLL");//poll payload for telemetry
    return;
}

void lowerPayload()
{
    if (mode == 'F')
    {
        if (millis() - releaseStart >= flightLowerTime)
        {
            //stop descent servo
            lowerStat = true;
            return;
        }
    }
    else if (mode == 'S')
    {
        if (millis() - releaseStart >= simLowerTime)
        {
            //stop descent servo
            lowerStat = true;
            return;
        }
    }
   
    return;
}

void downlink_telem()
{
    if ((millis() - lastCPoll) >= 950)//slightly faster than 1 sec
    {
        curPacket = String(String(TeamID) + ',' + mish + ':' + mismin + ':' + String(missec) + ',' + packets + ",C," + mode + ',' + tpRelease + ',' + String(altitude) + ',' + temp + ',' + voltage + ',' + GPS_hour + ':' + GPS_min + ':' + GPS_sec + ',' + GPS_Lat + ',' + GPS_Lon + ',' + GPS_alt + ',' + GPS_Sats + ',' + String(state) + ',' + echo);
        Serial4.println(String(curPacket));//write telemetry to open log
        if (CX)//transmit to GND if telemetry is enabled
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
    if (millis() - lastReadAlt >= 2000)//check every 2 sec
    {
        altitude = get_altitude();
        if (altitude < lastAlt + 1 && altitude > lastAlt - 1 && altitude < 100)
        {
            return true;
        }
        lastReadAlt = millis();
        lastAlt = altitude;
        return false;

    }
    return false;
}

void updateEEPROM()
{
    //save all current data for recovery
    return;
}

void read_payload()
{

    if (Serial2.available() > 0)
    {
        inChar = ' ';
        serialWait = millis();
        while (millis() - serialWait < SERIAL2TIMEOUT && Payload_Packets < 4 )
        {
            while (Serial2.available())
            {
                inChar = Serial2.read();
                if (inChar == '\n')
                {
                    if (payloadCurPacket.substring(0, 6) == "1092,T")//if team matches && is Payload packet
                    {
                        payloadCurPacket = payloadCurPacket.substring(4);//remove front of packet
                        sample_sensors();
                        
                        payloadCmd = String(TeamID) + "," + String(mish) + ":" + String(mismin) + ":" + String(missec) + "," + packets;

                        Serial4.println(payloadCmd + payloadCurPacket);//record to open log
                        Serial1.println(payloadCmd + payloadCurPacket);//send to GND
                        packets++;
                    }
                    payloadCmd = "";
                    payloadCurPacket = "";
                    Payload_Packets++;
                }
                else
                {
                    payloadCurPacket += String(inChar);
                }

            }
        }
        Payload_Packets = 0;
    }
    return;
}

float get_altitude()
{
   
    if (mode == 'S')//if in simulation mode
    {
        return simAlt;
    }
    else
    {
        return bmp.readAltitude(seaLvlPres);
    }
}




void setup() {
    pinMode(TESTLED, OUTPUT);

    //Start All Sensors
    Serial.begin(9600);
    Serial1.begin(9600);//ground xbee
    Serial2.begin(115200);//payload xbee
    Serial4.begin(9600);//Open Log
    GPSSerial.begin(9600);

    I2C_Sensor_Init();//Init I2C Sensors

    /*Temporarily remove the recovery feature
   if(!recover())
   {
    state = Stby; //default standby state
    //sample the barometer to get the sea level pressure
   }
  */

    state = Stby;//temporary
}

////////////////////////////////////////
//***Primary Flight State Functis***//
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
                //payload release servo
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
                tpRelease = 'R';//payload is released
                //start unspool servo
                releaseStart = millis();
            }
            else
            {
                lastReadAlt = millis();
            }
        }
        break;
    case Desc3:
        if (!lowerStat)//if not done lowering
        {
            lowerPayload();
        }

        poll_payload();
        delay(50);
        read_payload();

        if (check_landing())//state check
        {
            state = Grnd;
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
    read_serial();//Read incomming serial data (xbees)

    downlink_telem();//send data to ground if time and telem on

    //check for commands after each flight function if serial recieved

}