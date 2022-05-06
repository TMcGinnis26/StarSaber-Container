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

//Telemetry Values
float voltage, altitude, temp, pError, GPS_Lat, GPS_Lon, GPS_alt;//voltage draw, altitude, temperature, pointing error
int packets, mh, mm;//Packet Count, Flight State(moved to operation vars), Mission Hours, Mission Minutes 
int GPS_Sats, GPS_hour, GPS_min;//GPS Sattelites, GPS Times: Hour, Minutes;
float ms, GPS_sec;//Mission seconds + milliseconds
bool tpRelease, cx;//Release state of tethered payload, telemetry state (on/off)
char mode;//F for flight, S for simulation
String echo;//Last command with its arguments, no space

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

float lastAlt, simPres;
unsigned int lastTime, lastPoll; 
states prev_state, state;
String cmd;


//***Recovery Functions***//
bool recover()
{
  //if data is present, recover from the EEPROM, return true
  return false;//return false if doesn't/cant recover
}
  




//***Primary Flight State Functions***//
void Standby()
{

  
  if(get_altitude() > 3)
  {
    state = Asc;
  }
 return; 
}

void Ascend()
{

  
  if(1)//if altitude has decreased
  {
    state = Desc1;
  }
  return;
}

void Stage1Desc()
{

  
  if(get_altitude() <= 400)//if altitude is or < 400m
  {
    //deploy the larger parachute
    state = Desc2;
  }
  return;
}

void Stage2Desc()
{

  
  if(get_altitude() <= 300)//if altitude is or < 300m
  {
    //lower the tethered payload
    state = Desc3;
  }
  return;
}

void Stage3Desc()
{

  
    if(get_altitude() > lastAlt-1 && get_altitude() < lastAlt+1)//if altitude is still within +-1 m
  {
    state = Grnd;
  }
  return;
}

void Grounded()//grounded loop
{
  return;
}

//***Secondary Flight Functions***///
void parse_packet()
{
  
  return;
}

void sample_sensors()
{
  
  return;
}

void transmit_data(int reciever)
{
  //reciever values: 0-payload 1-ground
  if(reciever == 1)
  {
    //send container telemetry packet to ground
    
  }else if(reciever == 0)
  {
    //poll the payload for data
  }
}

float get_altitude()
{
  float readAlt;
  if(mode == 'S')//if in simulation mode
  {
    if(1)//if serial is available from ground xbee
    {
      if(0)//if the command is SIMP
      {
        readAlt = simPres;//currently represents simulated altitude calculated from given pressure
      }
    }
    readAlt = 0.0;
  }else
  {
    //read current measured pressure
  }

  return readAlt;
}




void setup() {
  //Start All Sensors
Serial.begin(115200);
Serial1.begin(115200);


  
  
 if(!recover())
 {
  state = Stby; //default standby state
 }

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
        //error loop
      }
    
  }

}