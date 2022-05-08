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

float lastAlt, simPres, seaLvlPres;
unsigned int lastReadAlt, lastPoll; 
states prev_state, state;
String cmd;

Adafruit_BME280 bme;
Adafruit_BNO055 myIMU = Adafruit_BNO055();


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
  
  if(get_altitude() > 3)//switch states when necessary
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
  return;
}



/////////////////////////////////////
//***Secondary Flight Functions***///
/////////////////////////////////////
void sensor_checklist()
{
    Serial.println();
    //begin barometer
    if (!bme.begin())
    {
        Serial.println("BME280\t"+(String)'#');
    }
    else
    {
        Serial.println("BME280\t"+(String)'@');//if BME starts successfully
        seaLvlPres = bme.readPressure() / 100.0;
    }

    if (!myIMU.begin())
    {
        Serial.println("IMU\t" + (String)'#');
    }
    else
    {
        Serial.println("IMU\t" + (String)'@');//if IMU starts successfully
    }



    Serial.println("System Ready");
    return;
}

void parse_packet()
{
  
  return;
}

void sample_sensors()
{
  //sample sensors if been 250ms
  return;
}

void poll_payload()
{
    //poll payload if been 250ms
    return;
}

void transmit_data()
{
    //send packet from container to GCS (if CX is on) if been 1 sec
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

void saveRecovery()
{
    //save all current data for recovery
    return;
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

sensor_checklist();

Serial.println("Cur Altitude: " + (String)bme.readAltitude(seaLvlPres));


  
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
        //error loop
      }
  }
  //transmit_data()
  //check for commands after each flight function, parse_packet() if serial recieved

}