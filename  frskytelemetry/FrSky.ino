//Arduino pro mini 328 pinout
//8 - A0
//7 - A1
//6 - A2
//5 - A3
//1 - A4 - I2C baro, Accelerometer
//2 - A5 - I2C baro, Accelerometer
//2 - D1 - TXO Output to FrSky receiver
//8 - D2 - DALLAS temperature sensor
//7 - D3
//6 - D4 - GPS, TTL level
//5 - D5
//4 - D6
//3 - D7 - serial output
//2 - D8
//1 - D9


/*
2x4
7x3
1x2

baro, acc           2x4 PIN + - SCK SDL
DALLAS              2x3 PIN + - sig
GPS                 1x3 PIN + - sig
out to receiver     1x3 PIN + - sig
in from receiver    1x3 PIN + - 
fuel                1x3 PIN + - sig
RPM                 1x3 PIN + - sig
Voltage             1x2 PIN
*/


#include <SoftwareSerial.h>
 
// Interrupt
//#define ISR_UART ISR(USART0_UDRE_vect) 


// I2Cdev and BMA150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "BMA150.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"


// class default I2C address is 0x38
BMA150 accel;
//int16_t ax, ay, az;

#include "BMP085.h"
// class default I2C address is 0x77
// specific I2C addresses may be passed as a parameter here
// (though the BMP085 supports only one address)
BMP085 barometer;
//float pressure;
float altitude;
int32_t lastBMP085Time=0;
int16_t BMP085MeasDelay;
byte calibrationCycle=0;
float altitudeGround;
float altitudeAvg;
bool calBMPOk = false;
#define BMP_CAL_CYCLES 30

#define ignoreFix

static int32_t  GPS_coord[2];
#define LAT  0
#define LON  1

// user defines
//#define FAS_100  //if commment out, MultiWii vBat voltage will be send instead of FrSky FAS 100 voltage

static uint8_t  vbat;               // battery voltage in 0.1V steps

static uint8_t  GPS_numSat;
static uint16_t GPS_altitude,GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s
static uint16_t GPS_ground_course = 0;                       // degrees*10
static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
static uint8_t GPS_FIX;

static uint16_t Datas_RPM=50; //RPM
static uint16_t Datas_Fuel_level=50;

unsigned long lastTime=0;
unsigned int showInterval=1000;

// Data Ids  (bp = before point; af = after point)
// Official data IDs
#define ID_GPS_altidute_bp        0x01
#define ID_GPS_altidute_ap        0x09
#define ID_Temprature1            0x02
#define ID_RPM                    0x03
#define ID_Fuel_level             0x04
#define ID_Temprature2            0x05
#define ID_Volt                   0x06
#define ID_Altitude_bp            0x10
#define ID_Altitude_ap            0x21
#define ID_GPS_speed_bp           0x11
#define ID_GPS_speed_ap           0x19
#define ID_Longitude_bp           0x12
#define ID_Longitude_ap           0x1A
#define ID_E_W                    0x22
#define ID_Latitude_bp            0x13
#define ID_Latitude_ap            0x1B
#define ID_N_S                    0x23
#define ID_Course_bp              0x14
//#define ID_Course_ap            0x24
#define ID_Course_ap              0x1C
#define ID_Date_Month             0x15
#define ID_Year                   0x16
#define ID_Hour_Minute            0x17
#define ID_Second                 0x18
#define ID_Acc_X                  0x24
#define ID_Acc_Y                  0x25
#define ID_Acc_Z                  0x26
#define ID_Voltage_Amp_bp         0x3A
#define ID_Voltage_Amp_ap         0x3B
#define ID_Current                0x28
// User defined data IDs
#define ID_Gyro_X                 0x40
#define ID_Gyro_Y                 0x41
#define ID_Gyro_Z                 0x42

// Frame protocol
#define Protocol_Header           0x5E
#define Protocol_Tail             0x5E


// Serial config datas
#define TELEMETRY_FRSKY_SERIAL    1
#define TELEMETRY_FRSKY_BAUD      9600 
#define GPS_SERIAL                4
#define GPS_BAUD                  4800

SoftwareSerial SerialGPS        (GPS_SERIAL,5);             // RX, TX
//#define verbose
#ifdef verbose
SoftwareSerial SerialVerbose    (6,7); // RX, TX
#endif
static volatile uint8_t headTX,tailTX;
#define TX_BUFFER_SIZE 128
static uint8_t bufTX[TX_BUFFER_SIZE];
static uint8_t checksum;


// Timing
#define Time_telemetry_send 200 //ms
static uint8_t cycleCounter = 0;
static uint32_t FrSkyTime  = 0;

#include <OneWire.h>
#include <limits.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS D2
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
DallasTemperature dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
#ifndef NUMBER_OF_DEVICES
#define NUMBER_OF_DEVICES 2
#endif
DeviceAddress tempDeviceAddresses[NUMBER_OF_DEVICES];
unsigned int numberOfDevices; // Number of temperature devices found
unsigned long lastDsMeasStartTime;
#define MAX_CONVERSION_TIME12 750
#define MAX_CONVERSION_TIME11 375
#define MAX_CONVERSION_TIME10 188
#define MAX_CONVERSION_TIME9  94
#define DALLAS_RESOLUTION 12
bool dsMeasStarted=false;
float sensor[NUMBER_OF_DEVICES];

int sensorReading = INT_MIN;


void setup() {
  Serial.begin(9600);
  SerialGPS.begin(GPS_BAUD);
  #ifdef verbose
  SerialVerbose.begin(9600);
  SerialVerbose.println("Test GPS");
  #endif

  accInit();
  
  baroInit();
  barometer.setControl(BMP085_MODE_PRESSURE_3);
  delay(50);
  
  uint8_t i=0;
  while (numberOfDevices==0 && i<10) {
    dsInit(); 
    delay(500);
    i++;
  }
}

void loop() {
  //if (!SerialGPS.isListening()) SerialGPS.listen();
  if (SerialGPS.available())
    GPS_newFrame(SerialGPS.read());

  if (!dsMeasStarted)
  {
    #ifdef verbose
    SerialVerbose.print("\nRequest:");
    SerialVerbose.println(millis());
    #endif

    dsSensors.requestTemperatures(); 
    lastDsMeasStartTime=millis();
    dsMeasStarted=true;
  }

  if (dsMeasStarted) {
    if (millis() - lastDsMeasStartTime>MAX_CONVERSION_TIME12) {
      #ifdef verbose
      SerialVerbose.print("\nRead:");
      SerialVerbose.println(millis());
      #endif
      
      dsMeasStarted=false;
      //saving temperatures into variables
      for (byte i=0;i<numberOfDevices; i++) {
        float tempTemp=-126;
        for (byte j=0;j<10;j++) { //try to read temperature ten times
          //tempTemp = dsSensors.getTempCByIndex(i);
          tempTemp = dsSensors.getTempC(tempDeviceAddresses[i]);
          if (tempTemp>=-55) {
            break;
          }
        }
        
        #ifdef verbose
        SerialVerbose.print("ID:");
        for (byte j = 0; j < 8; j++) {
          if (tempDeviceAddresses[i][j] < 16) {
            SerialVerbose.print('0',HEX);
          }
          SerialVerbose.print(tempDeviceAddresses[i][j],HEX);
        }
        SerialVerbose.println();
        
        SerialVerbose.print("Temperature:");
        SerialVerbose.println(tempTemp);
        #endif
        sensor[i]=tempTemp;
      } 
      //sensor[0]=18;
      //sensor[1]=26;
    }
  }
  
  
  // request pressure (3x oversampling mode, high detail, 23.5ms delay)
  if (millis() - lastBMP085Time > BMP085MeasDelay) {
    // read calibrated pressure value in Pascals (Pa)
    //pressure = barometer.getPressure();
    // calculate absolute altitude in meters based on known pressure
    // (may pass a second "sea level pressure" parameter here,
    // otherwise uses the standard value of 101325 Pa)
    altitude = barometer.getAltitude(barometer.getPressure());
    if (calibrationCycle < BMP_CAL_CYCLES) {
      altitudeAvg += altitude;
      calibrationCycle++;
    }
    if (calibrationCycle == BMP_CAL_CYCLES) {
      altitudeGround = altitudeAvg / calibrationCycle;
      #ifdef verbose
      SerialVerbose.print(" Alt ground:");
      SerialVerbose.print(altitudeGround);
      #endif
      calibrationCycle++;
      calBMPOk = true;
    }
    
    #ifdef verbose
    //SerialVerbose.print("Press [hPa]:");
    //SerialVerbose.println(pressure);
    if (altitudeGround>0) {
      SerialVerbose.print("Alt [m]:");
      SerialVerbose.println(altitude - altitudeGround);
    }
    #endif
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    lastBMP085Time = millis();
  }
  
  
  telemetry_frsky();
    

    
//  if (millis() - lastTime > showInterval) {
 //   lastTime=millis();
    
    
    /*
    Serial.print("FIX:");
    if (GPS_FIX>0)
      Serial.println("YES");
    else
      Serial.println("NO");
    Serial.print("LAT:");
    Serial.println(GPS_coord[LAT]);
    Serial.print("LON:");
    Serial.println(GPS_coord[LON]);
    Serial.print("SatInView:");
    Serial.println(GPS_numSat);
    Serial.print("Altitude:");
    Serial.println(GPS_altitude);
    Serial.print("Speed:");
    Serial.println(GPS_speed);
    Serial.print("Course:");
    Serial.println(GPS_ground_course);
    */
  //}
}

#define FRAME_GGA  1
#define FRAME_RMC  2

bool GPS_newFrame(char c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;
  if (c == '$') {
    //SerialVerbose.println();
    //SerialVerbose.print(c);
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    //SerialVerbose.print(c);
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {GPS_coord[LAT] = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS_coord[LAT] = -GPS_coord[LAT];
      else if (param == 4)                     {GPS_coord[LON] = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS_coord[LON] = -GPS_coord[LON];
      else if (param == 6)                     {GPS_FIX = (string[0]  > '0');}
      else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
      else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}	// altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
    //  if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation	
      if      (param == 7)                     {GPS_speed = (uint32_t)grab_fields(string,1);}  //gps speed in knots*10
      else if (param == 8)                     {GPS_ground_course = grab_fields(string,1); }                 //ground course deg*10 
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    //SerialVerbose.print(c);
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     //SerialVerbose.print(c);
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  if (frame) GPS_Present = 1;
  return frameOK && (frame==FRAME_GGA);
}


#define DIGIT_TO_VAL(_x)	(_x - '0')
uint32_t GPS_coord_to_degrees(char* s)
{
	char *p, *q;
	uint8_t deg = 0, min = 0;
	unsigned int frac_min = 0;

	// scan for decimal point or end of field
	for (p = s; isdigit(*p); p++)
		;
	q = s;

	// convert degrees
	while ((p - q) > 2) {
		if (deg)
			deg *= 10;
		deg += DIGIT_TO_VAL(*q++);
	}
	// convert minutes
	while (p > q) {
		if (min)
			min *= 10;
		min += DIGIT_TO_VAL(*q++);
	}
	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (int i = 0; i < 4; i++) {
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

// helper functions 
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;
  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {		// convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 

 static void sendDataHead(uint8_t Data_id)
 {
    //SerialVerbose.print("Header:");
    write_FrSky8(Protocol_Header);
    #ifdef verbose
    SerialVerbose.print("ID:");
    #endif
    write_FrSky8(Data_id);
 }

void write_FrSky8(uint8_t Data)
{
  #ifdef verbose
  SerialVerbose.print(Data, HEX);
  SerialVerbose.print(" ");
  #endif
  Serial.write(Data);
}

void write_FrSky16(uint16_t Data)
{
  uint8_t Data_send;
  Data_send = Data;
  check_FrSky_stuffing(Data_send);
  Data_send = Data >> 8 & 0xff;
  check_FrSky_stuffing(Data_send);
}

static void sendDataTail(void)
{
  write_FrSky8(Protocol_Tail);      
  #ifdef verbose
  SerialVerbose.println();
  #endif
}

void check_FrSky_stuffing(uint8_t Data) //byte stuffing
{
  if (Data == 0x5E)   
  {
     write_FrSky8(0x5D);
     write_FrSky8(0x3E);
  }
  else if (Data == 0x5D)   
  {
     write_FrSky8(0x5D);
     write_FrSky8(0x3D);
  }
  else
  {
     write_FrSky8(Data);         
  }
}

void telemetry_frsky()
{
  if (millis() > FrSkyTime ) //
  {          
    FrSkyTime = millis() + Time_telemetry_send;
    cycleCounter++;
    //SerialVerbose.println("200ms");
     // Datas sent every 200 ms
    send_Temperature1();
    send_RPM();
    send_Temperature2();
    send_Cell_volt();      
    if (calBMPOk) { //calibration is complete
      send_Altitude();
    }
    send_Accel();
    send_Voltage_ampere();
    sendDataTail();   
    
    if ((cycleCounter % 5) == 0)
    {     
      // Datas sent every 1s
      send_GPS_altitude();
      send_Fuel_level();
      send_GPS_speed();
      send_GPS_position();
      send_Course();
      sendDataTail();   
    }
  
    if ((cycleCounter % 25) == 0)
    {     
      //SerialVerbose.println("5s");
      // Datas sent every 5s            
      send_Time();
      sendDataTail(); 
      
      cycleCounter = 0;       
     }
  }
}


 //*********************************************************************************
 //-----------------   Telemetrie Datas   ------------------------------------------   
 //*********************************************************************************

 // GPS altitude
 void send_GPS_altitude(void)
 {         
    #ifndef ignoreFix
    if (GPS_FIX && GPS_numSat >= 4)
    {
    #endif
       //SerialVerbose.print("GPS altitude:");
       //SerialVerbose.println(GPS_altitude);
       sendDataHead(ID_GPS_altidute_bp);
       //SerialVerbose.print("ALT before:");
       write_FrSky16(GPS_altitude);
       sendDataHead(ID_GPS_altidute_ap);
       //SerialVerbose.print("Alt after:");
       write_FrSky16(0);
    #ifndef ignoreFix
    }
    #endif
 }
 
 // Temperature
void send_Temperature1(void)
{
  sendDataHead(ID_Temprature1);
  #ifdef verbose
  SerialVerbose.println((int16_t)sensor[0]);
  #endif
  write_FrSky16((int16_t)sensor[0]);
}

//Temperature 2
void send_Temperature2(void)
{
  sendDataHead(ID_Temprature2);
  write_FrSky16((int16_t)sensor[1]);
}

 
// RPM
void send_RPM(void)
 {
  sendDataHead(ID_RPM);
  write_FrSky16(Datas_RPM);
}

// Fuel level
void send_Fuel_level(void)
{
  sendDataHead(ID_Fuel_level);
  write_FrSky16(Datas_Fuel_level);
}


// Cell voltage  todo !!!!!!!!!!!!!!!!!!
void send_Cell_volt(void) // Datas FrSky FLVS-01 voltage sensor
{
  /*the first 4 bit of the voltage data refers to battery cell number, while the last 12 bit refers to the 
  voltage value. 0-2100 corresponding to 0-4.2V. 
  e.g: 
  ......0x5E 0x06 0x18 0x34 0x5E........
  0x06 refers to the voltage DataID
  0x18 0x34
  0001 1000 0011 0100
  0001(1) means the first cell of pack, the last12bit 0x834 (2100) means the value is 4.2V

  //bit
  //0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
  //cell#  | voltage - 2100 = 0-4.2V
  */
  
  uint16_t datasVolt;
  uint16_t cellNo=1;
  cellNo=cellNo<<12;
  
  float voltageConst=0.002;
  float voltage = 4.2; //V
  datasVolt=(uint16_t)(voltage/voltageConst);
  
  datasVolt=cellNo | datasVolt;

  uint16_t temp;
  //0001 1000 0011 0011   0x1833
  //datasVolt & 0xff
  //0000 0000 0011 0011   0x33
  //<<8
  //0011 0011 0000 0000   0x3300
  temp = (datasVolt & 0xff)<<8; 
  //datasVolt>>8
  //0001 1000 0000 0000   0x18
  //| temp
  //0011 0011 0001 1000   0x3318
  temp = datasVolt>>8 | temp;
  datasVolt = temp;
  
  /*sendDataHead(ID_Volt);
  #ifdef verbose
  SerialVerbose.print("Volt:");
  SerialVerbose.println(datasVolt);
  #endif
  write_FrSky16(datasVolt);
  */
  sendDataHead(ID_Volt);
  write_FrSky16(0x1408);
  
  sendDataHead(ID_Volt);
  write_FrSky16(0x1F1B);


  sendDataHead(ID_Volt);
  write_FrSky16(0x3428);

  sendDataHead(ID_Volt);
  write_FrSky16(0x3438);

  sendDataHead(ID_Volt);
  write_FrSky16(0x2448);

  sendDataHead(ID_Volt);
  write_FrSky16(0x3458);

  /*sendDataHead(ID_Volt);
  write_FrSky16(0x3168);*/
  
}

// Altitude from BMP
void send_Altitude(void)
{
  sendDataHead(ID_Altitude_bp);
  write_FrSky16((int16_t)(altitude - altitudeGround));
  sendDataHead(ID_Altitude_ap);
  write_FrSky16(0); //nejak nepracuje s desetinama
}

// GPS speed
void send_GPS_speed(void)
{
  #ifndef ignoreFix
  if (GPS_FIX && GPS_numSat >= 4)
  {            
  #endif
    sendDataHead(ID_GPS_speed_bp);
    //write_FrSky16(GPS_speed * 0.036); //cm/s -> km/h
    write_FrSky16(GPS_speed / 10); //knots
    sendDataHead(ID_GPS_speed_ap);
    write_FrSky16(GPS_speed % 10); //knots
    //write_FrSky16(0);
  #ifndef ignoreFix  
  }
  #endif
}

// GPS position
void send_GPS_position(void)
{
  #ifndef ignoreFix  
  if (GPS_FIX && GPS_numSat >= 4)
  {
  #endif
    sendDataHead(ID_Longitude_bp);
    write_FrSky16(abs(GPS_coord[LON]) / 100000);
    sendDataHead(ID_Longitude_ap);
    write_FrSky16(abs((GPS_coord[LON])/10)  % 10000);
    sendDataHead(ID_E_W);
    write_FrSky16(GPS_coord[LON] < 0 ? 'W' : 'E');

    sendDataHead(ID_Latitude_bp);
    write_FrSky16(abs(GPS_coord[LAT]) / 100000);
    sendDataHead(ID_Latitude_ap);
    write_FrSky16(abs((GPS_coord[LAT])/10) % 10000);
    sendDataHead(ID_N_S);
    write_FrSky16(GPS_coord[LAT] < 0 ? 'S' : 'N');
  #ifndef ignoreFix  
  }
  #endif
}

 // Course
 void send_Course(void)
 {
    sendDataHead(ID_Course_bp);
    write_FrSky16(GPS_ground_course);
    sendDataHead(ID_Course_ap);
    write_FrSky16(0);
 }

// Time
void send_Time(void)
{
  uint32_t seconds_since_start = millis() / 1000;
        
  sendDataHead(ID_Date_Month); //0x15
  
  //Example
  //write_FrSky16(540); 
  //0000 0010 0001 1100 = 0x021C = 28.2.  
  // 0x02    |  0x1C
  //write_FrSky16((0x02<<8)|0x1C); //28.2.

  write_FrSky16((0x01<<8)|0x01); //1.1.
  sendDataHead(ID_Year); //0x16
  write_FrSky16(12); //2012

  sendDataHead(ID_Hour_Minute); //0x17
  write_FrSky16(((seconds_since_start / 60) % 60)<<8 | ((seconds_since_start / 3600) % 24));
  sendDataHead(ID_Second); //0x18
  write_FrSky16(seconds_since_start % 60);
}

// ACC
void send_Accel(void)
{
  int16_t Datas_Acc_X;
  int16_t Datas_Acc_Y;
  int16_t Datas_Acc_Z;
  // read raw gyro measurements from device
  accel.getAcceleration(&Datas_Acc_X, &Datas_Acc_Y, &Datas_Acc_Z);
  // display tab-separated accel x/y/z values
  #ifdef verbose
  SerialVerbose.print("accel:\t");
  SerialVerbose.print(Datas_Acc_X); SerialVerbose.print("\t");
  SerialVerbose.print(Datas_Acc_Y); SerialVerbose.print("\t");
  SerialVerbose.println(Datas_Acc_Z);    
  SerialVerbose.println(accel.getTemperature());
  #endif
  
  // Datas_Acc_X = ((float)accSmooth[0] / acc_1G) * 1000;
  // Datas_Acc_Y = ((float)accSmooth[1] / acc_1G) * 1000;
  // Datas_Acc_Z = ((float)accSmooth[2] / acc_1G) * 1000;

  //for debug
  Datas_Acc_X = 8888;
  Datas_Acc_Y = 512;
  Datas_Acc_Z = 251;

  sendDataHead(ID_Acc_X);
  write_FrSky16(Datas_Acc_X);
  sendDataHead(ID_Acc_Y);
  write_FrSky16(Datas_Acc_Y);
  sendDataHead(ID_Acc_Z);
  write_FrSky16(Datas_Acc_Z);      
}

void send_Voltage_ampere(void)
{
  #if defined (FAS_100)   // todo   !!!!!!!!!!!!!!!!!
  {
     uint16_t Datas_Voltage_Amp_bp;
     uint16_t Datas_Voltage_Amp_ap;
     uint16_t Datas_Current;   

     Datas_Voltage_Amp_bp = 0;
     Datas_Voltage_Amp_ap = 0;   
     Datas_Current = 0;

     sendDataHead(ID_Voltage_Amp_bp);
     write_FrSky16(Datas_Voltage_Amp_bp);
     sendDataHead(ID_Voltage_Amp_ap);
     write_FrSky16(Datas_Voltage_Amp_ap);   
     sendDataHead(ID_Current);
     write_FrSky16(Datas_Current);
  }
  #else   // use vBat
  {
     uint16_t Datas_Voltage_vBat_bp;
     uint16_t Datas_Voltage_vBat_ap;   
     uint16_t voltage;
     vbat =420;
     voltage = (vbat * 110) / 21;          
     Datas_Voltage_vBat_bp = voltage / 100;
     Datas_Voltage_vBat_ap = ((voltage % 100) + 5) / 10;         

     sendDataHead(ID_Voltage_Amp_bp);
     write_FrSky16(Datas_Voltage_vBat_bp);
     sendDataHead(ID_Voltage_Amp_ap);
     write_FrSky16(Datas_Voltage_vBat_ap);   

    sendDataHead(ID_Current);
    write_FrSky16(505); //50A

  }
  #endif
}

void dsInit(void) {
  
  dsSensors.begin();
  numberOfDevices = dsSensors.getDeviceCount();
  
  #ifdef verbose
  SerialVerbose.print("DALLAS:");
  SerialVerbose.println(numberOfDevices);
  #endif
  // Loop through each device, print out address
  for (byte i=0;i<numberOfDevices; i++) {
      // Search the wire for address
    if (dsSensors.getAddress(tempDeviceAddress, i)) {
      /*for (byte j=0; j<8; j++) {
        if (tempDeviceAddress[j] < 16) Serial.print("0");
      }
      */
      memcpy(tempDeviceAddresses[i],tempDeviceAddress,8);
    }
    else
    {
      //Serial.println("Unable to get device address for sensor " + i);
    }
  }
  dsSensors.setResolution(DALLAS_RESOLUTION);
  dsSensors.setWaitForConversion(false);
}


void accInit(void) {
  Wire.begin();
  accel.initialize();
  #ifdef verbose
  SerialVerbose.println("Testing device connections...");
  SerialVerbose.println(accel.testConnection() ? "BMA150 connection successful" : "BMA150 connection failed");
  SerialVerbose.print("Device ID:");
  SerialVerbose.println(accel.getDeviceID());
  SerialVerbose.print("Chip rev.:");
  SerialVerbose.println(accel.getChipRevision());
  #endif
  accel.setRange(BMA150_RANGE_8G);
}

void baroInit() {
  barometer.initialize();
  // verify connection
  #ifdef verbose
  SerialVerbose.println("Testing device connections...");
  SerialVerbose.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
  #endif
  BMP085MeasDelay = barometer.getMeasureDelayMilliseconds();
}


/*
void SerialWrite(uint8_t port,uint8_t c){
 switch (port) {
    case 0: serialize8(c);UartSendData(); break;                 // Serial0 TX is driven via a buffer and a background intterupt
    #if defined(MEGA) || defined(PROMICRO)
    case 1: while (!(UCSR1A & (1 << UDRE1))) ; UDR1 = c; break;  // Serial1 Serial2 and Serial3 TX are not driven via interrupts
    #endif
    #if defined(MEGA)
    case 2: while (!(UCSR2A & (1 << UDRE2))) ; UDR2 = c; break;
    case 3: while (!(UCSR3A & (1 << UDRE3))) ; UDR3 = c; break;
    #endif
  }
}

void serialize8(uint8_t a) {
  uint8_t t = headTX;
  if (++t >= TX_BUFFER_SIZE) t = 0;
  bufTX[t] = a;
  checksum ^= a;
  headTX = t;
}

void UartSendData() {
  #if defined(PROMICRO)
    while(headTX != tailTX) {
      if (++tailTX >= TX_BUFFER_SIZE) tailTX = 0;
      uint8_t* p = bufTX+tailTX;
      #if !defined(TEENSY20)
        USB_Send(USB_CDC_TX,p,1);
      #else
        Serial.write(p,1);
      #endif
    }
  #else
    UCSR0B |= (1<<UDRIE0); // enable transmitter UDRE interrupt if deactivacted
  #endif
}

#if !defined(PROMICRO)
  ISR_UART {
    uint8_t t = tailTX;
    if (headTX != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR0 = bufTX[t];  // Transmit next byte in the ring
      tailTX = t;
    }
    if (t == headTX) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
#endif
*/