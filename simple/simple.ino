#include <EEPROM.h>
#include "EEPROMExt.h"
#include <MPU9250.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Wire.h>
#include <DFRobot_SIM7000.h>
#include <DFRobot_SIMsms.h>

#define RESTRICT_PITCH
#define PIN_TX     7
#define PIN_RX     10
#define GETURL     "apps.rahiemy.id/iot2/ins/"
#define GETNUM	   "082245431745"

SoftwareSerial     mySerial(PIN_RX, PIN_TX);
DFRobot_SIM7000    sim7000;
DFRobot_SIMsms     simSMS;

MPU9250 mpu;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanLat; // Create the Kalman instances
Kalman kalmanLong;

int satisfied;
bool b_calibrated = false;
bool messageSent = false;

unsigned int EEP_CALIB_FLAG = 0;
unsigned int EEP_ACC_BIAS = 1;
unsigned int EEP_GYRO_BIAS = 13;
unsigned int EEP_MAG_BIAS = 25;
unsigned int EEP_MAG_SCALE = 37;

float kalAngleX, kalAngleY;
uint32_t timer;

char frame[256];
byte GNSSrunstatus;  //<<<<<<< was char array
byte Fixstatus;           //<<<<<<< was char array
char UTCdatetime[15];
char latitude[10];
char logitude[11];
char altitude[8];
char speedOTG[6];
float fLat, fLong, fSpeed, kXpost, kYpost;
double u = 0;

bool isCalibrated()
{
  return (EEPROM.read(EEP_CALIB_FLAG) == 1);
}

void clearing()
{
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    if (EEPROM.read(i) != 0)                    //skip already "empty" addresses
    {
      EEPROM.write(i, 0);                       //write 0 to address i
    }
  }
  Serial.println("EEPROM erased");
}

void printCalibration() {
  Serial.println("< calibration parameters >");
  Serial.print("calibrated? : ");
  Serial.println(EEPROM.read(EEP_CALIB_FLAG) ? "YES" : "NO");
  Serial.print("acc bias x  : ");
  Serial.println(EEPROM.read(EEP_ACC_BIAS + 0) * 1000.f);
  Serial.print("acc bias y  : ");
  Serial.println(EEPROM.read(EEP_ACC_BIAS + 4) * 1000.f);
  Serial.print("acc bias z  : ");
  Serial.println(EEPROM.read(EEP_ACC_BIAS + 8) * 1000.f);
  Serial.print("gyro bias x : ");
  Serial.println(EEPROM.read(EEP_GYRO_BIAS + 0));
  Serial.print("gyro bias y : ");
  Serial.println(EEPROM.read(EEP_GYRO_BIAS + 4));
  Serial.print("gyro bias z : ");
  Serial.println(EEPROM.read(EEP_GYRO_BIAS + 8));
  Serial.print("mag bias x  : ");
  Serial.println(EEPROM.read(EEP_MAG_BIAS + 0));
  Serial.print("mag bias y  : ");
  Serial.println(EEPROM.read(EEP_MAG_BIAS + 4));
  Serial.print("mag bias z  : ");
  Serial.println(EEPROM.read(EEP_MAG_BIAS + 8));
  Serial.print("mag scale x : ");
  Serial.println(EEPROM.read(EEP_MAG_SCALE + 0));
  Serial.print("mag scale y : ");
  Serial.println(EEPROM.read(EEP_MAG_SCALE + 4));
  Serial.print("mag scale z : ");
  Serial.println(EEPROM.read(EEP_MAG_SCALE + 8));
}

void loadCalibration() {
  if (EEPROM.read(EEP_CALIB_FLAG) == 1)
  {
    Serial.println("calibrated? : YES");
    Serial.println("load calibrated values");
    mpu.setAccBias(0, EEPROM.read(EEP_ACC_BIAS + 0));
    mpu.setAccBias(1, EEPROM.read(EEP_ACC_BIAS + 4));
    mpu.setAccBias(2, EEPROM.read(EEP_ACC_BIAS + 8));
    mpu.setGyroBias(0, EEPROM.read(EEP_GYRO_BIAS + 0));
    mpu.setGyroBias(1, EEPROM.read(EEP_GYRO_BIAS + 4));
    mpu.setGyroBias(2, EEPROM.read(EEP_GYRO_BIAS + 8));
    mpu.setMagBias(0, EEPROM.read(EEP_MAG_BIAS + 0));
    mpu.setMagBias(1, EEPROM.read(EEP_MAG_BIAS + 4));
    mpu.setMagBias(2, EEPROM.read(EEP_MAG_BIAS + 8));
    mpu.setMagScale(0, EEPROM.read(EEP_MAG_SCALE + 0));
    mpu.setMagScale(1, EEPROM.read(EEP_MAG_SCALE + 4));
    mpu.setMagScale(2, EEPROM.read(EEP_MAG_SCALE + 8));
  }
  else
  {
    Serial.println("calibrated? : NO");
    Serial.println("load default values");
    mpu.setAccBias(0, +0.005);
    mpu.setAccBias(1, -0.008);
    mpu.setAccBias(2, -0.001);
    mpu.setGyroBias(0, +1.5);
    mpu.setGyroBias(1, -0.5);
    mpu.setGyroBias(2, +0.7);
    mpu.setMagBias(0, +186.41);
    mpu.setMagBias(1, -197.91);
    mpu.setMagBias(2, -425.55);
    mpu.setMagScale(0, +1.07);
    mpu.setMagScale(1, +0.95);
    mpu.setMagScale(2, +0.99);
  }
}

void saveCalibration() {
  EEPROM.write(EEP_CALIB_FLAG, 1);
  EEPROM.put(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
  EEPROM.put(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
  EEPROM.put(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
  EEPROM.put(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
  EEPROM.put(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
  EEPROM.put(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
  EEPROM.put(EEP_MAG_BIAS + 0, mpu.getMagBias(0));
  EEPROM.put(EEP_MAG_BIAS + 4, mpu.getMagBias(1));
  EEPROM.put(EEP_MAG_BIAS + 8, mpu.getMagBias(2));
  EEPROM.put(EEP_MAG_SCALE + 0, mpu.getMagScale(0));
  EEPROM.put(EEP_MAG_SCALE + 4, mpu.getMagScale(1));
  EEPROM.put(EEP_MAG_SCALE + 8, mpu.getMagScale(2));
  Serial.println("save function on work!");
}

void setupEEPROM()
{
  b_calibrated = isCalibrated();
  Serial.println("EEPROM start");
  if (!b_calibrated)
  {
    Serial.println("Need Calibration!!");
    mpu.calibrateMag();
    mpu.calibrateAccelGyro();
    saveCalibration();
  }
  Serial.println("EEPROM calibration value is : ");
  printCalibration();
  Serial.println("Loaded calibration value is : ");
  loadCalibration();
}

int8_t get_GPS() { //THX FOR SOMEONE ON ARDUINO FORUM MADE ME HAVE THIS IDEA!!!//

  int8_t counter, answer;
  long previous;
	
  // First get the NMEA string
  // Clean the input buffer
  while ( mySerial.available() > 0) mySerial.read();
  // request Basic string
  mySerial.println("AT+CGNSINF"); //sendATcommand("AT+CGNSINF", "AT+CGNSINF\r\n\r\n", 2000);

  counter = 0;
  answer = 0;
  memset(frame, '\0', sizeof(frame));    // Initialize the string
  previous = millis();
  // this loop waits for the NMEA string
  do {

    if (mySerial.available() != 0) {
      frame[counter] = mySerial.read();
      counter++;
      // check if the desired answer is in the response of the module
      if (strstr(frame, "OK") != NULL)
      {
        answer = 1;
      }
    }
    // Waits for the asnwer with time out
  }
  while ((answer == 0) && ((millis() - previous) < 2000));

  frame[counter - 3] = '\0';

  // Parses the string
  strtok(frame, ",");
  strcpy(Fixstatus, strtok(NULL, ",")); // Gets GNSS run status
  strcpy(UTCdatetime, strtok(NULL, ",")); // Gets Fix status
  memcpy(latitude, strtok(NULL, ","), 7); // Gets UTC date & time
  memcpy(logitude, strtok(NULL, ","), 9); // Gets longitude
  strcpy(altitude, strtok(NULL, ",")); // Gets MSL altitude
  strcpy(speedOTG, strtok(NULL, ",")); // Gets speed over ground
}

void setupModuleSim() {
  int signalStrength, dataNum;
  Serial.begin(115200);
  while (!Serial);
  sim7000.begin(mySerial);
  Serial.println("Turn ON SIM7000......");
  if (sim7000.turnON()) {                                        //Turn ON SIM7000
    Serial.println("Turn ON !");
  }

  Serial.println("Set baud rate......");
  while (1) {
    if (sim7000.setBaudRate(19200)) {                          //Set SIM7000 baud rate from 115200 to 19200 reduce the baud rate to avoid distortion
      Serial.println("Set baud rate:19200");
      break;
    } else {
      Serial.println("Failed to set baud rate");
      delay(1000);
    }
  }

  Serial.println("Check SIM card......");
  if (sim7000.checkSIMStatus()) {                                //Check SIM card
    Serial.println("SIM card READY");
  } else {
    Serial.println("SIM card ERROR, Check if you h ave insert SIM card and restart SIM7000");
    while (1);
  }

  Serial.println("Set net mode......");
  while (1) {
    if (sim7000.setNetMode(GPRS)) {                      //Set net mod GPRS
      Serial.println("Set GPRS mode");
      break;
    } else {
      Serial.println("Fail to set mode");
      delay(1000);
    }
  }

  Serial.println("Get signal quality......");
  signalStrength = sim7000.checkSignalQuality();           //Check signal quality from (0-30)
  Serial.print("signalStrength =");
  Serial.println(signalStrength);
  delay(500);

  Serial.println("Init positioning function......");
  while (1) {
    if (sim7000.initPos()) {
      Serial.println("Positioning function initialized");
      break;
    } else {
      Serial.println("Fail to init positioning function");
      delay(1000);
    }
  }

  Serial.println("Attaching service......");
  while (1) {
    if (sim7000.attacthService()) {                      //Open the connection
      Serial.println("Attach service");
      break;
    } else {
      Serial.println("Fail to Attach service");
      delay(1000);
    }
  }

  Serial.println("Init http......");
  while (1) {
    if (sim7000.httpInit(GPRS)) {                        //Init http service
      Serial.println("HTTP init !");
      break;
    } else {
      Serial.println("Fail to init http");
    }
  }
}

void sendData(double &dt) {
  get_GPS();
  double *deltaT = &dt;
  //timer = micros();
  fLat = atof (latitude);
  fLong = atof (logitude);
  fSpeed = atof (speedOTG);
  kXpost = kalmanLong.getAngle(fLong, fSpeed, *deltaT);
  kYpost = kalmanLat.getAngle(fLat, fSpeed, *deltaT);
  
  delay(1000);

  if (deAcc(dt,fSpeed) == false && fallen(kalAngleX, kalAngleY) == false && kXpost != 0 && kYpost != 0) {                                   //Get the current position
    Serial.print(" Latitude : ");
    Serial.println(kYpost, 8);                    //Get longitude
    Serial.print(" Longitude : ");
    Serial.println(kXpost, 8);                     //Get latitude
    String latlng = String();
	latlng += "1";
	latlng += "/";
	latlng += String(fLat, 7);
	latlng += "/";
	latlng += String(fLong, 7);
	latlng += "/";
    latlng += String(kYpost,8);
    latlng += "/";
    latlng += String(kXpost,7);
	latlng += "/";
	latlng += "0";
    Serial.println("Getting position......");
    String surl = GETURL + latlng;
    int len = surl.length() + 1;
    char senturl[len];
    surl.toCharArray(senturl, len);
    Serial.println(senturl);
    sim7000.httpGet(senturl);
    
 Serial.print(kalAngleX);
 Serial.println(kalAngleY);
  }
  else if (deAcc(dt,fSpeed) == false && fallen(kalAngleX, kalAngleY) == true
  && simSMS.sendSMS() == false && kXpost != 0 && kYpost != 0 ) {
    
	Serial.println("Getting position for sms......CRASH");
    String latlngS = String();
    latlngS += "Kecelakaan Tabrakan atau jatuh";
	latlngS += " ";
    latlngS += "Link Lokasi: ";
	latlngS += "https://www.google.com/maps?saddr=My+Location&daddr=";
	latlngS += String(kYpost,7);
	latlngS += ",";
	latlngS += String(kXpost,7);
    String smsM = latlngS;
    String phoneN = GETNUM;
    int lenS = smsM.length() + 1;
    int lenP = phoneN.length() + 1;
    char sentSms[lenS];
    char phoneNumber[lenP];
    smsM.toCharArray(sentSms, lenS);
    phoneN.toCharArray(phoneNumber, lenP);
    simSMS.beginSMS(phoneNumber);
    simSMS.editSMS(sentSms);
    simSMS.sendSMS();
	
	Serial.println("Getting position for OTA......");
	Serial.print(" Latitude : "); 					//send data OTA
    Serial.println(kYpost, 8);                    //Get longitude
    Serial.print(" Longitude : ");
    Serial.println(kXpost, 8);                     //Get latitude
    String latlng = String();
	latlng += "1";
	latlng += "/";
	latlng += String(fLat, 7);
	latlng += "/";
	latlng += String(fLong, 7);
	latlng += "/";
    latlng += String(kYpost,7);
    latlng += "/";
    latlng += String(kXpost,7);
	latlng += "/";
	latlng += "1";
	
	String surl = GETURL + latlng;
    int len = surl.length() + 1;
    char senturl[len];
    surl.toCharArray(senturl, len);
    Serial.println(senturl);
    sim7000.httpGet(senturl);
	messageSent = true;
 Serial.print(kalAngleX);
 Serial.println(kalAngleY);
	
  }/* else if (deAcc(dt,fSpeed) == true && fallen(kalAngleX, kalAngleY) ==  true
  && simSMS.sendSMS() == false && kXpost != 0 && kYpost != 0 && fSpeed == 0) {
    Serial.println("Getting position for sms...... FALL");
    String latlngS = String();
    latlngS += "Kecelakaan jatuh";
    latlngS += " ";
    latlngS += "Link Lokasi: ";
	latlngS += "https://www.google.com/maps?saddr=My+Location&daddr=";
	latlngS += String(kYpost,7);
	latlngS += ",";
	latlngS += String(kXpost,7);
    String smsM = latlngS;
    String phoneN = GETNUM;
    int lenS = smsM.length() + 1;
    int lenP = phoneN.length() + 1;
    char sentSms[lenS];
    char phoneNumber[lenP];
    smsM.toCharArray(sentSms, lenS);
    phoneN.toCharArray(phoneNumber, lenP);
    simSMS.beginSMS(phoneNumber);
    simSMS.editSMS(sentSms);
    simSMS.sendSMS();
	
	Serial.println("Getting position for OTA......");
	Serial.print(" Latitude : "); 					//send data OTA
    Serial.println(kYpost, 8);                    //Get longitude
    Serial.print(" Longitude : ");
    Serial.println(kXpost, 8);                     //Get latitude
    String latlng = String();
	latlng += "1";
	latlng += "/";
	latlng += String(fLat, 7);
	latlng += "/";
	latlng += String(fLong, 7);
	latlng += "/";
    latlng += String(kYpost,7);
    latlng += "/";
    latlng += String(kXpost,7);
	latlng += "/";
	latlng += "2";
	
	String surl = GETURL + latlng;
    int len = surl.length() + 1;
    char senturl[len];
    surl.toCharArray(senturl, len);
    Serial.println(senturl);
    sim7000.httpGet(senturl);
	messageSent = true;
	
  }*/ else if (fSpeed > 10 && fallen(kalAngleX, kalAngleY) == true 
  && simSMS.sendSMS() == false && kXpost != 0 && kYpost != 0) {
    Serial.println("Getting position for sms...... CRASH HIGH VELOCITY");
    String latlngS = String();
    latlngS += "Kecelakaan jatuh HIGH VELOCITY";
    latlngS += " ";
    latlngS += "Link Lokasi: ";
	latlngS += "https://www.google.com/maps?saddr=My+Location&daddr=";
	latlngS += String(kYpost,7);
	latlngS += ",";
	latlngS += String(kXpost,7);
    String smsM = latlngS;
    String phoneN = GETNUM;
    int lenS = smsM.length() + 1;
    int lenP = phoneN.length() + 1;
    char sentSms[lenS];
    char phoneNumber[lenP];
    smsM.toCharArray(sentSms, lenS);
    phoneN.toCharArray(phoneNumber, lenP);
    simSMS.beginSMS(phoneNumber);
    simSMS.editSMS(sentSms);
    simSMS.sendSMS();
	
	Serial.println("Getting position for OTA......");
	Serial.print(" Latitude : "); 					//send data OTA
    Serial.println(kYpost, 8);                    //Get longitude
    Serial.print(" Longitude : ");
    Serial.println(kXpost, 8);                     //Get latitude
    String latlng = String();
	latlng += "1";
	latlng += "/";
	latlng += String(fLat, 7);
	latlng += "/";
	latlng += String(fLong, 7);
	latlng += "/";
    latlng += String(kYpost,7);
    latlng += "/";
    latlng += String(kXpost,7);
	latlng += "/";
	latlng += "3";
	String surl = GETURL + latlng;
    int len = surl.length() + 1;
    char senturl[len];
    surl.toCharArray(senturl, len);
    Serial.println(senturl);
    sim7000.httpGet(senturl);
	messageSent = true;
	
  } else
  {
    Serial.println("Wrong data try again");
  }

}

bool deAcc(double &dt, float &speed) {
  float a = mpu.getAccX();
  float b = mpu.getAccY();
  float c = mpu.getAccZ();
  float d = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
  if (&speed == 0 && d < (2)) {
	return true;
  } else {
    return false;
  }
}

bool fallen(float angleX, float angleY) {
  if (angleX >= 20 || angleX <= -20) {
    return true;
  } else if (angleY >= 20 || angleY <= -20){
    return true;
  }
  else {
	  return false;
  }
}

/*THIS IS WHERE ARDUINO START ROCK'IN !!!!!!!!!!!!!!!*/

void setup()
{
  setupModuleSim();
  Serial.begin(115200);
  Wire.begin();
  Serial.print("If satisfied using calibrated data press 1, if not satisfied or not yet calibrate Press 2");
  delay(5000);
  satisfied = Serial.read();
  Serial.println(satisfied);
  delay(5000);
  if (satisfied != 50) {
    mpu.setup();
    setupEEPROM();
  }
  else
  {
    mpu.setup();
    clearing();
    setupEEPROM();
  }
  delay(8000);
  //timer = micros();
  mpu.update();
  get_GPS();

  fLat = atof (latitude);
  fLong = atof (logitude);
  float roll = mpu.getRoll();
  float pitch = mpu.getPitch();
  float a = mpu.getAccX();
  float b = mpu.getAccY();
  float c = mpu.getAccZ();
  float d = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));


  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  kalmanX.setQbias(184);
  kalmanY.setQbias(72);

  kalmanLat.setAngle(fLat); kalmanLat.setQbias(0.00003);
  kalmanLong.setAngle(fLong); kalmanLong.setQbias(0.00003);

  delay(0);
}


void loop()
{
    static uint32_t period = 0.5 * 60000;
	uint32_t tStart = millis();
  double dt = (double)(micros() - timer) / 1000000;
	while ((millis() - tStart) < period)
	{
		mpu.update();
		timer = micros();
		double dt = (double)(micros() - timer) / 1000000; // Calculate delta timer
		float rolls = mpu.getRoll();
		float pitchs = mpu.getPitch();
		float gyroXrate = mpu.getGyroX();
		float gyroYrate = mpu.getGyroY();
		kalAngleX = kalmanX.getAngle(rolls, gyroXrate, dt);
		kalAngleY = kalmanY.getAngle(pitchs, gyroXrate, dt);

		if (messageSent == true) {
			messageSent = false;
			//delay(10000);
		}
	}
	sendData(dt);
  /* Serial.print(kalAngleX);
   Serial.println(kalAngleY);*/
}
