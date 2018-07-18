#include <Wire.h> //for I2C
#include <SPI.h> //for SD card
#include <SD.h> //for SD card
#include <Adafruit_Sensor.h> //for IMU
#include <Adafruit_BNO055.h> //for IMU
#include <Adafruit_GPS.h> //for GPS
#include <TinyGPS.h> //for GPS
//////////////////// XBEE STUFF ////////////////////
#define LED_RED 16
#define LED_GREEN 15
#define LED_BLUE 14
//////////////////// XBEE STUFF ////////////////////
#define XBEESERIAL Serial1
//////////////////// IMU STUFF ////////////////////
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//////////////////// GPS STUFF ////////////////////
#define GPSSERIAL Serial3
TinyGPS gps;
//////////////////// EPM STUFF ////////////////////
#define EPM_PIN 23
////////////////////PROGRAM CONTROL//////////////////
//some constants
#define ACTIVESERIAL XBEESERIAL //serial to use for comms
const String FIRMWARE_VERSION = "1.0";
const String BUILD_DATE = "Dec 30 2015";
const unsigned int MAX_LED_TIME = 3000; //3 s
const unsigned int MAX_DURATION = 36000000; //10 hours
const unsigned int MAX_DELAY = 3600000; //1 hour
const unsigned int EPM_RELEASE = 13; //13/255 = 5% duty cycle at 50Hz PWM = 1ms (0.75 to 1.25 ms is "off")
const unsigned int EPM_NEUTRAL = 19; //19/255 = 7.5% duty cycle at 50Hz PWM = 1.5ms (1.25 to 1.75 ms is "neutral")
const unsigned int EPM_HOLD = 26; //26/255 = 10% duty cycle at 50Hz PWM = 2ms (1.75 to 2.25 ms is "on")
const String HELP_MESSAGE = "DataBubble Version "+FIRMWARE_VERSION+" - "+BUILD_DATE+"\r\n"
 +"commands:\r\n"
 +" help: display this message\r\n"
 +" reset: stops the device and reloads configuration\r\n"
 +" stop: halts any activity, returns to standby mode\r\n"
 +" clear log: deletes the log\r\n"
 +" calibrate: stores the imu calibration\r\n"
 +" save: saves the current configuration\r\n"
 +" start: begins the test cycle\r\n"
 +" led on/off: turns the led on or off\r\n"
 +" magnet on/off: turns the magnet on or off\r\n"
 +" status: display system status message\r\n"
 +" show log: prints the data log\r\n"
 +" show offsets: prints the calibration offsets\r\n"
 +" set debug on/off: when debug is on, information will be logged to console in addition to the logfile\r\n"
 +" set imu mode [0,12]: sets the imu mode, see Adafruit_BNO055 library for more info\r\n"
 +" set led color 1/2/3: sets the led color to red/green/blue\r\n"
 +" set led on/off [0,"+MAX_LED_TIME+"]: sets the led duty cycle in ms\r\n"
 +" set log speed [0,5000]: sets the time between log entries in ms\r\n"
 +" set log commands on/off: if log commands is on, commands will be logged to the logfile\r\n"
 +" set test duration [10000,"+MAX_DURATION+"]: sets the test duration in ms\r\n"
 +" set test delay [0,"+MAX_DELAY+"]: sets delay time before test in ms\r\n"
 +"For more information, consult the user guide";

//state stuff
char state = 0; //0 - initial
unsigned long testStart = 0; //in millis
unsigned long previousLog = 0; //in millis
unsigned long previousLED = 0; //in millis
unsigned long age, date, time = 0;
long lat, lon = 0;
bool debug = false;
bool ballastOn = true;
bool canLog = true;
bool microSDInitialized = false;
//control parameters
String command = "";
unsigned int ledColor = 1; //1=red,2=blue,3=green
unsigned long ledOnTime = 500; //in millis
unsigned long ledOffTime = 500; //in millis
unsigned long logSpeed = 500; //in millis
unsigned long testDuration = 600000; //in millis
unsigned long testDelay = 0; //in millis
uint8_t imuMode = 12; //adafruit_bno055_opmode_t
bool ledOn = false;
bool logCommands = false;
void setup(void)
{
 //start usb thing
 Serial.begin(9600);
 //start xbee radio serial connection
 XBEESERIAL.begin(57600);
 //give serial some time to connect
 delay(2000);

 ACTIVESERIAL.println("starting up...");

 setupMicroSD();
 log("DataBubble Version "+FIRMWARE_VERSION+" - "+BUILD_DATE,true,true);

 state = 0;

 //set up led pins
 pinMode(LED_RED, OUTPUT);
 digitalWrite(LED_RED, LOW);
 pinMode(LED_GREEN, OUTPUT);
 digitalWrite(LED_GREEN, LOW);
 pinMode(LED_BLUE, OUTPUT);
 digitalWrite(LED_BLUE, LOW);
 powerLED(false);
 //set up PWM pins
 analogWriteResolution(8); //goes from 0-255
 analogWriteFrequency(EPM_PIN,50); //sets the PWM period to 20ms
 analogWrite(EPM_PIN,EPM_NEUTRAL);

 readConfig();
 //setupNFC();
 setupIMU();
 loadCalibration();
 setupGPS();
 log("READY", false, true);
}
void loop(void)
{
 //get command, if available
 while(ACTIVESERIAL.available() > 0) {
 command = command + ACTIVESERIAL.readString().toUpperCase();
 if (command.endsWith("\r") || command.endsWith("\n")) {
 command = command.trim();
 log("COMMAND RECEIVED: "+command, logCommands, true);
 parseCommand(command);
 command = "";
 }
 }
 //do the state thing
 unsigned long current = millis();
 switch (state) {
 case 0: {

 }
 break;
 case 1: //pre-test state
 {
 if ((unsigned long)(current - testStart) >= testDelay) {
 state = 2;
 testStart = millis();
 }
 }
 break;
 case 2: //running test, sample motion and log.
 {
 if ((unsigned long)(current - testStart) >= testDuration) {
 //testDuration has been exceeded, release ballast and proceed to next state
 state = 3;
 switchMagnet(false);
 } else if ((unsigned long)(current - previousLog) >= logSpeed) {
 //log a sample
 //sensors_event_t event;
 //bno.getEvent(&event);
 imu::Vector<3> xyz = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
 String dataString = "IMU,"+String(xyz.x());
 dataString = dataString+","+String(xyz.y());
 dataString = dataString+","+String(xyz.z());

 xyz = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
 dataString = dataString+","+String(xyz.x());
 dataString = dataString+","+String(xyz.y());
 dataString = dataString+","+String(xyz.z());

 xyz = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
 dataString = dataString+","+String(xyz.x());
 dataString = dataString+","+String(xyz.y());
 dataString = dataString+","+String(xyz.z());

 xyz = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
 dataString = dataString+","+String(xyz.x());
 dataString = dataString+","+String(xyz.y());
 dataString = dataString+","+String(xyz.z());
 dataString = dataString+","+getStatus();
 log(dataString);
 previousLog = current;
 }
 }
 break;
 case 3: //the test is over, release ballast, cycle LED, and transmit position
 {
 //LED stuff
 if (ledOn) {
 if ((unsigned long)(current - previousLED) >= ledOnTime) {
 previousLED = current;
 ledOn = false;
 }
 } else {
 if ((unsigned long)(current - previousLED) >= ledOffTime) {
 previousLED = current;
 ledOn = true;
 }
 }
 powerLED(ledOn);
 //GPS stuff
 if (GPSSERIAL.available() && gps.encode(GPSSERIAL.read())) {
 gps.get_position(&lat, &lon, &age);
 gps.get_datetime(&date, &time, &age);
 }
 if ((unsigned long)(current - previousLog) >= 1000) {
 String dataString = "GPS,"+String(date);
 dataString = dataString+","+String(time);
 dataString = dataString+","+String(lat);
 dataString = dataString+","+String(lon);
 log(dataString,true,true);
 previousLog = current;
 }
 }
 break;
 default://this should never happen.
 {
 log(F("ERROR: Unknown program state!"));
 }
 break;
 }
}
void parseCommand(String command) {
 //these first few commands are control commands
 if (command.startsWith("H")) {
 log(HELP_MESSAGE,false,true);
 } else if (command == "RESET") {
 setup();
 } else if (command == "STOP") {
 powerLED(false);
 state = 0;
 } else if (command == "CLEAR LOG") {
 SD.remove("datalog.txt");
 } else if (command.startsWith("CAL")) {
 writeCalibration();
 } else if (command == "SAVE") {
 writeConfig();
 } else if (command == "START") {
 state = 1;
 testStart = millis();
 } else if (command == "LED ON") {
 powerLED(true);
 } else if (command == "LED OFF") {
 powerLED(false);
 } else if (command == "MAGNET ON") {
 switchMagnet(true);
 } else if (command == "MAGNET OFF") {
 switchMagnet(false);
 } else if (command.startsWith("STAT")) {
 String stat = getStatus();
 log("STAT,"+stat,false,true);
 } else if (command.startsWith("SHOW")) {
 String var = command.substring(5);
 if (var.startsWith("LOG")) {
 File logFile = SD.open("datalog.txt", FILE_READ);

 if (logFile) {
 while (logFile.available()) {
 String logLine = logFile.readStringUntil('\n').trim();
 ACTIVESERIAL.println(logLine);
 }
 logFile.close();
 } else {
 log(F("LOG FILE NOT FOUND"),false,true);
 }
 } else if (var.startsWith("OFFSETS")) {
 File offsetsFile = SD.open("offsets.txt", FILE_READ);

 if (offsetsFile) {
 while (offsetsFile.available()) {
 String offsetsLine = offsetsFile.readStringUntil('\n').trim();
 ACTIVESERIAL.println(offsetsLine);
 }
 offsetsFile.close();
 } else {
 log(F("CALIBRATION FILE NOT FOUND"),false,true);
 }
 } else {
 log("ERROR: INVALID SHOW COMMAND '"+var+"'",logCommands,true);
 }
 } else if (command.startsWith("SET")) {
 //all the set commands are for changing the configuration
 String var = command.substring(4);
 if (var.startsWith("DEBUG")) {
 String value = var.substring(6);
 if (value == "ON") {
 debug = true;
 } else if (value == "OFF") {
 debug = false;
 } else {
 log("ERROR: INVALID DEBUG OPTION '"+var+"', MUST BE 'ON' OR 'OFF'",logCommands,true);
 }
 } else if (var.startsWith("IMU MODE")) {
 unsigned int value = var.substring(9).toInt();
 if (value < 0 || value > 12) {
 log("ERROR: INVALID IMU MODE '"+String(value)+"', MUST BE IN RANGE [0,12]",logCommands,true);
 } else {
 imuMode = value;
 }
 } else if (var.startsWith("LED")) {
 var = var.substring(4);
 if (var.startsWith("COLOR")) {
 int value = var.substring(6).toInt();
 if (value < 1 || value > 3) {
 log("ERROR: INVALID LED COLOR '"+String(value)+"', MUST BE 1, 2, OR 3",logCommands,true);
 } else {
 ledColor = value;
 }
 } else if (var.startsWith("ON")) {
 unsigned int value = var.substring(3).toInt();
 if (value < 0 || value > MAX_LED_TIME) {
 log("ERROR: LED ON MUST BE BETWEEN 0 AND "+String(MAX_LED_TIME),logCommands,true);
 } else {
 ledOnTime = value;
 }
 } else if (var.startsWith("OFF")) {
 unsigned int value = var.substring(4).toInt();
 if (value < 0 || value > MAX_LED_TIME) {
 log("ERROR: LED OFF MUST BE BETWEEN 0 AND "+String(MAX_LED_TIME),logCommands,true);
 } else {
 ledOffTime = value;
 }
 } else {
 log("ERROR: INVALID LED COMMAND '"+var+"'",logCommands,true);
 }
 } else if (var.startsWith("LOG")) {
 var = var.substring(4);
 if (var.startsWith("SPEED")) {
 unsigned int value = var.substring(6).toInt();
 if (value < 0 || value > 5000) {
 log("ERROR: LOG SPEED MUST BE BETWEEN 0 AND 5000",logCommands,true);
 } else {
 logSpeed = value;
 }
 } else if (var.startsWith("COMMANDS")) {
 var = var.substring(9);
 if (var == "ON") {
 logCommands = true;
 } else if (var == "OFF") {
 logCommands = false;
 } else {
 log("ERROR: INVALID LOG COMMANDS OPTION '"+var+"', MUST BE 'ON' OR 'OFF'",logCommands,true);
 }
 } else {
 log("ERROR: INVALID LOG COMMAND '"+var+"'",logCommands,true);
 }
 } else if (var.startsWith("TEST")) {
 var = var.substring(5);
 if (var.startsWith("DURATION")) {
 unsigned int value = var.substring(9).toInt();
 if (value < 10000 || value > MAX_DURATION) {
 log("ERROR: TEST DURATION MUST BE BETWEEN 10000 AND "+String(MAX_DURATION),logCommands,true);
 } else {
 testDuration = value;
 }
 } else if (var.startsWith("DELAY")) {
 unsigned int value = var.substring(6).toInt();
 if (value < 0 || value > MAX_DELAY) {
 log("ERROR: TEST DELAY MUST BE BETWEEN 0 AND "+String(MAX_DELAY),logCommands,true);
 } else {
 testDelay = value;
 }
 }
 } else {
 log("ERROR: INVALID SET COMMAND '"+var+"'",logCommands,true);
 }
 } else {
 log("ERROR: UNKNOWN COMMAND '"+command+"'",logCommands,true);
 }
}
void loadCalibration() {
 File calibrationFile = SD.open("offsets.txt", FILE_READ);
 adafruit_bno055_offsets_t offsets;
 if (calibrationFile) {
 offsets.accel_offset_x = calibrationFile.parseInt();
 offsets.accel_offset_y = calibrationFile.parseInt();
 offsets.accel_offset_z = calibrationFile.parseInt();
 offsets.gyro_offset_x = calibrationFile.parseInt();
 offsets.gyro_offset_y = calibrationFile.parseInt();
 offsets.gyro_offset_z = calibrationFile.parseInt();
 offsets.mag_offset_x = calibrationFile.parseInt();
 offsets.mag_offset_y = calibrationFile.parseInt();
 offsets.mag_offset_z = calibrationFile.parseInt();
 offsets.accel_radius = calibrationFile.parseInt();
 offsets.mag_radius = calibrationFile.parseInt();

 bno.setSensorOffsets(offsets);
 calibrationFile.close();
 } else {
 ACTIVESERIAL.println(F("CALIBRATION NOT FOUND"));
 }
}
void writeCalibration() {

 adafruit_bno055_offsets_t offsets;

 if (bno.getSensorOffsets(offsets)) {
 SD.remove("offsets.txt");
 File calibrationFile = SD.open("offsets.txt", FILE_WRITE);

 if (calibrationFile) {
 calibrationFile.println(offsets.accel_offset_x);
 calibrationFile.println(offsets.accel_offset_y);
 calibrationFile.println(offsets.accel_offset_z);
 calibrationFile.println(offsets.gyro_offset_x);
 calibrationFile.println(offsets.gyro_offset_y);
 calibrationFile.println(offsets.gyro_offset_z);
 calibrationFile.println(offsets.mag_offset_x);
 calibrationFile.println(offsets.mag_offset_y);
 calibrationFile.println(offsets.mag_offset_z);

 calibrationFile.println(offsets.accel_radius);
 calibrationFile.println(offsets.mag_radius);

 calibrationFile.close();
 } else {
 ACTIVESERIAL.println(F("CALIBRATION NOT FOUND"));
 }
 } else {
 ACTIVESERIAL.println(F("SENSOR IS NOT CALIBRATED"));
 }
}
void readConfig() {
 File configFile = SD.open("config.txt", FILE_READ);
 if (configFile) {
 while (configFile.available()) {
 String configLine = configFile.readStringUntil('\n').trim();
 log(configLine, true, true);
 parseCommand(configLine);
 }
 configFile.close();
 } else {
 ACTIVESERIAL.println(F("CONFIGURATION NOT FOUND"));
 }
}
void writeConfig() {
 SD.remove("config.txt");
 File configFile = SD.open("config.txt", FILE_WRITE);
 if (configFile) {
 configFile.print("SET DEBUG ");
 if (debug) {
 configFile.println("ON");
 } else {
 configFile.println("OFF");
 }
 configFile.println("SET LED COLOR "+String(ledColor));
 configFile.println("SET LED ON "+String(ledOnTime));
 configFile.println("SET LED OFF "+String(ledOffTime));
 configFile.println("SET LOG SPEED "+String(logSpeed));
 configFile.print("SET LOG COMMANDS ");
 if (logCommands) {
 configFile.println("ON");
 } else {
  configFile.println("OFF");
 }
 configFile.println("SET TEST DURATION "+String(testDuration));
 configFile.println("SET TEST DELAY "+String(testDelay));
 configFile.println("SET IMU MODE "+String(imuMode));
 configFile.close();
 } else {
 ACTIVESERIAL.println(F("CONFIGURATION NOT FOUND"));
 }
}
void setupMicroSD() {
 if (!microSDInitialized && !SD.begin(10)) { //sd card 'cs' pin
 ACTIVESERIAL.println(F("MicroSD card failed, or not present"));
 // don't do anything more:
 return;
 }
 microSDInitialized = true;
 log(F("MicroSD initialized."),false,debug);
}
/*
void setupNFC() {
 log(F("Initializing NFC..."));

 nfc.begin();
 uint32_t versiondata = nfc.getFirmwareVersion();
 if (! versiondata) {
 log(F("NFC failed, or not present"));
 // don't do anything more:
 return;
 }
 log("Found chip PN5" + versiondata);

 log(F("NFC initialized."));
}
*/
void setupIMU() {
 log(F("Initializing IMU..."),false,debug);
 Adafruit_BNO055::adafruit_bno055_opmode_t opmode = Adafruit_BNO055::OPERATION_MODE_NDOF;
 switch(imuMode) {
 case 0: opmode = Adafruit_BNO055::OPERATION_MODE_CONFIG; break;
 case 1: opmode = Adafruit_BNO055::OPERATION_MODE_ACCONLY; break;
 case 2: opmode = Adafruit_BNO055::OPERATION_MODE_MAGONLY; break;
 case 3: opmode = Adafruit_BNO055::OPERATION_MODE_GYRONLY; break;
 case 4: opmode = Adafruit_BNO055::OPERATION_MODE_ACCMAG; break;
 case 5: opmode = Adafruit_BNO055::OPERATION_MODE_ACCGYRO; break;
 case 6: opmode = Adafruit_BNO055::OPERATION_MODE_MAGGYRO; break;
 case 7: opmode = Adafruit_BNO055::OPERATION_MODE_AMG; break;
 case 8: opmode = Adafruit_BNO055::OPERATION_MODE_IMUPLUS; break;
 case 9: opmode = Adafruit_BNO055::OPERATION_MODE_COMPASS; break;
 case 10: opmode = Adafruit_BNO055::OPERATION_MODE_M4G; break;
 case 11: opmode = Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF; break;
 case 12: opmode = Adafruit_BNO055::OPERATION_MODE_NDOF; break;
 default:
 log("ERROR: UNKNOWN IMU MODE '"+String(imuMode)+"'");
 }

 if(!bno.begin(opmode)) {
 log(F("IMU failed, or not present"),false,debug);
 return;
 }

 delay(1000);
 bno.setExtCrystalUse(true);
 log("STAT,"+getStatus());

 log(F("IMU initialized."),false,false);
}
void setupGPS() {
 log(F("Initializing GPS..."),false,false);
 //start the serial communication with the GPS
 GPSSERIAL.begin(9600);
 // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
 //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 // uncomment this line to turn on only the "minimum recommended" data for high update rates!
 GPSSERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
 // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
 //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);

 // Set the update rate
 // Note you must send both commands below to change both the output rate (how often the position
 // is written to the serial line), and the position fix rate.
 // 1 Hz update rate
 GPSSERIAL.println(PMTK_SET_NMEA_UPDATE_1HZ);
 GPSSERIAL.println(PMTK_API_SET_FIX_CTL_1HZ);
 // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
 //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
 //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
 // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
 // Note the position can only be updated at most 5 times a second so it will lag behind serial output.
 //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
 //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
 // Request updates on antenna status, comment out to keep quiet
 //GPS.sendCommand(PGCMD_ANTENNA);
 log(F("GPS initialized."),false,false);
}
String getStatus() {
 uint8_t system, gyro, accel, mag;
 system = gyro = accel = mag = 0;
 bno.getCalibration(&system, &gyro, &accel, &mag);
 String dataString = String(system);
 dataString = dataString+","+String(gyro);
 dataString = dataString+","+String(accel);
 dataString = dataString+","+String(mag);
 uint8_t stat, test, err;
 stat = test = err = 0;
 bno.getSystemStatus(&stat, &test, &err);
 dataString = dataString+","+String(stat);
 dataString = dataString+","+String(test);
 dataString = dataString+","+String(err);

 return dataString;
}
void powerLED(bool power) {
 if (power) {
 if (ledColor == 1) {
 digitalWrite(LED_RED, HIGH);
 }
 if (ledColor == 2) {
 digitalWrite(LED_GREEN, HIGH);
 }
 if (ledColor == 3) {
 digitalWrite(LED_BLUE, HIGH);
 }
 } else {
 digitalWrite(LED_RED, LOW);
 digitalWrite(LED_GREEN, LOW);
 digitalWrite(LED_BLUE, LOW);
 }
}
//true = switch magnet on, false = switch magnet off
void switchMagnet(bool magnetOn) {
 if (magnetOn) {
 analogWrite(EPM_PIN, EPM_HOLD);
 } else {
 analogWrite(EPM_PIN, EPM_RELEASE);
 }
 //wait for action
 delay(1000);
 //go back to neutral
 analogWrite(EPM_PIN, EPM_NEUTRAL);
}
void log(String message) {
 log(message, true, false);
}
void log(String message, bool store, bool forceDebug) {
 //prepend with timer value
 message = String(millis())+","+message;
 //spit to serial
 if (debug || forceDebug) {
 ACTIVESERIAL.println(message);
 ACTIVESERIAL.flush();
 }
 if (store) {
 File dataFile = SD.open("datalog.txt", FILE_WRITE);

 // if the file is available, write to it:
 if (dataFile) {
 dataFile.println(message);
 dataFile.close();
 } else if (canLog) {
 canLog = false;
 ACTIVESERIAL.println(F("LOGGING FAILED"));
 }
 }
}
