/*====================================================================================//

    Seismometer, firmware version 2.0
    Updated 2017-06-25
    Hardware design and assembly by JM Rautenberg
    Firmware by JM Rautenberg
    This firmware version works with PCB version 2.0

    This program continuously records accelerations in a circular pre-buffer in three
    orthogonal directions using an ADXL355 accelerometer. Data is saved to the on-board
    micro-SD card after accelerations are detected above a user-set threshold value.
    In order to operate to it's fullest functionality, the user should include a file
    named "SETTINGS.IN", which allows the user to select a number of settings and
    populate the header of the output files. See separate "READ_ME.TXT" file for further
    description of settings file requirements.

    Acceleration values from both the circular buffer and after a triggering event are
    stored in three on-board SRAM chips, which provide faster read/write access than the
    microSD card. The SRAM chips can store up to 43,690 (x,y,z) acceleration point sets
    before space on the SRAMs is exhausted, whereater it dumps the results to the microSD
    card. At 200Hz, this equates to a maximum record length of about 3:38 (m:ss).

    On-board LEDs (RED and GREEN) are used to inform the user what the unit is doing
    at certain times. The following sequence of events occurs in the unit:
      Power up
      Delay one second
      RED 'on' followed by GREEN 'on'
        Delay two seconds
      GREEN 'off' followed by RED 'off'
      Delay two seconds
      Import settings from "SETTINGS.IN" file on micro-SD card
      RED 'on'
        Compute zero values in accelrometer, approx. 5 seconds
      RED 'off'
      GREEN quick blink after each loop through the pre-buffer (default 15s)
      [Acceleration exceeds threshold value]
      RED 'on' (GREEN 'off', though it is unlikely that the GREEN LED is on at this time)
        Record accelerations to SRAM until [post-buffer duration] seconds after last
        excursion past the threshold
      GREEN 'on'
        Creat output file on micro-SD card and write acceleration data
      GREEN 'off'
        Re-compute zero values in accelerometer, approx. 5 seconds
      RED 'off'
      GREEN quick blink after each loop through the pre-buffer (default 15s)
      etc.

    If the user pushes the 'Zero' button (note: this only functions while recording to SRAM,
    either to pre-buffer or during motion):
      RED and GREEN 'off'
      RED and GREEN double blink together five times
      RED 'on'
        Re-compute zero values in accelerometer, approx. 5 seconds
      RED 'off'
      GREEN quick blink after each loop through the pre-buffer (default 15s)
      etc.

    If the user pushes the 'Reset' button, the unit reboots as if power cycled; see above.

    Separate programs are required for (1) pre-populating the EEPROM of the ATMEGA328P
    with the baseline header information for the output files, and (2) setting the real-
    time clock.

//====================================================================================*/


//==================================== LIBRARIES =====================================//
#include <Wire.h>                                   //  I2C library
#include <SPI.h>                                    //  SPI library
#include <SD.h>                                     //  SD card library
#include <EEPROM.h>                                 //  EEPROM library (non-volitile storage of file no., header info)
//====================================================================================//


//=============================== GENERAL DEFINITIONS ================================//
#define SD_DETECT       8                           //  Pin PB0 (14)   Pin reading to see if microSD card is present in slot
#define CS_ADXL         9                           //  Pin PB1 (15)   ADXL355 chip select pin
#define CS_SD           10                          //  Pin PB2 (16)   MicroSD Card reader/writer chip select pin
#define CS_SRAM3        A1                          //  Pin PC1 (24)   23LC1024 SRAM chip 3 select pin
#define CS_SRAM2        A2                          //  Pin PC2 (25)   23LC1024 SRAM chip 2 select pin
#define CS_SRAM1        A3                          //  Pin PC3 (26)   23LC1024 SRAM chip 1 select pin
#define ZERO            2                           //  Pin PD2 ( 4)   Pin used to determine if 'ZERO' button is pressed
#define GREEN_LED       3                           //  Pin PD3 ( 5)   Pin controlling green LED
#define RED_LED         4                           //  Pin PD4 ( 6)   Pin controlling red LED

#define SRAMSpeed       8.00 * 1e6                  //  SPI speed for SRAM read/write operations (16 kHz = full speed)
#define ADXLSpeed       8.00 * 1e6                  //  SPI speed for ADXL read operations (16 kHz = full speed)
#define SDSpeed         SPI_CLOCK_DIV2              //  SPI speed for SD read/write operations
#define NZeroPoints     200                         //  Number of points to take for zeroing of accelerometer axes
#define recordTimestamp 0                           //  Record 4-byte timestamp into SRAM and onto SD card (0 or 1)
#define waveform        0                           //  Enable debugging waveform plotter (use Serial Plotter in Arduino IDE when set to 1)
#define settingsName    "SETTINGS.IN"               //  Name of file on SD card containing the user-selectable settings
#define points          24                          //  Maximum number of characters in each half of a header line
#define lines           40                          //  Number of header lines (x 2)
//====================================================================================//


//================================= ADXL DEFINITIONS =================================//
#define ADXL_READ       0b00000001                  //  Read instruction for ADXL313
#define ADXL_WRITE      0b00000000                  //  Write instruction for ADXL313
#define DEVID_AD        0x00                        //  R      0b 1010 1101    Fixed device ID 1 (should output  AD in HEX)
#define DEVID_MST       0x01                        //  R      0b 0001 1101    Fixed device ID 2 (should output  1D in HEX)
#define PARTID          0x02                        //  R      0b 1110 1101    Part ID           (should output 355 in OCT)
#define REVID           0x03                        //  R      0b 0000 0000    Silicon revision no.
#define TEMP2           0x06                        //  R      0b 0000 0000    Temperature bits 8 to 11
#define XDATA3          0x08                        //  R      0b 0000 0000    Bits 12 to 19 of x-accel
#define FILTER          0x28                        //  R/W    0b 0000 0000    Internal high- and low-pass filters
#define RANGE           0x2C                        //  R/W    0b 1000 0001    I2C disable, interrupt polarity, measurement range
#define POWER_CTL       0x2D                        //  R/W    0b 0000 0001    Power control

#define accelLen        3                           //  Number of bytes per acceleration recording value
#define N_Axes          3                           //  Number of accelerometer axes (X, Y, Z)
#define INTERCEPT       1972.0                      //  Intercept of temperature (C) vs. LSB line for ADXL (rough calibrated)
#define SLOPE            -11.1                      //  Slope of temperature (C) vs. LSB line for ADXL     (rough calibrated)
//#define INTERCEPT       1852.0                      //  Intercept of temperature (C) vs. LSB line for ADXL (per datasheet)
//#define SLOPE            -9.05                      //  Slope of temperature (C) vs. LSB line for ADXL     (per datasheet)
//====================================================================================//


//================================= SRAM DEFINITIONS =================================//
#define SRAM_WRITE      0b00000010                  //  Write command for SRAM chip
#define SRAM_READ       0b00000011                  //  Read command for SRAM chip
#define SRAM_RDSR       0b00000101                  //  Registry Read command for SRAM chip
#define SRAM_WRSR       0b00000001                  //  Registry Write command for SRAM chip
#define SRAM_BYTE       0b00000000                  //  Read/write SRAM in Byte mode
#define SRAM_PAGE       0b10000000                  //  Read/write SRAM in Page mode
#define SRAM_SEQ        0b01000000                  //  Read/write SRAM in Sequential mode
#define SRAM_LENGTH     0x20000                     //  Total number of bytes in each SRAM chip (0x20000 = 131,072 BYTES = 1,048,576 BITS)
#define N_SRAM          3                           //  Number of SRAM chips used
//====================================================================================//


//================================= RTC DEFINITIONS ==================================//
#define RTCADDR         0b1101111                   //  I2C address of real time clock
#define CONTROL         0x07                        //  Control register address
#define OSCTRIM         0x08                        //  Digital trim control register address
#define RTCWKDAY        0x03                        //  First address of date/time info register
#define RTCSEC          0x00                        //  Second address of date/time info register
//====================================================================================//


//================================= GLOBAL VARIABLES =================================//
bool        SDDetectFlag,
            ZeroFlag,
            bufferLooped = 0;
uint8_t     CS_SRAM[N_SRAM],
            timeLen,
            bytesPerLine,
            rtcYear,
            rtcMonth,
            rtcDay,
            rtcHours,
            rtcMinutes,
            rtcSeconds,
            rtcWeekDay,
            accel_range,
            recordingRate,
            preBuffer,
            postBuffer,
            rtcTrimmer,
            ADXL_Filter;
char        currentOutputFile[13],
            latitude   [points],
            longitude  [points],
            elevation  [points],
            location_1 [points],
            location_2 [points],
            orientation[points];
uint16_t    fileNo,
            preBufferPoints,
            postBufferPoints,
            triggerMG;
uint32_t    lastPreBufferAddr,
            scaleXYZ;
int32_t     accel_zero[N_Axes],
            triggerLSB;
float       readDelay;
SPISettings ADXLSettings (ADXLSpeed, MSBFIRST, SPI_MODE0),
            SRAMSettings (SRAMSpeed, MSBFIRST, SPI_MODE0);
//====================================================================================//


//================================== SETUP FUNCTION ==================================//
void setup() {

  delay(2000);

  pinMode(GREEN_LED,  OUTPUT);    digitalWrite(GREEN_LED, LOW);
  pinMode(RED_LED,    OUTPUT);    digitalWrite(RED_LED,   LOW);
  pinMode(ZERO,       INPUT );

  delay(1000);
  digitalWrite(RED_LED,   HIGH);  delay(250);
  digitalWrite(GREEN_LED, HIGH);  delay(2000);
  digitalWrite(GREEN_LED, LOW );  delay(250);
  digitalWrite(RED_LED,   LOW );  delay(2000);


  CS_SRAM[0] = CS_SRAM1;
  CS_SRAM[1] = CS_SRAM2;
  CS_SRAM[2] = CS_SRAM3;
  if (recordTimestamp)  timeLen = 4;
  else                  timeLen = 0;
  bytesPerLine = timeLen + N_Axes * accelLen;


  initialize_Serial();                                            //  Initialize serial monitor
  initialize_SPI();                                               //  Initialize SPI protocol
  read_settings_file();

  setup_ADXL();                                                   //  Setup ADXL355 registers
  setup_SRAM();                                                   //  Setup registers of all three 23LC1024 SRAM ICs
  setup_RTC();                                                    //  Setup registers RTC IC via I2C bus
//  if (!waveform) {
//    readID_ADXL();                                                //  Read ADXL355 ID info
//  }

  fileNo = ((uint16_t)EEPROM.read(0) << 8) | EEPROM.read(1);      //  Determine what the current fileNo is (stored in non-volitile memory)
  if (!waveform) {
    Serial.print("Last file: ");
    Serial.println(fileNo, DEC);
  }

  digitalWrite(RED_LED, HIGH);
  clear_SRAM();
  zero_ADXL();                                                    //  Compute zero-values for accelerometer
  digitalWrite(RED_LED, LOW);


  if (waveform) {
    int32_t accel_xyz[3];
    while (1) {
      read_ADXL(accel_xyz);
//      Serial.print(              1.0 * scaleXYZ); Serial.print('\t');
//      Serial.print(             -1.0 * scaleXYZ); Serial.print('\t');
      Serial.print(                  triggerLSB); Serial.print('\t');
      Serial.print(                 -triggerLSB); Serial.print('\t');
      Serial.print(accel_xyz[0] - accel_zero[0]); Serial.print('\t');
      Serial.print(accel_xyz[1] - accel_zero[1]); Serial.print('\t');
      Serial.print(accel_xyz[2] - accel_zero[2]); Serial.println();
      if (trigger_exceeded(accel_xyz)) {
        digitalWrite(RED_LED, HIGH); digitalWrite(GREEN_LED, HIGH);
        delay(2);
        digitalWrite(RED_LED,  LOW); digitalWrite(GREEN_LED,  LOW);
      }
    }
  }

  attachInterrupt(digitalPinToInterrupt(ZERO), intFunction, FALLING);
  ZeroFlag = 0;

}
//====================================================================================//


//==================================== MAIN LOOP =====================================//
void loop() {

  uint16_t  preBufferPoint  = 0,
            postBufferPoint = 0,
            EQPointCounter  = 0,
            maxLines        = (uint16_t)((N_SRAM * (uint32_t)SRAM_LENGTH - 1) / bytesPerLine);
  uint32_t  clockStart      = 0,
            clockNow        = 0,
            currentMillis   = 0,
            lastMillis      = 0,
            loopCounter     = 0,
            startMillis;
  int32_t   accel_xyz[N_Axes];

  clockStart  = millis();
  startMillis = clockStart;

  while (1) {

    if (ZeroFlag) {
      digitalWrite(RED_LED,   LOW );
      digitalWrite(GREEN_LED, LOW );
      delay(1000);
      for (uint8_t i = 0; i < 5; i++) {
        digitalWrite(RED_LED, HIGH); digitalWrite(GREEN_LED, HIGH);
        delay(100);
        digitalWrite(RED_LED,  LOW); digitalWrite(GREEN_LED,  LOW);
        delay(100);
        digitalWrite(RED_LED, HIGH); digitalWrite(GREEN_LED, HIGH);
        delay(100);
        digitalWrite(RED_LED,  LOW); digitalWrite(GREEN_LED,  LOW);
        delay(1000);
      }
      digitalWrite(RED_LED, HIGH);
      ZeroFlag = 0;
      EIFR = bit (INTF0);
      goto ZeroPush;
    }

    //  Check to see if millis() rolls over; should happen every 2^32ms, or a hair under 50 days.
    clockNow = millis();
    if (clockNow < clockStart) {
      clockStart  = clockNow;
      startMillis = clockStart;
      loopCounter = 0;
      lastMillis  = 0;
    }

    currentMillis = clockNow - startMillis;

    if (((float)currentMillis / readDelay > (float)loopCounter) && (currentMillis > lastMillis)) {

      read_ADXL(accel_xyz);

      if (trigger_exceeded(accel_xyz)) {
        if (postBufferPoint == 0)
          digitalWrite(RED_LED, HIGH);
        postBufferPoint = 1;
      }

      if (postBufferPoint == 0) {
        write_dataset_to_SRAM(accel_xyz, currentMillis, (uint32_t)preBufferPoint);
        if (preBufferPoint < (preBufferPoints - 1))
          preBufferPoint++;
        else {
          digitalWrite(GREEN_LED, HIGH);
          preBufferPoint = 0;
          delay(2);
          digitalWrite(GREEN_LED, LOW);
          bufferLooped = 1;
        }
        lastMillis = currentMillis;
        loopCounter++;
      } else if ((postBufferPoint < postBufferPoints) & ((preBufferPoints + EQPointCounter) < maxLines)) {
        write_dataset_to_SRAM(accel_xyz, currentMillis, (uint32_t)(EQPointCounter + preBufferPoints));
        postBufferPoint++;
        lastMillis = currentMillis;
        loopCounter++;
        EQPointCounter++;
      } else {
        digitalWrite(GREEN_LED, HIGH);
        
        
        incrementFile();
        writeHeader  (preBufferPoint, EQPointCounter);
        outputLogFile(preBufferPoint, EQPointCounter);
//        writeHeader(EQPointCounter + preBufferPoints);                        // Sends too many if buffer hasn't been looped through once.
//        outputLogFile(preBufferPoint, EQPointCounter + preBufferPoints);
        ZeroPush:
        digitalWrite(GREEN_LED, LOW);
        postBufferPoint   = 0;
        loopCounter       = 0;
        EQPointCounter    = 0;
        preBufferPoint    = 0;
        lastMillis        = 0;
        bufferLooped      = 0;
        clear_SRAM();
        zero_ADXL();
        digitalWrite(RED_LED, LOW);
        startMillis = millis();
        }
      }
  }
}
//====================================================================================//


//=================== READ SETTINGS FILE OFF SD CARD, IF AVAILABLE ===================//
void read_settings_file() {

  String   stringBuffer;
  bool     error_flag;
  char     charBuffer[points];
  uint16_t intBuffer;

  SD.begin(CS_SD);
  SPI.setClockDivider(SDSpeed);
  File myFile;
  myFile = SD.open(settingsName, FILE_READ);

  if (myFile) {
    while (myFile.available()) {

      //LINE 1: LATITUDE
      stringBuffer = myFile.readStringUntil('\n');
      stringBuffer.toCharArray(charBuffer, points);
      for (uint8_t i = 0; i < (stringBuffer.length() - 1); i++) {
        latitude[i] = charBuffer[i];
      }
      latitude[stringBuffer.length() - 1] = '\0';

      //LINE 2: LONGITUDE
      stringBuffer = myFile.readStringUntil('\n');
      stringBuffer.toCharArray(charBuffer, points);
      for (uint8_t i = 0; i < (stringBuffer.length() - 1); i++) {
        longitude[i] = charBuffer[i];
      }
      longitude[stringBuffer.length() - 1] = '\0';

      //LINE 3: ELEVATION
      stringBuffer = myFile.readStringUntil('\n');
      stringBuffer.toCharArray(charBuffer, points);
      for (uint8_t i = 0; i < (stringBuffer.length() - 1); i++) {
        elevation[i] = charBuffer[i];
      }
      elevation[stringBuffer.length() - 1] = '\0';

      //LINE 4: ORIENTATION
      stringBuffer = myFile.readStringUntil('\n');
      stringBuffer.toCharArray(charBuffer, points);
      for (uint8_t i = 0; i < (stringBuffer.length() - 1); i++) {
        orientation[i] = charBuffer[i];
      }
      orientation[stringBuffer.length() - 1] = '\0';

      //LINE 5: LOCATION DESCRIPTION 1
      stringBuffer = myFile.readStringUntil('\n');
      stringBuffer.toCharArray(charBuffer, points);
      for (uint8_t i = 0; i < (stringBuffer.length() - 1); i++) {
        location_1[i] = charBuffer[i];
      }
      location_1[stringBuffer.length() - 1] = '\0';

      //LINE 6: LOCATION DESCRIPTION 2
      stringBuffer = myFile.readStringUntil('\n');
      stringBuffer.toCharArray(charBuffer, points);
      for (uint8_t i = 0; i < (stringBuffer.length() - 1); i++) {
        location_2[i] = charBuffer[i];
      }
      location_2[stringBuffer.length() - 1] = '\0';

      //LINE 7: ACCELERATION RANGE
      stringBuffer = myFile.readStringUntil('\n');
      intBuffer = stringBuffer.toInt();
      switch (intBuffer) {
        case 2:
          scaleXYZ = 256000;
          accel_range = 0b10000001;
          break;
        case 4:
          scaleXYZ = 128000;
          accel_range = 0b10000010;
          break;
        case 8:
          scaleXYZ =  64000;
          accel_range = 0b10000011;
          break;
        default:
          scaleXYZ = 256000;
          accel_range = 0b10000001;
          break;
      }

      //LINE 8: TRIGGER LEVEL
      stringBuffer = myFile.readStringUntil('\n');
      intBuffer = stringBuffer.toInt();
      if ((intBuffer >= 5) && (intBuffer <= 1000)) {
        triggerMG = intBuffer;
      } else {
        triggerMG = 50;
      }

      //LINE 9: RECORDING RATE
      stringBuffer = myFile.readStringUntil('\n');
      intBuffer = stringBuffer.toInt();
      if ((intBuffer >= 1) && (intBuffer <= 200)) {
        recordingRate = intBuffer;
      } else {
        recordingRate = 200;
      }

      //LINE 10: PRE-BUFFER DURATION (SEC)
      stringBuffer = myFile.readStringUntil('\n');
      intBuffer = stringBuffer.toInt();
      if ((intBuffer >= 0) && (intBuffer <= 60)) {
        preBuffer = intBuffer;
      } else {
        preBuffer = 15;
      }

      //LINE 11: POST-BUFFER DURATION (SEC)
      stringBuffer = myFile.readStringUntil('\n');
      intBuffer = stringBuffer.toInt();
      if ((intBuffer >= 0) && (intBuffer <= 60)) {
        postBuffer = intBuffer;
      } else {
        postBuffer = 30;
      }

      //LINE 12: RTC TRIMMER
      stringBuffer = myFile.readStringUntil('\n');
      intBuffer = stringBuffer.toInt();
      if ((intBuffer >= 0) && (intBuffer <= 255)) {
        rtcTrimmer = intBuffer;
      } else {
        rtcTrimmer = 0;
      }

      //LINE 13: ADXL FILTER
      stringBuffer = myFile.readStringUntil('\n');
      intBuffer = stringBuffer.toInt();
      if ( ((intBuffer >=   0) && (intBuffer <=  10)) ||
           ((intBuffer >=  16) && (intBuffer <=  26)) ||
           ((intBuffer >=  32) && (intBuffer <=  42)) ||
           ((intBuffer >=  48) && (intBuffer <=  58)) ||
           ((intBuffer >=  64) && (intBuffer <=  74)) ||
           ((intBuffer >=  80) && (intBuffer <=  90)) ||
           ((intBuffer >=  96) && (intBuffer <= 106)) ) {
        ADXL_Filter = intBuffer;
      } else {
        ADXL_Filter = 0;
      }

    }
    myFile.close();

  } else {
    strncpy(   latitude, "Not Defined", points);    //  LINE  1
    strncpy(  longitude, "Not Defined", points);    //  LINE  2
    strncpy(  elevation, "Not Defined", points);    //  LINE  3
    strncpy(orientation, "Not Defined", points);    //  LINE  4
    strncpy( location_1, "Not Defined", points);    //  LINE  5
    strncpy( location_2, "Not Defined", points);    //  LINE  6
    scaleXYZ      =     256000;                     //  LINE  7
    accel_range   = 0b10000001;                     //  LINE  7 (cont'd)
    triggerMG     =         50;                     //  LINE  8
    recordingRate =        200;                     //  LINE  9
    preBuffer     =         15;                     //  LINE 10
    postBuffer    =         30;                     //  LINE 11
    rtcTrimmer    =          0;                     //  LINE 12
    ADXL_Filter   =          0;                     //  LINE 13
  }

  readDelay         = 1000.0 / (float)recordingRate;
  preBufferPoints   = recordingRate    * preBuffer;
  postBufferPoints  = recordingRate    * postBuffer;
  triggerLSB        = int32_t((float)triggerMG / 1000 * scaleXYZ + 0.5);
  lastPreBufferAddr = preBufferPoints  * (bytesPerLine + 1) - 1;

}
//====================================================================================//


//================================ INTERRUPT FUNCTION ================================//
void intFunction() {

  ZeroFlag = 1;
  Serial.println(F("'Zero' pushed."));

}
//====================================================================================//


//============================ INITIALIZE SERIAL MONITOR =============================//
void initialize_Serial() {

  Serial.begin(9600);
  while (!Serial) { }

}
//====================================================================================//


//============================= INITIALIZE SPI PROTOCOL ==============================//
void initialize_SPI() {

  SPI.begin();
  pinMode(CS_ADXL , OUTPUT);  digitalWrite(CS_ADXL , HIGH);
  for (uint8_t i = 0; i < N_SRAM; i++) {
    pinMode(CS_SRAM[i], OUTPUT);  digitalWrite(CS_SRAM[i], HIGH);
  }

}
//====================================================================================//


//============================== INITIALIZE RTC VIA I2C ==============================//
void setup_RTC() {

  uint8_t temp = 0;

  //sets up I2C at 100kHz (datasheet says it works to 400kHz)
  Wire.setClock(100000);
  Wire.begin();

  Wire.beginTransmission(RTCADDR);
  Wire.write(CONTROL);
  Wire.write(0b00000000);                   // Turn OFF the coarse digital trim function (trims every minute)
//  Wire.write(0b00000100);                   // Turn ON the coarse digital trim function (trims every second)
  Wire.endTransmission();

  Wire.beginTransmission(RTCADDR);
  Wire.write(OSCTRIM);
  Wire.write(rtcTrimmer);                   // Set amount to trim, either every second or every minute (see above)
  Wire.endTransmission();

  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCWKDAY);
  Wire.endTransmission();
  Wire.requestFrom(RTCADDR, 1);
  temp = Wire.read();
  temp |= 0x08;                             // enable Battery backup
  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCWKDAY);
  Wire.write(temp);
  Wire.endTransmission();

  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCSEC);
  Wire.endTransmission();
  Wire.requestFrom(RTCADDR, 1);
  temp = Wire.read();                       // read out seconds
  temp |= 0x80;                             // flip the start bit to ON
  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCSEC);
  Wire.write(temp);                         // write it back in... now the RTC is running (if it wasn't already)
  Wire.endTransmission();

}
//====================================================================================//


//================================ READ IDS OF ADXL355 ===============================//
//void readID_ADXL() {
//
//  // Read the ID values from the accelerometer
//  uint8_t output = 0;
//  Serial.println();
//
//  output = read_ADXL_reg(((DEVID_AD  << 1) & (0b01111110)) | ADXL_READ);
//  Serial.print("AD ID:      "); Serial.print(output, HEX); Serial.println();
//
//  output = read_ADXL_reg(((DEVID_MST << 1) & (0b01111110)) | ADXL_READ);
//  Serial.print("AD MEMS ID: "); Serial.print(output, HEX); Serial.println();
//
//  output = read_ADXL_reg(((PARTID    << 1) & (0b01111110)) | ADXL_READ);
//  Serial.print("Device ID:  "); Serial.print(output, OCT); Serial.println();
//
//  output = read_ADXL_reg(((REVID     << 1) & (0b01111110)) | ADXL_READ);
//  Serial.print("Revision:   "); Serial.print(output, DEC); Serial.println();
//
//  Serial.println();
//
//}
//====================================================================================//


//======================= READ ACCELEROMETER ADXL355 REGISTER ========================//
uint8_t read_ADXL_reg(uint8_t address) {

  uint8_t output;

  SPI.beginTransaction(ADXLSettings);
  digitalWrite(CS_ADXL, LOW);
  SPI.transfer(address);
  output = SPI.transfer(0);
  digitalWrite(CS_ADXL, HIGH);
  SPI.endTransaction();

  return output;

}
//====================================================================================//


//====================== INITIAL SETUP OF ADXL355 ACCELEROMETER ======================//
void setup_ADXL() {

  // IMPORTANT: Need to set up (almost) all registers before waking up
  write_ADXL_reg(((FILTER    << 1) & (0b01111110)) | ADXL_WRITE, ADXL_Filter);        // Setup internal low- and high-pass filters
  write_ADXL_reg(((RANGE     << 1) & (0b01111110)) | ADXL_WRITE, accel_range);        // Setup acceleration range
  write_ADXL_reg(((POWER_CTL << 1) & (0b01111110)) | ADXL_WRITE,  0b00000000);        // Wake-up Call

}
//====================================================================================//


//======================= WRITE ACCELEROMETER ADXL355 REGISTER =======================//
void write_ADXL_reg(uint8_t address, uint8_t value) {

  SPI.beginTransaction(ADXLSettings);
  digitalWrite(CS_ADXL, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS_ADXL, HIGH);
  SPI.endTransaction();

}
//====================================================================================//


//=========================== INITIAL SETUP OF SRAM CHIPS ============================//
void setup_SRAM() {

  for (uint8_t i = 0; i < N_SRAM; i++) {
    write_SRAM_reg(CS_SRAM[i], SRAM_WRSR, SRAM_BYTE);
  }

}
//====================================================================================//


//============================ CLEAR ALL DATA FROM SRAM ==============================//
void clear_SRAM() {

  for (uint32_t i = 0; i < (N_SRAM * (uint32_t)SRAM_LENGTH); i++)
    write_SRAM(i, 0x00);

}
//====================================================================================//


//========================= WRITE TO REGISTERS OF SRAM CHIP ==========================//
void write_SRAM_reg(uint8_t CS, uint8_t address, uint8_t instruction) {

  SPI.beginTransaction(SRAMSettings);
  digitalWrite(CS, LOW);
  SPI.transfer(address);
  SPI.transfer(instruction);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();

}
//====================================================================================//


//======================= ZERO ADXL355 ACCELEROMETER CHANNELS ========================//
void zero_ADXL() {

  int32_t totals[N_Axes],
          accelsNow[N_Axes];

  for (uint8_t i = 0; i < N_Axes; i++)
    totals[i] = 0;

  for (uint16_t i = 0; i < NZeroPoints; i++) {
    read_ADXL(accelsNow);
    for (uint8_t j = 0; j < N_Axes; j++)
      totals[j] += accelsNow[j];
    delay(25);
  }

  for (uint8_t i = 0; i < N_Axes; i++)
    accel_zero[i] = int32_t(totals[i] / NZeroPoints + 0.5);

  if (!waveform) {
    Serial.println();
    Serial.print(F("Zeros: "));
    Serial.print(accel_zero[0]); Serial.print('\t');
    Serial.print(accel_zero[1]); Serial.print('\t');
    Serial.print(accel_zero[2]); Serial.println();
    Serial.println();
  }

}
//====================================================================================//


//========================= READ ACCELEROMETER ADXL355 DATA ==========================//
void read_ADXL(int32_t accel_xyz[]) {

  uint8_t accel_raw[N_Axes * accelLen];

  SPI.beginTransaction(ADXLSettings);
  digitalWrite(CS_ADXL, LOW);
  SPI.transfer( ((XDATA3 << 1) & (0b01111110)) | ADXL_READ );
  for (uint8_t i = 0; i < (N_Axes * accelLen); i++)
    accel_raw[i] = SPI.transfer(0);
  digitalWrite(CS_ADXL, HIGH);
  SPI.endTransaction();

  for (uint8_t i = 0; i < N_Axes; i++)
    accel_xyz[i] = (int32_t)((((uint32_t)accel_raw[accelLen * i    ] << 24) & 0xFF000000) |
                             (((uint32_t)accel_raw[accelLen * i + 1] << 16) & 0x00FF0000) |
                             (((uint32_t)accel_raw[accelLen * i + 2] <<  8) & 0x0000F000)) >> 12;

}
//====================================================================================//


//================================== SRAM -- WRITE ===================================//
void write_SRAM(uint32_t address, uint8_t value) {

  bool    keepLooking = 1;
  uint8_t CS,
          i = 0;

  while (keepLooking) {
    if (i < N_SRAM) {
      if (address < ((i + 1) * (uint32_t)SRAM_LENGTH)) {
        CS = CS_SRAM[i];
        address = address - (i * (uint32_t)SRAM_LENGTH);
        keepLooking = 0;
      } else i++;
    } else {
      Serial.print(F("Cannot write ")); Serial.print(value); Serial.print(F(" to SRAM ")); Serial.println(address);
      keepLooking = 0;
      CS = CS_SRAM[N_SRAM - 1];
      address = N_SRAM * (uint32_t)SRAM_LENGTH - 1;
      value = 0;
    }
  }

  SPI.beginTransaction(SRAMSettings);
  digitalWrite(CS, LOW);
  SPI.transfer(SRAM_WRITE);               //  Enter into Write mode
  SPI.transfer((uint8_t)(address >> 16)); //  Send first 8 bits of address
  SPI.transfer((uint8_t)(address >>  8)); //  Send second 8 bits of address
  SPI.transfer((uint8_t)(address      )); //  Send last 8 bits of address
  SPI.transfer(value);                    //  Write data to SRAM chip
  digitalWrite(CS, HIGH);
  SPI.endTransaction();

}
//====================================================================================//


//====================== WRITE ONE SET OF ADXL355 DATA TO SRAM =======================//
void write_dataset_to_SRAM(int32_t accel_xyz[], unsigned long currentMillis, uint32_t lineNo) {

  if (recordTimestamp) {
    for (uint8_t i = 0; i < timeLen; i++) {
      write_SRAM((bytesPerLine * lineNo) + i, (uint8_t)(currentMillis >> (8 * (timeLen - i - 1))));
    }
  }

  for (uint8_t i = 0; i < N_Axes; i++) {
    for (uint8_t j = 0; j < accelLen; j++) {
      write_SRAM((bytesPerLine * lineNo) + (accelLen * i) + (timeLen + j),
                 (uint8_t)((uint32_t)accel_xyz[i] >> (8 * (accelLen - j - 1))));
    }
  }

}
//====================================================================================//


//========================== CHECK IF TRIGGERS ARE EXCEEDED ==========================//
bool trigger_exceeded(int32_t accel_xyz[]) {

  if ((abs(accel_xyz[0] - accel_zero[0]) >= triggerLSB) ||
      (abs(accel_xyz[1] - accel_zero[1]) >= triggerLSB) ||
      (abs(accel_xyz[2] - accel_zero[2]) >= triggerLSB))
    return(1);
  else return(0);

}
//====================================================================================//


//=================================== SRAM -- READ ===================================//
uint8_t read_SRAM(uint32_t address) {

  bool    keepLooking = 1,
          errorTracker = 0;
  uint8_t CS,
          i = 0,
          output;

  while (keepLooking) {
    if (i < N_SRAM) {
      if (address < ((i + 1) * (uint32_t)SRAM_LENGTH)) {
        CS = CS_SRAM[i];
        address = address - (i * (uint32_t)SRAM_LENGTH);
        keepLooking = 0;
      } else i++;
    } else {
      Serial.print(F("Cannot read from SRAM ")); Serial.println(address);
      keepLooking = 0;
      CS = CS_SRAM[N_SRAM - 1];
      address = N_SRAM * (uint32_t)SRAM_LENGTH - 1;
      errorTracker = 1;
    }
  }

  if (errorTracker)
    output = 0;
  else {
    SPI.beginTransaction(SRAMSettings);
    digitalWrite(CS, LOW);
    SPI.transfer(SRAM_READ);                //  Enter into Read mode
    SPI.transfer((uint8_t)(address >> 16)); //  Send first 8 bits of address
    SPI.transfer((uint8_t)(address >>  8)); //  Send second 8 bits of address
    SPI.transfer((uint8_t)(address      )); //  Send last 8 bits of address
    output = SPI.transfer(0);               //  Read data from SRAM chip
    digitalWrite(CS, HIGH);
    SPI.endTransaction();
  }

  return output;

}
//====================================================================================//


//============================== INCREMENT FILE NUMBER ===============================//
void incrementFile() {

  fileNo++;
  EEPROM.write(0, (uint8_t)(fileNo >> 8));
  EEPROM.write(1, (uint8_t)(fileNo     ));
  sprintf(currentOutputFile, "OUT%05d.OUT", fileNo);
  Serial.println();
  Serial.println(currentOutputFile);

}
//====================================================================================//


//============================== GET TIMESTAMP FROM RTC ==============================//
void get_timestamp(char timeStamp[]) {

  bool  rtc12hrMode,
        rtcPM;

  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCSEC);
  Wire.endTransmission();
  Wire.requestFrom(RTCADDR, 7);

  //now read each byte in and clear off bits we don't need, hence the AND operations
  rtcSeconds = Wire.read() & 0x7F;
  rtcMinutes = Wire.read() & 0x7F;
  rtcHours   = Wire.read() & 0x7F;
  rtcWeekDay = Wire.read() & 0x3F;
  rtcDay     = Wire.read() & 0x3F;
  rtcMonth   = Wire.read() & 0x3F;
  rtcYear    = Wire.read();

  //now format the data, combine lower and upper parts of byte to give decimal number
  rtcSeconds = (rtcSeconds >> 4) * 10 + (rtcSeconds & 0x0F);
  rtcMinutes = (rtcMinutes >> 4) * 10 + (rtcMinutes & 0x0F);

  if ((rtcHours >> 6) == 1)//check for 12hr mode
    rtc12hrMode = true;
  else rtc12hrMode = false;

  // 12hr check and formatting of Hours
  if (rtc12hrMode) { //12 hr mode so get PM/AM
    if ((rtcHours >> 5) & 0x01 == 1)
      rtcPM = true;
    else rtcPM = false;
    rtcHours = ((rtcHours >> 4) & 0x01) * 10 + (rtcHours & 0x0F);       //only up to 12
  }
  else { //24hr mode
    rtcPM = false;
    rtcHours = ((rtcHours >> 4) & 0x03) * 10 + (rtcHours & 0x0F);       //uses both Tens digits, '23'
  }

  //more formatting bytes into decimal numbers
  rtcDay = (rtcDay >> 4) * 10 + (rtcDay & 0x0F);
  rtcMonth = ((rtcMonth >> 4) & 0x01) * 10 + (rtcMonth & 0x0F);
  rtcYear = (rtcYear >> 4) * 10 + (rtcYear & 0x0F);

//  sprintf(timeStamp, "20%02d-%02d-%02d %02d:%02d:%02d", rtcYear, rtcMonth, rtcDay, rtcHours, rtcMinutes, rtcSeconds);
  sprintf(timeStamp, "20%02d-%02d-%02d %02d:%02d", rtcYear, rtcMonth, rtcDay, rtcHours, rtcMinutes);

}
//====================================================================================//


//============================== GET RECORDING DURATION ==============================//
float get_duration(uint32_t totalPoints) {

  //  Grab recording duration
  float duration;
  duration = (float)(totalPoints - 1) / recordingRate;
  return duration;

}
//====================================================================================//


//============================= GET PGA IN EA. DIRECTION =============================//
void get_PGAs(int32_t PGA[], uint16_t preBufferPoint, uint16_t EQPointCounter) {

  //  Grab PGAs in each direction
  int32_t accel_xyz[N_Axes];
  for (uint8_t i = 0; i < N_Axes; i++)
    PGA[i] = 0;


  if (bufferLooped) {

    for (uint32_t i = 0; i < (preBufferPoints + EQPointCounter); i++) {
      for (uint8_t j = 0; j < N_Axes; j++) {
        accel_xyz[j] = read_SRAM_accel((bytesPerLine * i) + (accelLen * j) + timeLen);
        if ((accel_xyz[j] - accel_zero[j]) > PGA[j])
          PGA[j] = accel_xyz[j] - accel_zero[j];
        else if (-(accel_xyz[j] - accel_zero[j]) > PGA[j])
          PGA[j] = -(accel_xyz[j] - accel_zero[j]);
      }
    }

  } else {

    for (uint32_t i = 0; i < preBufferPoint; i++) {
      for (uint8_t j = 0; j < N_Axes; j++) {
        accel_xyz[j] = read_SRAM_accel((bytesPerLine * i) + (accelLen * j) + timeLen);
        if ((accel_xyz[j] - accel_zero[j]) > PGA[j])
          PGA[j] = accel_xyz[j] - accel_zero[j];
        else if (-(accel_xyz[j] - accel_zero[j]) > PGA[j])
          PGA[j] = -(accel_xyz[j] - accel_zero[j]);
      }
    }

    for (uint32_t i = preBufferPoints; i < (preBufferPoints + EQPointCounter); i++) {
      for (uint8_t j = 0; j < N_Axes; j++) {
        accel_xyz[j] = read_SRAM_accel((bytesPerLine * i) + (accelLen * j) + timeLen);
        if ((accel_xyz[j] - accel_zero[j]) > PGA[j])
          PGA[j] = accel_xyz[j] - accel_zero[j];
        else if (-(accel_xyz[j] - accel_zero[j]) > PGA[j])
          PGA[j] = -(accel_xyz[j] - accel_zero[j]);
      }
    }

  }

  for (uint8_t i = 0; i < N_Axes; i++)
    PGA[i] = (int32_t)((float)PGA[i] / scaleXYZ * 1000 + 0.5);

}
//====================================================================================//


//============================= GET AMBIENT TEMPERATURE ==============================//
uint16_t get_temp() {

  uint16_t  tempLSB;
  float     tempC,
            tempF;

  SPI.beginTransaction(ADXLSettings);
  digitalWrite(CS_ADXL, LOW);
  SPI.transfer(((TEMP2 << 1) & (0b01111110)) | ADXL_READ);
  tempLSB = ((uint16_t)(SPI.transfer(0) & 0x0F) << 8) | SPI.transfer(0);
  digitalWrite(CS_ADXL, HIGH);
  SPI.endTransaction();

  tempC = ((float)tempLSB - INTERCEPT) / SLOPE;
  tempF = tempC * 9.0 / 5.0 + 32;

  return (uint16_t)(tempF + 0.5);

}
//====================================================================================//


//=========================== WRITE HEADER TO OUTPUT FILE ============================//
void writeHeader(uint16_t preBufferPoint, uint16_t EQPointCounter) {

  char     EEPROM_Writer[points];
  uint16_t totalPoints;

  if (bufferLooped)  totalPoints = preBufferPoints + EQPointCounter;
  else               totalPoints = preBufferPoint  + EQPointCounter;

  // Send timestamp to EEPROM
  for (uint8_t i = 0; i < points; i++) {
    EEPROM_Writer[i] = '\0';
  }
  get_timestamp(EEPROM_Writer);
  populate_EEPROM_header(EEPROM_Writer,  1);

  // Send accelerometer range to EEPROM
  sprintf(EEPROM_Writer, "+/- %dmg", 1048576 * 1000 / 2 / scaleXYZ);
  populate_EEPROM_header(EEPROM_Writer,  7);

  // Send device orientation to EEPROM
  sprintf(EEPROM_Writer, orientation);
  populate_EEPROM_header(EEPROM_Writer, 11);

  // Send trigger level to EEPROM
  sprintf(EEPROM_Writer, "%d", (uint16_t)(triggerMG));
  populate_EEPROM_header(EEPROM_Writer, 13);

  // Send recording rate to EEPROM
  sprintf(EEPROM_Writer, "%d", recordingRate);
  populate_EEPROM_header(EEPROM_Writer, 15);


  // Send high- and low-pass filter information to EEPROM
  uint8_t ODR_LPF    = ADXL_Filter & 0b00001111;
  uint8_t HPF_CORNER = (ADXL_Filter & 0b01110000) >> 4;
  float   ODR        = 4000.0,
          LPF        = 1000.0,
          HPF;
  char    LPF_char[10],
          HPF_char[12];

  for (uint8_t i = 0; i < 10; i++)  LPF_char[i] = '\0';
  for (uint8_t i = 0; i < 12; i++)  HPF_char[i] = '\0';

  for (uint8_t i = 0; i < ODR_LPF; i++) {
    ODR = ODR / 2.0;
    LPF = LPF / 2.0;
  }

  switch (HPF_CORNER) {
    case 0:
      HPF = 0.0;
      break;
    case 1:
      HPF = 0.247000 * ODR;
      break;
    case 2:
      HPF = 0.062084 * ODR;
      break;
    case 3:
      HPF = 0.015545 * ODR;
      break;
    case 4:
      HPF = 0.003862 * ODR;
      break;
    case 5:
      HPF = 0.000954 * ODR;
      break;
    case 6:
      HPF = 0.000238 * ODR;
      break;
  }

  if      ( HPF             <      0.0) sprintf(HPF_char, "ERROR,     ");
  else if ( HPF            ==      0.0) sprintf(HPF_char, "HPF off,   ");
  else if ((HPF + 0.00005)  <     10.0) sprintf(HPF_char, "%d.%04d Hz, ", (uint16_t)(HPF + 0.00005), (uint16_t)((HPF + 0.00005) * 10000) % 10000);
  else if ((HPF +  0.0005)  <    100.0) sprintf(HPF_char, "%d.%03d Hz, ", (uint16_t)(HPF +  0.0005), (uint16_t)((HPF +  0.0005) *  1000) %  1000);
  else if ((HPF +   0.005)  <   1000.0) sprintf(HPF_char, "%d.%02d Hz, ", (uint16_t)(HPF +   0.005), (uint16_t)((HPF +   0.005) *   100) %   100);
  else                                  sprintf(HPF_char, "ERROR");

  if      ( LPF             <      0.0) sprintf(LPF_char, "ERROR    ");
  else if ((LPF + 0.00005)  <     10.0) sprintf(LPF_char, "%d.%04d Hz", (uint16_t)(LPF + 0.00005), (uint16_t)((LPF + 0.00005) * 10000) % 10000);
  else if ((LPF +  0.0005)  <    100.0) sprintf(LPF_char, "%d.%03d Hz", (uint16_t)(LPF +  0.0005), (uint16_t)((LPF +  0.0005) *  1000) %  1000);
  else if ((LPF +   0.005)  <   1000.0) sprintf(LPF_char, "%d.%02d Hz", (uint16_t)(LPF +   0.005), (uint16_t)((LPF +   0.005) *   100) %   100);
  else if ((LPF +    0.05)  <  10000.0) sprintf(LPF_char, "%d.%01d Hz", (uint16_t)(LPF +    0.05), (uint16_t)((LPF +    0.05) *    10) %    10);
  else                                  sprintf(LPF_char, "ERROR    ");

  for (uint8_t i = 0; i < points; i++) {
    if       (i < 11)  EEPROM_Writer[i] = HPF_char[     i];
    else if  (i < 20)  EEPROM_Writer[i] = LPF_char[i - 11];
    else               EEPROM_Writer[i] =             '\0';
  }

  populate_EEPROM_header(EEPROM_Writer, 17);



  // Send instrument latitude to EEPROM
  sprintf(EEPROM_Writer, latitude);
  populate_EEPROM_header(EEPROM_Writer, 19);

  // Send instrument longitude to EEPROM
  sprintf(EEPROM_Writer, longitude);
  populate_EEPROM_header(EEPROM_Writer, 21);

  // Send instrument elevation to EEPROM
  sprintf(EEPROM_Writer, elevation);
  populate_EEPROM_header(EEPROM_Writer, 23);

  // Send line 1 of location description to EEPROM
  sprintf(EEPROM_Writer, location_1);
  populate_EEPROM_header(EEPROM_Writer, 25);

  // Send line 2 of location description to EEPROM
  sprintf(EEPROM_Writer, location_2);
  populate_EEPROM_header(EEPROM_Writer, 27);

  // Send recording duration EEPROM
  float duration =  get_duration(totalPoints);
  sprintf(EEPROM_Writer, "%d.%01d", (int)(duration + 0.05), (int)((duration + 0.05) * 10) % 10);
  populate_EEPROM_header(EEPROM_Writer, 31);

  // Send PGAs to EEPROM
  int32_t PGA[3];
  get_PGAs(PGA, preBufferPoint, EQPointCounter);
  sprintf(EEPROM_Writer, "%d", PGA[0]);
  populate_EEPROM_header(EEPROM_Writer, 33);
  sprintf(EEPROM_Writer, "%d", PGA[1]);
  populate_EEPROM_header(EEPROM_Writer, 35);
  sprintf(EEPROM_Writer, "%d", PGA[2]);
  populate_EEPROM_header(EEPROM_Writer, 37);

  // Send ambient temperature EEPROM
  sprintf(EEPROM_Writer, "%d", get_temp());
  populate_EEPROM_header(EEPROM_Writer, 39);

  // PRINT TO SERIAL
  for (uint8_t i = 0; i < lines; i++) {
    for (uint8_t j = 0; j < points; j++)
      Serial.print((char)EEPROM.read(points * i + j + 2));
    if ((i & 0x01) == 0)
      Serial.print('\t');
    else Serial.println();
  }

}
//====================================================================================//


//========================== POPULATE EEPROM HEADER VALUES ===========================//
void populate_EEPROM_header(char EEPROM_Writer[], uint8_t i) {

  for (uint8_t j = 0; j < points; j++)
    EEPROM.write((points * i) + j + 2, EEPROM_Writer[j]);
  for (uint8_t j = 0; j < points; j++)
    EEPROM_Writer[j] = '\0';

}
//====================================================================================//


//==================== COMPILE EVERYTHING INTO SINGLE OUTPUT FILE ====================//
void outputLogFile(uint16_t preBufferPoint, uint16_t EQPointCounter) {

  uint32_t  timeNow;
  int32_t   accel_xyz[N_Axes];
  File      SD_Final_Output;


//  uint16_t totalPoints;


  Serial.println();
  Serial.print(F("Writing to SD card... "));



  SD.begin(CS_SD);
  SPI.setClockDivider(SDSpeed);
  SdFile::dateTimeCallback(dateTime);
  SD_Final_Output = SD.open(currentOutputFile, FILE_WRITE);

  for (uint8_t i = 0; i < lines; i++) {
    for (uint8_t j = 0; j < points; j++)
      if ((char)EEPROM.read(points * i + j + 2) != '\0') {
        SD_Final_Output.print((char)EEPROM.read(points * i + j + 2));
      }

    if ((i & 0x01) == 0)
      SD_Final_Output.print('\t');
    else SD_Final_Output.println();
  }

  SD_Final_Output.println();
  if (recordTimestamp) {
    SD_Final_Output.print("Time (ms)"); SD_Final_Output.print('\t');
  }
  SD_Final_Output.print("x (mg)"); SD_Final_Output.print  ('\t');
  SD_Final_Output.print("y (mg)"); SD_Final_Output.print  ('\t');
  SD_Final_Output.print("z (mg)"); SD_Final_Output.println(    );

  //  Grab SRAM data and write to SD card. Grab in time order, which isn't straight through from start to finish.
  if (bufferLooped) {
    stream_SRAMtoSD(SD_Final_Output,  preBufferPoint, preBufferPoints);
    stream_SRAMtoSD(SD_Final_Output,               0,  preBufferPoint);
  } else {
    stream_SRAMtoSD(SD_Final_Output,               0,  preBufferPoint);
  }
  stream_SRAMtoSD(SD_Final_Output, preBufferPoints, preBufferPoints + EQPointCounter);

  SD_Final_Output.close();

  Serial.println(F("done."));
  Serial.println();

}
//====================================================================================//


//============================= APPLY TIMESTAMP TO FILE ==============================//
void dateTime(uint16_t* date, uint16_t* time) {

  *date = FAT_DATE(2000 + rtcYear,   rtcMonth,     rtcDay);
  *time = FAT_TIME(      rtcHours, rtcMinutes,          0);

}
//====================================================================================//


//============================== STREAM FROM SRAM TO SD ==============================//
void stream_SRAMtoSD(File SD_Final_Output, uint32_t first, uint32_t last) {

  uint32_t timeNow;
  int32_t  accel_xyz[N_Axes];

  for (uint32_t i = first; i < last; i++) {
    timeNow = read_SRAM_timestamp(bytesPerLine * i);
    for (uint8_t j = 0; j < N_Axes; j++)
      accel_xyz[j] = read_SRAM_accel((bytesPerLine * i) + (accelLen * j) + timeLen);
    print_to_SD(SD_Final_Output, timeNow, accel_xyz);
  }

}
//====================================================================================//


//============================== PRINT POINT TO SD CARD ==============================//
void print_to_SD(File SD_Final_Output, uint32_t timeNow, int32_t accel_xyz[]) {

  if (recordTimestamp) {
    SD_Final_Output.print(timeNow); SD_Final_Output.print('\t');
  }
  for (uint8_t i = 0; i < N_Axes; i++) {
    char temp[10];
    sprintf(temp, "%d.%02d",     (int)(((float)accel_xyz[i] - accel_zero[i]) / ((float)scaleXYZ / 1000)              ),
                             abs((int)(((float)accel_xyz[i] - accel_zero[i]) / ((float)scaleXYZ / 1000) * 100) % 100));
    SD_Final_Output.print(temp);
    if (i == (N_Axes - 1))
      SD_Final_Output.println();
    else
      SD_Final_Output.print('\t');
  }

}
//====================================================================================//


//======================== READ TIMESTANP IN FOUR SRAM BYTES =========================//
uint32_t read_SRAM_timestamp(uint32_t startAddress) {

  uint32_t output;

  output = (((uint32_t)read_SRAM(startAddress    ) << 24) & 0xFF000000) |
           (((uint32_t)read_SRAM(startAddress + 1) << 16) & 0x00FF0000) |
           (((uint32_t)read_SRAM(startAddress + 2) <<  8) & 0x0000FF00) |
           (((uint32_t)read_SRAM(startAddress + 3)      ) & 0x000000FF) ;

  return output;

}
//====================================================================================//


//==================== READ ACCELERATION VAL IN THREE SRAM BYTES =====================//
int32_t read_SRAM_accel(uint32_t startAddress) {

  int32_t output;

  output = (int32_t)((((uint32_t)read_SRAM(startAddress    ) <<  28) & 0xF0000000) |
                     (((uint32_t)read_SRAM(startAddress + 1) <<  20) & 0x0FF00000) |
                     (((uint32_t)read_SRAM(startAddress + 2) <<  12) & 0x000FF000)) >> 12;

  return output;

}
//====================================================================================//
