/* Encoder library */
#include <SPI.h>
/* IMU libraries */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Serial baudrates for UART */
#define BAUDRATE        115200

/* AMT SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* Define special commands */
#define NEWLINE         0x0A
#define TAB             0x09

/* Define macros for compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* AMT to Arduino Mega pins */
#define ENC_0            2
#define ENC_1           46 // second encoder
#define SPI_MOSI        51
#define SPI_MISO        50
#define SPI_SCLK        52

/* Adafruit BNO055 */
#define BNO055_SAMPLERATE_DELAY_MS (100) // samples delay

/* Define sensors (id, address) */
Adafruit_BNO055 bno_1 = Adafruit_BNO055(55, 0x29);
Adafruit_BNO055 bno_2 = Adafruit_BNO055(56, 0x28);

/**************************************************************************/
/* Display sensor calibration status function */
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno_1.getCalibration(&system, &gyro, &accel, &mag);
  bno_2.getCalibration(&system, &gyro, &accel, &mag);
  
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system) { Serial.print("! "); }
}
/**************************************************************************/

void setup() 
{
  // IMU Set SPI IO mode
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC_0, OUTPUT);
  pinMode(ENC_1, OUTPUT);
  
  // Initialize serial connection for debugging
  Serial.begin(BAUDRATE);

  // Start-up reset encoders
  digitalWrite(ENC_0, HIGH); // second encoder
  digitalWrite(ENC_1, HIGH);

  //set the clockrate. Uno / Mega clock rate is 16Mhz, divider of 32 gives 500 kHz.
  //500 kHz is a good speed for our test environment
  //SPI.setClockDivider(SPI_CLOCK_DIV2);   // 8 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV4);   // 4 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV8);   // 2 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV64);  // 250 kHz
  // SPI.setClockDivider(SPI_CLOCK_DIV128); // 125 kHz
  
  // Start IMU SPI bus
  SPI.begin();

   /* Initialise the sensor */
  if(!bno_1.begin() || !bno_2.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("1 or 2 x BNO055 NOT detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);

  /* Display basic information about sensor */
  displaySensor_1();
  displaySensor_2();

  /* Display current status */
  bno_1.setExtCrystalUse(true);
  bno_2.setExtCrystalUse(true);
}

void loop() 
{
  // Create a 16 bit variable to hold the encoders position
  uint16_t encoderPosition;
  // Count tries in case of error
  uint8_t attempts;

  // Reset sensors to zero 0 
  setZeroSPI(ENC_0);
  setZeroSPI(ENC_1);

  // IMU loop
  while(1)
  {
    // Reset error count   
    attempts = 0;

    // This function gets the encoder position and returns it as a uint16_t
    // Send the function either res12 or res14
    encoderPosition = getPositionSPI(ENC_1, RES14); 

    // If the position returned was 0xFFFF we know that there was an error calculating the checksum
    // Make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
    while (encoderPosition == 0xFFFF && ++attempts < 3)
    {
      encoderPosition = getPositionSPI(ENC_1, RES14); // try again
    }

    if (encoderPosition == 0xFFFF) // position is bad, let the user know how many times we tried
    {
      Serial.print("Encoder 1 error. Attempts: ");
      Serial.print(attempts, DEC); // print out the number in decimal format. attempts - 1 is used since we post incremented the loop
      Serial.write(NEWLINE);
    }
    else // position was good, print to serial stream
    {
      
      Serial.print("Encoder angle: ");
      // mapping the data capture on the X-axis
      int temp = encoderPosition;      
      float mappedValue = map(temp, 0, 16384, 0, 360);
      Serial.println(mappedValue);
      Serial.write(NEWLINE);
    }

    /* Create sensor data instance & attribute it to new sensor event */
    sensors_event_t event;
    bno_1.getEvent(&event);

    /* Display the floating point data */
    Serial.print("     ");
    Serial.print(event.orientation.x, 0);

    // Repeate for the second sensor
    bno_2.getEvent(&event);

    /* Display data points */
    Serial.print("     ");
    Serial.print(360 - event.orientation.x, 0);
  
    /* New line for the next sample */
    Serial.println("");
    
    /* Wait time between data points */
    delay(BNO055_SAMPLERATE_DELAY_MS);

    // delay(500);
  }
}

uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       // 16-bit response from encoder
  bool binaryArray[16];           // after receiving the position we will populate this array and use it for calculating the checksum

  // get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  // this is the time required between bytes as specified in the datasheet.
  // We will implement that time delay here, however the arduino is not the fastest device so the delay
  // is likely inherantly there already
  delayMicroseconds(3);

  // OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  // run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  // using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      // we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; // bad position
  }

  // If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  // holder for the received over SPI
  uint8_t data;

  // set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  // There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  // We will implement that time delay here, however the arduino is not the fastest device so the delay
  // is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); // There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); // if releaseLine is high set it high else it stays low
  
  return data;
}

/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere 
 * This function takes the pin number of the desired device as an input
 */
void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_RESET, encoder, true);
  
  delay(250); //250 second delay to allow the encoder to start back up
}


/* NOTE:
This code has been modified to serve the project's purpuse. 
Credit to Adafruit and CUI Devices libraries */
