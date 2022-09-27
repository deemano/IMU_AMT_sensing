#include <SimpleFOC.h>  // BLDC motor controller library
// Adafruit IMU library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (200)

// BLDC & driver instance. Formats: BLDCMotor(pole pair#). BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// Global variables
float targetPos;          // variable to drive BLDC in radian
float defaultPos = 1.5;   // default BLDC movement

// Instantiate Commander interface
Commander command = Commander(Serial);

// Set & get functions for target commands
void doCCW_target(char* cmd) { command.scalar(&targetPos, cmd); }
void doCCW_default(char* cmd) { targetPos = defaultPos, command.scalar(&targetPos, cmd); }
void doReturnHome(char* cmd) { targetPos = 0, command.scalar(&targetPos, cmd); }

// Adafruit IMU (id, address)
Adafruit_BNO055 bno_1 = Adafruit_BNO055(55, 0x29);
Adafruit_BNO055 bno_2 = Adafruit_BNO055(56, 0x28);

// Display some basic info about the sensor status:
/*************************************************/
// Sensor #1 >>>>>
void displaySensor_1(void)
{
  sensor_t sensor;
  bno_1.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor #1:       "); Serial.println(sensor.name);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno_1.getSystemStatus(&system_status, &self_test_results, &system_error);
  
  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  Serial.println("------------------------------------");
  delay(500);
}
/*****************************************************/
// Sensor #2 >>>>>
void displaySensor_2(void)
{
  sensor_t sensor;
  bno_2.getSensor(&sensor);
  Serial.print  ("Sensor #2:       "); Serial.println(sensor.name);
  //Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno_2.getSystemStatus(&system_status, &self_test_results, &system_error);
  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/*****************************************************/
// Display sensor calibration status
/**************************************************************************/
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
  if (!system){ Serial.print("! "); }
  
  /* Display the individual values */
//  Serial.print("Sys:");
//  Serial.print(system, DEC);
//  Serial.print(" G:");
//  Serial.print(gyro, DEC);
//  Serial.print(" A:");
//  Serial.print(accel, DEC);
//  Serial.print(" M:");
//  Serial.print(mag, DEC);
}

// Main configuration function
void setup() {
  // Motor setup
  driver.voltage_power_supply = 12; // driver config. power supply voltage
  /* Note: Limit the driver max dc voltage to protect low-resistance motors */
  driver.voltage_limit = 6;
  driver.init();  
  
  // Link the motor & the driver
  motor.linkDriver(&driver);

  // Trial & error setup: Start low, well under 1Amp
  motor.voltage_limit = 3;    // [V]
  // Set the transition velocity between target angles
  motor.velocity_limit = 0.6; // [rad/s] cca 50rpm
  
  // Open loop control configuration
  motor.controller = MotionControlType::angle_openloop;

  // Initiate motor functions
  motor.init();

  // Setting target commands
  command.add('d', doCCW_target, "CCW target angle");
  command.add('w', doCCW_default, "CCW 1.5 Radians");
  command.add('s', doReturnHome, "return to start position");

  // Set serial communication baud frequency & get feedback
  Serial.begin(115200);
  Serial.println("===================================================");
  Serial.println("\nProject name: Quark_Zero1 - Another jumping robot\nProject developed by: Denis Manolescu\nSupervisor: Prof. Dr. Emanuele Secco\nLiverpool Hope University. @ September 2022\n===============================\n");
  Serial.println("Quark_Zero1 robot is ready!\n");
  Serial.println("Instructions:\n1. Type 'w' to move the robot to its default position.\n2. Type 'd' followed by the number of radians, to move it where you want [ex: d0.8 or d2]\n3. Type 's' to go back Home to its initial position.");
  _delay(1000);

  // Adafruit setup
  Serial.println("\nOrientation Sensor Feedback"); Serial.println("");
  /* Initialise the sensor */
  if(!bno_1.begin() || !bno_2.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("1 or 2 x BNO055 NOT detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(3000);
  displaySensor_1();
  displaySensor_2();

  /* Optional: Display current status */
  bno_1.setExtCrystalUse(true);
  bno_2.setExtCrystalUse(true);
}

// Main loop
void loop() {
    /* Get a new sensor event */
  sensors_event_t event;
  bno_1.getEvent(&event);

  /* Display the floating point data */
  Serial.print("     "); //X: ");
  Serial.print(360 - event.orientation.x, 0);
  
  bno_2.getEvent(&event);

    /* Display the floating point data */
  Serial.print("     "); //X: ");
  Serial.print(360 - event.orientation.x, 0);

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  // The open loop angle movement function
  motor.move(targetPos);
  
  // User communication
  command.run();
}
