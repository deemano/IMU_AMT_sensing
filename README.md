# IMU_AMT_sensing
BNO055 + AMT223 fusion sensing

This repository includes two Arduino .ino scripts for a robotic system combining motor control and sensor integration:

IMU and Encoder Fusion: Implements Adafruit IMU (BNO055) and AMT22 encoder for precise orientation and motion tracking. Features include SPI and I2C communication, sensor data synchronization, and encoder commands for high-resolution feedback.

Jumping Robot Prototype: Integrates a BLDC motor controller using the SimpleFOC library with an Adafruit IMU for stability and motion control. Configures motor parameters, handles sensor data, and ensures seamless operation through synchronized sampling and motor commands.
