#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "MPU6050.h"
#include "hardware/i2c.h"
#include "imu.h"
#include "pico/stdlib.h"
#include <stdio.h>

MPU6050 mpu;

bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0
                   // = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;   // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 gy; // [x, y, z]            gyro sensor measurements
VectorInt16
    aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float
    ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

volatile bool mpuInterrupt =
    false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void IMU::I2C_init() {
  // This example will use I2C0 on the  SDA and SCL pins on Pico
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(SCL, GPIO_FUNC_I2C);
  gpio_set_function(SDA, GPIO_FUNC_I2C);
  gpio_pull_up(SCL);
  gpio_pull_up(SDA);
  // Make the I2C pins available to picotool
}

void IMU::imu_init() {
  I2C_init();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  /* --- if you have calibration data then set the sensor offsets here --- */
  // mpu.setXAccelOffset();
  // mpu.setYAccelOffset();
  // mpu.setZAccelOffset();
  // mpu.setXGyroOffset();
  // mpu.setYGyroOffset();
  // mpu.setZGyroOffset();

  /* --- alternatively you can try this (6 loops should be enough) --- */

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true; // set our DMP Ready flag so the main loop() function knows
                     // it's okay to use it
    packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for
                                             // later comparison
  } else { // ERROR!        1 = initial memory load failed         2 = DMP
           // configuration updates failed        (if it's going to break,
           // usually the code will be 1)
           //        printf("DMP Initialization failed (code %d)", devStatus);
    sleep_ms(2000);
  }
  yaw = 0.0;
  pitch = 0.0;
  roll = 0.0;
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
IMU::data IMU::get_data() {
  if (!dmpReady)
    ; // if programming failed, don't try to do anything
  mpuInterrupt = true;
  fifoCount = mpu.getFIFOCount(); // get current FIFO count
  if ((mpuIntStatus & 0x10) ||
      fifoCount == 1024) // check for overflow (this should never happen unless
                         // our code is too inefficient)
  {
    mpu.resetFIFO();              // reset so we can continue cleanly
                                  //            printf("FIFO overflow!");
  } else if (mpuIntStatus & 0x01) // otherwise, check for DMP data ready
                                  // interrupt (this should happen frequently)
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount(); // wait for correct available data length,
                                      // should be a VERY short wait
    mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
    fifoCount -= packetSize; // track FIFO count here in case there is > 1
                             // packet available display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }
  IMU::data data;
  data.ang_x = ypr[2] * 180 / PI;
  data.ang_y = ypr[1] * 180 / PI;
  data.ang_z = ypr[0] * 180 / PI;
  data.lin_accel_x = aaReal.x / 8192.0;
  data.lin_accel_y = aaReal.y / 8192.0;

  return data;
}
