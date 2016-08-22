// Quadcopter sketch based on I2Cdevlib https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-08-03 - enable i2cbypass and include hmc5883l magnetometer compass
// MIT licence Copyright (c) 2016 Ben Gaunt

#define GRAVITY 9.80665 //m s^-2

#include <hmc5883l.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

hmc5883l compass; // create global variable for hmc5883l magnetometer instance

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin D2.
 * ========================================================================= */
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat vvReal;     // [x, y, z]            linear velocities
VectorFloat ssReal;     // [x, y, z]            linear displacements

// time interval for integration
unsigned long cur_time, prev_time;

float grav_scale = GRAVITY / 16384.0; //Mutiply by accelerometer readings to get SI unit versions

#define SMOOTH_F 0.01   // smoothing factor for compass
float compass_offset;   // offset to correct compass

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200); // TODO may need to reduce for communication with Raspberry Pi
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices...")); // TODO reduce strings of text to key character strings for communication with RPi
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    mpu.setI2CBypassEnabled(true);
    compass = hmc5883l();
    compass.SetScale(1.3); // Set the scale of the compass.
    compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    compass_offset = 0.0;
    prev_time = micros();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // ========================================================
        // ===      READ AND WRITE SERIAL FROM AND TO RPi
        // send: location and speed 6 axis
        // receive: target location, z rotation

        // ========================================================
        // ===      READ SoftwareSerial GPS using TinyGPS (reading from D10 pin)
        // smoothing (median filter?), update of dead reckoning position and velocity

        // ========================================================
        // ===      READ PRESSURE FROM bmp085
        // smoothing update of altitude as per GPS

        // ========================================================
        // ===      DO COURSE CALCULATIONS AND PID
        // calculate required roll and pitch for given yaw and magnitude of thrust vector
        // PID for overall thrust and three differential factors

        // ========================================================
        // ===      CHANGE SERVO PWM OUTPUTS TO ESCs
        // write pwm values to appropriate arduino pins D3, D5, D6, D11 available

        // if FIFO overflows during all this then test in between sections
        // to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display YawPitchRoll angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        MagnetometerScaled mags = compass.ReadScaledAxis();
        float sr = sin(ypr[2]); float cr = cos(ypr[2]); // sin and cos of roll
        float sp = sin(ypr[1]); float cp = cos(ypr[1]); // sin and cos of pitch
        float x_mag =  -mags.XAxis * cp + mags.ZAxis * sp;
        float y_mag = -mags.XAxis * sr * sp + mags.YAxis * cr - mags.ZAxis * sr * cp;
        float compass_diff = atan2(y_mag, -x_mag) - ypr[0];
        if (abs(compass_offset - compass_diff) > 0.25) { // i.e. major change such as from pi to -pi
          compass_offset = compass_diff;
        }
        else {
          compass_offset = compass_offset * (1.0 - SMOOTH_F) + compass_diff * SMOOTH_F; 
        }
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI); // yaw in degrees without correction
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI); // pitch in degrees
        Serial.print("\t");
        Serial.print(ypr[2] * 180/M_PI); // roll in degrees
        Serial.print("\t");
        Serial.println((ypr[0] + compass_offset) * 180/M_PI); // yaw in degrees with correction

        // read linear acceleration and integrate for velocity and position
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // just subtracts the components of gravity
        float sy = -sin(ypr[0]  + compass_offset); float cy = cos(ypr[0] + compass_offset); // sin and cos of true yaw
        //          ^If stuff's not working, change -sy back to sy
        //put acceleration in SI units
        aaReal.x *= grav_scale; aaReal.y *= grav_scale; aaReal.z *= grav_scale; 
        //Store old acceleration and velocity values
        float ax_old = aaWorld.x; float ay_old = aaWorld.y; float az_old = aaWorld.z;
        float vx_old = vvReal.x; float vy_old = vvReal.y; float vz_old = vvReal.z;
        //Apply an inverse rotation matrix to turn the accelerometer readings into the actual observed accelerations:
        aaWorld.x = (aaReal.x * cp * cy) + (aaReal.y * cp * sy) - (aaReal.z * sp);
        aaWorld.y = (aaReal.x * ((sr * sp * cy) - (sy * cr))) + (aaReal.y * ((sr * sp * sy) + (cr * cy))) + (aaReal.z * sr * cp);
        aaWorld.z = (aaReal.x * ((cr * sp * cy) + (sy * sr))) + (aaReal.y * ((cr * sp * sy) - (sr * cy))) + (aaReal.z * cr * cp);

        cur_time = micros();
        if(cur_time < 10000000) {
          vvReal.x = vvReal.y = vvReal.z = 0.0;
          ssReal.x = ssReal.y = ssReal.y = 0.0;
        }
        float half_delta_t = (cur_time - prev_time) / 2000000.0; //since we use (delta_t / 2) multiple times in integration.
        vvReal.x += (aaWorld.x + ax_old) * half_delta_t;
        vvReal.y += (aaWorld.y + ay_old) * half_delta_t;
        vvReal.z += (aaWorld.x + az_old) * half_delta_t;
        ssReal.x += (vvReal.x + vx_old) * half_delta_t;
        ssReal.y += (vvReal.y + vy_old) * half_delta_t;
        ssReal.z += (vvReal.z + vz_old) * half_delta_t;
        prev_time = cur_time;

        Serial.print("accel\t");
        Serial.print(aaWorld.x); Serial.print("\t");
        Serial.print(aaWorld.y); Serial.print("\t");
        Serial.println(aaWorld.z); 

        Serial.print("vel\t");
        Serial.print(vvReal.x); Serial.print("\t");
        Serial.print(vvReal.y); Serial.print("\t");
        Serial.println(vvReal.z); 

        Serial.print("xyz\t");
        Serial.print(ssReal.x); Serial.print("\t");
        Serial.print(ssReal.y); Serial.print("\t");
        Serial.println(ssReal.z); 

        

        
        
        
        

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
