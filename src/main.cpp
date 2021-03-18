#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;
Servo motor;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13
#define INTERRUPT_PIN 2

// VL53L0X

bool blinkState = false;
bool dmpReady = false;  

uint8_t mpuIntStatus;  
uint8_t devStatus;
uint16_t packetSize;  
uint16_t fifoCount;     
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;       // w, x, y, z] quaternion container
VectorFloat gravity;// [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//ROUND
//int angle;
//int angle_old = 180;

//NO ROUND
float angle;
float angle_old = 180.00;
float error = 0;

String COMMAND_STRING;
int COMMAND_INTEGER;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

float getAngle(){
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    return ((ypr[0] * 180/M_PI) + 180);
}

void setup() {

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    

    Serial.begin(115200);
    motor.attach(3);
    motor.write(98);                                   // SERVO NEUTRAL

    sensor.setTimeout(500);
    
    if (!sensor.init())
    {
        //Serial.println("Failed to detect and initialize sensor!");
        while (1) {}
    }
    
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //digitalPinToInterrupt(INTERRUPT_PIN);
        //Serial.print();
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    motor.write(105);
    delay(1000);
}

void loop() {
    while (dmpReady && !sensor.timeoutOccurred())
    {
        angle = getAngle();
        if (round(angle) != angle_old){
            error = error - 0.0045;
            Serial.print(round (angle + error));
            Serial.print(",");
            Serial.print(sensor.readRangeSingleMillimeters());
            Serial.println(",");
            angle_old = round(angle);
        }

        if(Serial.available()>0){
            COMMAND_INTEGER = (Serial.readString()).toInt();
            //motor.write(COMMAND_INTEGER);
            Serial.println(COMMAND_INTEGER);
            while (angle != COMMAND_INTEGER){
                mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                angle = (round(ypr[0] * 180/M_PI)) + 180;
                motor.write(105);
                Serial.println((ypr[0] * 180/M_PI) + 180);
            }
            motor.write(98);
        }
    }
    Serial.println(" TIMEOUT");
    return;
        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);
    
}