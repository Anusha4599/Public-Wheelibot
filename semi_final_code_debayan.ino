#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//#include <Wire.h>

Adafruit_MPU6050 mpu1;
Servo ESC;
int ctr=0;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define INTERRUPT_PIN 2  //mpu

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int encoder_pin = 5;
unsigned int rpm;
volatile int pulses = 0; 
unsigned long timeold = 0;
unsigned int pulsesperturn = 10;

float mpu_pos = 0;
float mpu_vel = 0;

Quaternion q;           // [w, x, y, z]         quaternion container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat gravity;    // [x, y, z]            gravity 
int16_t gyro[3];

//MPU6050 mpu;
MPU6050 mpu(0x69); // <-- use for AD0 high

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float return_vel()
{ 
  mpu.dmpGetGyro(gyro, fifoBuffer);
  //Serial.print(gyro[2]);
  return(gyro[2]);
}

/*
int return_vel() {
  sensors_event_t a, g, temp;
  mpu1.getEvent(&a, &g, &temp);
  return(g.gyro.x);
}*/

float return_roll(){

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 // Serial.println(ypr[2]);
  return(ypr[2]*180/M_PI);}

void DMP_initialize(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif
      while (!Serial);
      Serial.println(F("Initializing I2C devices..."));
      mpu.initialize();
      pinMode(INTERRUPT_PIN, INPUT);
      Serial.println(F("Testing device connections..."));
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      Serial.println(F("Initializing DMP..."));
      devStatus = mpu.dmpInitialize();
    
      mpu.setXGyroOffset(49);
      mpu.setYGyroOffset(-1);
      mpu.setZGyroOffset(-4);
      mpu.setZAccelOffset(1043); // 1688 factory default for my test chip

      if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready!"));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
     } else {
        
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
     }
  }
   
void counter(){
   pulses++;
}

int return_speed(){
      detachInterrupt(encoder_pin);
      rpm = (60*1000 / pulsesperturn )/ (millis() - timeold)* pulses;
      timeold = millis();
      pulses = 0;
      attachInterrupt(encoder_pin, counter, FALLING);
      return((int)rpm);
      }
      
void speed_up(){
  int sensorValue = analogRead(A2);
  int pwm = map(sensorValue, 0, 1023, 0, 180);
  ESC.write(pwm);  
}

void direction_control(int ctr){
    if(ctr==0)
    {
      digitalWrite(8,HIGH);
    }
    else
    {
      digitalWrite(8,LOW);
    }
  }

//void mpu_adafruit_init() {
//  // Try to initialize!
//  if (!mpu1.begin(0x69)) {
//    Serial.println("Failed to find MPU6050 chip");
//    while (1) {
//      delay(10);
//    }
//  }
//  Serial.println("MPU6050 Found!");
//  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
//  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
//}
void setup() {
  Serial.begin(9600);
  DMP_initialize();
  //mpu_adafruit_init();
  ESC.attach(9,0,2000);   //speed control for BLDC motor
  pinMode(3,INPUT_PULLUP);   //cotrol output for switching directions 
  pinMode(8,OUTPUT);
  pinMode(encoder_pin, INPUT);
  attachInterrupt(encoder_pin, counter, FALLING);
  timeold = millis();
}

void loop() {
    speed_up();
    ctr = digitalRead(3);
    direction_control(ctr);
    if (!dmpReady) return;
    //Serial.println(mpu.dmpGetCurrentFIFOPacket(fifoBuffer));
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu_pos = return_roll();
    mpu_vel = return_vel();
    //Serial.println("Entered this cool if");
    }
    int spd = return_speed();
    Serial.print("POS = ");
    Serial.print(mpu_pos);
    Serial.print("  ");
    Serial.print("VEL = ");
    Serial.print(mpu_vel);
    Serial.print("  ");
    Serial.print("WHEEL SPEED = ");
    Serial.print(spd);
    Serial.println();
}
