#include <Servo.h>
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
//#include <Wire.h>

//Adafruit_MPU6050 mpu1;
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
unsigned long timeold_1 =0;
unsigned int pulsesperturn = 10;

#define K1 -0.5
#define K2 -0.2
#define K3 -0

float mpu_pos = 0;
float mpu_vel = 0;
float control = 0 ;

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

void myHandler(float mpu_pos,float mpu_vel,int wheel_spd){
  control = K1*mpu_pos + K2*mpu_vel + K3*wheel_spd;
  //constrain(control,-12,12);
  if (control>12)
  {
    control=12;
  }
  else if (control<-12)
  {
    control=-12;
  }
  
  Serial.println(control);
  int control_out = map(control, -12, 12, -180, 180);
  Serial.println(control_out);
  if(control_out>0) 
  {digitalWrite(8, HIGH);}
  else
  {digitalWrite(8,LOW);}
  ESC.write(abs(control_out));
}


float return_vel()
{ 
  mpu.dmpGetGyro(gyro, fifoBuffer);
  //Serial.print(gyro[2]);
  return(gyro[2]);
}

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

void main_loop(){
  if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu_pos = return_roll();
    mpu_vel = return_vel();
    }   
    int spd = return_speed();
    if(mpu_pos>3 || mpu_pos<-3){
    myHandler(mpu_pos,mpu_vel,spd);}
    
//    Serial.print("POS = ");
//    Serial.print(mpu_pos);
//    Serial.print("  ");
//    Serial.print("VEL = ");
//    Serial.print(mpu_vel);
//    Serial.print("  ");
//    Serial.print("WHEEL SPEED = ");
//    Serial.print(spd);
//    Serial.println();
    timeold_1=millis();
    }

void setup() {
  Serial.begin(9600);
  DMP_initialize();
  ESC.attach(9,0,2000);   //speed control for BLDC motor
  pinMode(3,INPUT_PULLUP);   //cotrol output for switching directions 
  pinMode(8,OUTPUT);
  pinMode(encoder_pin, INPUT);
  attachInterrupt(encoder_pin, counter, FALLING);
  timeold = millis();
  timeold_1 = millis();
}

void loop() {
    if (millis() - timeold_1 >= 50)
    {
      main_loop(); 
    }
}
