

#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h> 
#include "TB6612FNG.h"


const int mpu6050SDA_PIN = 21;
const int mpu6060SCL_PIN = 22;

const int motorPWMA = 32;
const int motorA1 = 33; 
const int motorA2 = 25;
const int motorSTBY = 26;

const int motorPWMB = 12;
const int motorB1 = 27; 
const int motorB2 = 14;


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
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int32_t gyro[3];

double Kp = 20.0;
double Ki = 1.0;
double Kd = 1.5;
double rollOffset = -3.0;

double integralTheta = 0;


MPU6050 mpu; 
Tb6612fng motors(motorSTBY, motorA1, motorA2, motorPWMA, motorB1, motorB2, motorPWMB);


void setup() {
    Wire.begin(mpu6050SDA_PIN,mpu6060SCL_PIN);  
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

     // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(31);
    mpu.setYGyroOffset(15);
    mpu.setZGyroOffset(62);

    mpu.setXAccelOffset(1827); 
    mpu.setYAccelOffset(25); 
    mpu.setZAccelOffset(1343); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    
    motors.begin();
}

double pwm;
int startupCount = 0; //起動後IMUが落ち着くまで待つためのカウンタ

void loop() {
  if(startupCount < 3000) {
    Serial.println("starting");
    startupCount += 1;
    return;
  }
  
  // put your main code here, to run repeatedly:
  if (dmpReady) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGyro(gyro, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      double _roll = ypr[2]*180/M_PI + rollOffset;
      double roll = _roll/360.0;
      //float gyroX = (float)(gyro[0])/131.0/360.0;
      double gyroX = ((gyro[0])/131.0/360)/360;

      integralTheta += roll;

      pwm = Kp*roll + Ki*integralTheta + Kd*gyroX;
      pwm = max((double)-1.0, min((double)1.0, pwm));
      /*
      Serial.print("pwm:");
      Serial.print(pwm);
      
      Serial.print(" gyroX:");
      Serial.print(gyroX);
            
      Serial.print(" roll:");
      Serial.print(roll);
      Serial.print(" _roll:");
      Serial.println(_roll);
      */
      if(abs(_roll) > 25.0) {
        pwm = 0.0;
        integralTheta = 0.0;
      }
      motors.drive(pwm);
      
    }
    
  }

}
