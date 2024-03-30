#include <Arduino.h>
#include <Servo.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "Leg.h"
#include "RP2040_PWM.h"

#include "MPU6050Sensor.h"
#include <Wire.h>
#include <VL53L0X.h>

//Walk states
#define stop 0
#define walk1 1
#define walk2 2

//Right states
#define stop 0
#define right1 1
#define right2 2

//Stairs states
#define stop 0
#define stairs1 1
#define stairs2 2

//For tests
uint8_t test_walk=0;
uint8_t test_stairs=1;

uint8_t object_notdetected=0;

VL53L0X tof;
MPU6050Sensor mpu(18,19);

MyServo myservo[8];
Leg FrontLeft(myservo[2], myservo[3]);
Leg FrontRight(myservo[0], myservo[1]);
Leg BackLeft(myservo[4], myservo[5]);
Leg BackRight(myservo[6], myservo[7]);

uint32_t t;
uint32_t interval = 100;
uint32_t last_cycle;

typedef struct {
  int state, new_state;
  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// State machines
fsm_t Walk, Stairs, Right;

//funtion to set state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state; 

    fsm.tes = millis();
    fsm.tis = 0;
  }
}

//function to update tis
void update_tis()
{
  Walk.tis = millis() - Walk.tes;
  Stairs.tis = millis() - Stairs.tes;
  Right.tis = millis() - Right.tes;
}

//function to calculate parabolic angle
float calculateParabolicAngle(float t, float start_angle, float peak_angle, float end_angle, float total_time) {

  // Vertex (h, k) of the parabola
  float h = total_time / 2;
  float k = peak_angle;

  float a = (start_angle - k) / (h * h);
  float angle = a * (t - h) * (t - h) + k;

  return angle;
}

//function to calibrate servos
void calibrate()
{
  
  myservo[0].calibrate(2500, 500, 2200, 1900, 1220, 1120); //knee
  myservo[1].calibrate(2500, 500, 600, 1125, 1650, 1750); //ankle
  
  myservo[2].calibrate(2500, 500, 630, 1100, 1600, 1800); //knee
  myservo[3].calibrate(2500, 500, 2350, 1855, 1300, 1260); //ankle

  myservo[4].calibrate(2500, 500, 2500, 2050, 1550, 1230); //knee
  myservo[5].calibrate(2500, 500, 550, 1050, 1530, 1650); //ankle

  myservo[6].calibrate(2500, 500, 700, 1100, 1650, 1850); //knee
  myservo[7].calibrate(2500, 500, 1750, 1150, 680, 600); //ankle
   
}

//Initial walk angles
void base_walk()
{
  FrontLeft.ankle.setAngle(60);
  FrontRight.ankle.setAngle(60);
  BackLeft.ankle.setAngle(60);
  BackRight.ankle.setAngle(60);
  FrontLeft.knee.setAngle(35);
  FrontRight.knee.setAngle(110);
  BackLeft.knee.setAngle(60);
  BackRight.knee.setAngle(110);
}

//Initial right angles
void base_right()
{
  FrontLeft.ankle.setAngle(60);
  FrontRight.ankle.setAngle(60);
  BackLeft.ankle.setAngle(60);
  BackRight.ankle.setAngle(60);
  FrontLeft.knee.setAngle(45);
  FrontRight.knee.setAngle(45);
  BackLeft.knee.setAngle(0);
  BackRight.knee.setAngle(0);
}

//function to walk
void ME_Walk(int speed)
{
  t = Walk.tis;
  
  switch (Walk.state)
  {
    case stop: 
        if(test_walk==1 && Right.state == stop)
        {
          Walk.new_state = walk1;
        }
      break;
    
    case walk1:
      FrontRight.knee.angulo = map(t, 0, speed, 110, 60);
      FrontRight.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      FrontLeft.knee.angulo = map(t, 0, speed, 35, 110);
      BackRight.knee.angulo = map(t, 0, speed, 110, 60);
      BackLeft.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      BackLeft.knee.angulo = map(t, 0, speed, 60, 110);

      if(t > speed)
      {
        Walk.new_state = walk2;
      }

      break;

    case walk2:
      FrontLeft.knee.angulo = map(t, 0, speed, 110, 35);
      FrontLeft.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      FrontRight.knee.angulo = map(t, 0, speed, 60, 110);
      BackLeft.knee.angulo = map(t, 0, speed, 110, 60);
      BackRight.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      BackRight.knee.angulo = map(t, 0, speed, 60, 110);
      
      if(t > speed)
      {
        
        if(tof.readRangeContinuousMillimeters()<=250)
        {
        
          base_right();
          Walk.new_state = stop;
          Right.new_state = right1;

        }
        else
        {
          Walk.new_state = walk1;
        }
      } 

      break;
  }
}

//function to climb stairs
void ME_Stairs(int speed)
{
  t = Stairs.tis;
  
  switch (Stairs.state)
  {
    case stop: 
        if(test_stairs==1 && Right.state == stop && Walk.state == stop)
        {
          Stairs.new_state = stairs1;
        }
      break;
    
    case stairs1:
      FrontRight.knee.angulo = map(t, 0, speed, 110, 0);
      FrontRight.ankle.angulo = calculateParabolicAngle(t, 90, 0, 90, speed);
      FrontLeft.knee.angulo = map(t, 0, speed, 0, 110);
      BackRight.knee.angulo = map(t, 0, speed, 110, 0);
      BackLeft.ankle.angulo = calculateParabolicAngle(t, 90, 0, 90, speed);
      BackLeft.knee.angulo = map(t, 0, speed, 0, 110);

      if(t > speed)
      {
        Stairs.new_state = stairs2;
      }

      break;

    case stairs2:
      FrontLeft.knee.angulo = map(t, 0, speed, 110, 0);
      FrontLeft.ankle.angulo = calculateParabolicAngle(t, 90, 0, 90, speed);
      FrontRight.knee.angulo = map(t, 0, speed, 0, 110);
      BackLeft.knee.angulo = map(t, 0, speed, 110, 0);
      BackRight.ankle.angulo = calculateParabolicAngle(t, 90, 0, 90, speed);
      BackRight.knee.angulo = map(t, 0, speed, 0, 110);

      if(t > speed)
      {
        Stairs.new_state = stairs1;
      }

      break;
  }
}

//function to turn right
void ME_Right( int speed) {

  t = Right.tis;

  switch(Right.state){

    case stop:
    break;

    case right1:
  
    BackRight.knee.angulo = map(t, 0, speed, 0, 45);
    BackRight.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);

    FrontRight.knee.angulo = map(t, 0, speed, 45, 0);
    BackLeft.knee.angulo = map(t, 0, speed, 0, 45);
    FrontLeft.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    FrontLeft.knee.angulo = map(t, 0, speed, 45, 0);

    if(Right.tis > speed)
    {
      Right.new_state = 2;
    }
    break;

  case right2:

    FrontRight.knee.angulo = map(t, 0, speed, 0, 45);
    FrontRight.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    BackRight.knee.angulo = map(t, 0, speed, 45, 0);
    FrontLeft.knee.angulo = map(t, 0, speed, 0, 45);
    BackLeft.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
    BackLeft.knee.angulo = map(t, 0, speed, 45, 0);

    if(Right.tis > speed)
    {
      
      if(object_notdetected==0)
      {
        Right.new_state = right1;
      }
     
      if(object_notdetected==1)
      {
      BackRight.knee.angulo = map(t, 0, speed, 0, 45);
      BackRight.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      FrontRight.knee.angulo = map(t, 0, speed, 45, 0);
      BackLeft.knee.angulo = map(t, 0, speed, 0, 45);
      FrontLeft.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      FrontLeft.knee.angulo = map(t, 0, speed, 45, 0);

      FrontRight.knee.angulo = map(t, 0, speed, 0, 45);
      FrontRight.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      BackRight.knee.angulo = map(t, 0, speed, 45, 0);
      FrontLeft.knee.angulo = map(t, 0, speed, 0, 45);
      BackLeft.ankle.angulo = calculateParabolicAngle(t, 60, 45, 60, speed);
      BackLeft.knee.angulo = map(t, 0, speed, 45, 0);
        
      Right.new_state = stop;
      object_notdetected=0;
      }
      
      if (tof.readRangeContinuousMillimeters()>400)
      {
        object_notdetected=1;

      }
      
    }
    break;

 }
}

//function to write outputs
void write_outputs()
{
  BackRight.knee.updateAngle();
  BackRight.ankle.updateAngle();
  FrontRight.knee.updateAngle();
  FrontRight.ankle.updateAngle();
  FrontLeft.knee.updateAngle();
  FrontLeft.ankle.updateAngle();
  BackLeft.knee.updateAngle();
  BackLeft.ankle.updateAngle();
}

void setup(){

  Serial.begin(115200);

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  mpu.begin();

  tof.setTimeout(500);

  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  
  tof.startContinuous();

  myservo[0].attach(0);
  myservo[1].attach(1);
  myservo[2].attach(2);
  myservo[3].attach(3);
  myservo[4].attach(10);
  myservo[5].attach(5);
  myservo[6].attach(6);
  myservo[7].attach(7);

  set_state(Walk, 0);
  Walk.new_state = 0;
  set_state(Stairs, 0);
  Stairs.new_state = 0;
  set_state(Right, 0);  
  Right.new_state = 0;

  calibrate();
}

void loop()
{
  unsigned long now = millis();
  if (now - last_cycle > interval) {
    last_cycle = now;
   
    update_tis();
    
    mpu.update(now);
    Serial.print(mpu.getAngleX());
    Serial.print(mpu.getAngleY());
    Serial.print(mpu.getAngleZ());
    
    ME_Stairs(500);
    write_outputs();
    ME_Right(500);
    write_outputs();
    ME_Walk(300);
    write_outputs();

    set_state(Walk, Walk.new_state);
    set_state(Stairs, Stairs.new_state);
    set_state(Right, Right.new_state);

  }

}



