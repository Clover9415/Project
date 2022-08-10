//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
#include <SPI.h>
#include <PIDLoop.h>
#include <Pixy2.h>

#define X_CENTER         (pixy.frameWidth/2)
#define MotorA  8
#define MotorSpeedA  6
#define MotorBrakeA  4


#define MotorB  9
#define MotorSpeedB 7
#define MotorBrakeB 5

#define CW  HIGH
#define CCW LOW

#define showComments  1 // shows comments in serial monitor

int8_t i;
  int8_t lineInfo;
  int32_t error;
  int left, right;
  char buf[96];

Pixy2 pixy;
PIDLoop headingLoop(5000, 0, 0, false);
void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  pinMode(MotorA, OUTPUT);
  pinMode(MotorSpeedA, OUTPUT);
  pinMode(MotorBrakeA, OUTPUT);

  // motor B pin assignment
  pinMode(MotorB, OUTPUT);
  pinMode(MotorSpeedB, OUTPUT);
  pinMode(MotorBrakeB, OUTPUT);


  pixy.init();
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  Serial.println(pixy.changeProg("line"));
  pixy.init();
  // Turn on both lamps, upper and lower for maximum exposure
  pixy.setLamp(1, 1);
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  pixy.changeProg("line");

  // look straight and down
  pixy.setServos(560, 1000);

}
void brake(char motor, int brk)
{
  if (motor == 'A')
  {
    digitalWrite(MotorBrakeA, brk);// brake
    delay(1000);
  } else {
    digitalWrite(MotorBrakeB, brk);// brake
    delay(1000);

  }
  Serial.print("stop ");
  Serial.println("lineInfo");
}


//motor is char A or B refering to motor A or B.
//dir is motor direction, CW or CCW
//is PWM value between 0 to 255
void driveMotor(char motor, int dir, int speed)
{
  int motorPin;
  int motorSpeedPin;

  if (motor == 'A')
  {
    motorPin      = MotorA;
    motorSpeedPin = MotorSpeedA;
  } else {
    motorPin      = MotorB;
    motorSpeedPin = MotorSpeedB;
  }
  digitalWrite(motorPin, dir);// set direction for motor
  analogWrite(motorSpeedPin, speed);// set speed of motor
}//driveMotor end


void followline()
{
  if (lineInfo <= 0)
  {
    brake('A', 1); //  brakes for A are applied
    brake('B', 1); //  brakes for A are applied
    Serial.print("stop ");
    //Serial.println(res);
    return;
  }
  else {
    driveMotor('A', CW, 150);// motot A rotate CCW at 100 PWM value
    driveMotor('B', CW, 145);// motot B rotate CW at 145 PWM value
    brake('A', 0); //  brakes for A are applied
    brake('B', 0); //  brakes for A are applied
  }
  
  if (lineInfo & LINE_VECTOR)
  {
    // Calculate heading error with respect to m_x1, which is the far-end of the vector,
    // the part of the vector we're heading toward.
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;

    pixy.line.vectors->print();

    // Perform PID calcs on heading error.
    headingLoop.update(error);

    // separate heading into left and right wheel velocities.
    left = headingLoop.m_command;
    right = -headingLoop.m_command;

    // If vector is heading away from us (arrow pointing up), things are normal.
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1)
    {
     driveMotor('A', CW, 200 ); // motor A rotates clockwise moving forward
        driveMotor('B', CW, 150);// motot B rotate CW at 145 speed
      
      // ... but slow down a little if intersection is present, so we don't miss it.
      if (pixy.line.vectors->m_flags & LINE_FLAG_INTERSECTION_PRESENT)
      {
        driveMotor('A', CW, 120 ); // motor A rotates clockwise moving forward
        driveMotor('B', CW, 120);// motot B rotate CW at 145 speed

      }

    }
    else  // If the vector is pointing down, or down-ish, we need to go backwards to follow.
    {
      driveMotor('A', CW, 200);// motot A rotate CW at 150 speed
      driveMotor('B', CW, 100);// motot B rotate CW at 150 speed

    }

  }
  if (lineInfo & LINE_INTERSECTION)
  {

    pixy.line.intersections->print();
  }

}






void loop()
{
  

  lineInfo = pixy.line.getMainFeatures();
 followline();
}
