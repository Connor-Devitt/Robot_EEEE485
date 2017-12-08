#include <Servo.h>
#include <Encoder.h>
#include <Motor.h>
#include <ServoLoop.h>
#include <Wire.h>
#include <PixyI2C.h>

//Pin definitions (3, 4, 16, 17 reserved for encoders)
#define MOTOR_L_PWM_PIN 21
#define MOTOR_L_DIR_PIN 16
#define MOTOR_L_ENBL_PIN 13

#define MOTOR_R_PWM_PIN 20
#define MOTOR_R_DIR_PIN 17
#define MOTOR_R_ENBL_PIN 13

#define MOTOR_SLOW_SPEED 10
#define MOTOR_MED_SPEED 15

#define SERVO_MOVE_DELAY 200

#define SERVO_ARM 12

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

Servo servo_Arm;

//Encoders and motors
Encoder *encoder_l, *encoder_r;
IntervalTimer velTimer;

Motor *motor_left, *motor_right;

volatile int oldCnt_l = 0, oldCnt_r = 0;
volatile int newCnt_l = 0, newCnt_r = 0;
volatile int diffCnt_l = 0, diffCnt_r = 0;

//Camera
PixyI2C pixy;
ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);


void calcVel_tisr(void);

void setup() {
  Serial.begin(9600);

  servo_Arm.attach(12);
  servo_Arm.write(0);

  //Create each motor
  motor_left = new Motor(MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN, MOTOR_L_ENBL_PIN);
  motor_right = new Motor(MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN, MOTOR_R_ENBL_PIN);

  //Setup encoders
  encoder_l = new Encoder(23, 22);
  encoder_r = new Encoder(9, 8);

  encoder_l->write(0);
  encoder_r->write(0);

  velTimer.begin(calcVel_tisr, 10000);

  //Camera
  pixy.init();

  delay(1000);
}

void loop() {
  if (pixy.getBlocks()) {
    Serial.println("CAMERA DETECTED");

    //Stop moving
    motor_left->setVel_basic(0);
    motor_right->setVel_basic(0);
    delay(100);

    if (pixy.getBlocks()) {

      panLoop.update(X_CENTER - pixy.blocks[0].x);
      tiltLoop.update(pixy.blocks[0].y - Y_CENTER);

      pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

      int camera_angle = panLoop.m_pos - PIXY_RCS_CENTER_POS;
      if (camera_angle > 100) { //camera is right
        motor_right->setVel(MOTOR_MED_SPEED, 1);
        motor_left->setVel(MOTOR_MED_SPEED, -1);
        motor_left->setCurrentVel(diffCnt_l);
        motor_right->setCurrentVel(diffCnt_r);

        servo_Arm.write(0);
      }
      else if (camera_angle < -100) { //camera to left
        motor_right->setVel(MOTOR_MED_SPEED, -1);
        motor_left->setVel(MOTOR_MED_SPEED, 1);
        motor_left->setCurrentVel(diffCnt_l);
        motor_right->setCurrentVel(diffCnt_r);

        servo_Arm.write(0);
      }
      else { //camera centered
        motor_right->setVel(MOTOR_SLOW_SPEED, -1);
        motor_left->setVel(MOTOR_MED_SPEED, -1);
        motor_left->setCurrentVel(diffCnt_l);

        servo_Arm.write(60);
        motor_right->setCurrentVel(diffCnt_r);
      }

      if ((tiltLoop.m_pos - PIXY_RCS_CENTER_POS) > 400) {
        servo_Arm.write(0);

        while (!(motor_left->setVel(0) & motor_right->setVel(0))) {
          motor_left->setCurrentVel(diffCnt_l);
          motor_right->setCurrentVel(diffCnt_r);
        }

      }
      else {
        servo_Arm.write(60);
      }

    }
    else {
      pixy.setServos(500, 500);
    }
  }
}
void calcVel_tisr(void) {
  oldCnt_l = newCnt_l;
  newCnt_l = encoder_l->read();
  diffCnt_l = newCnt_l - oldCnt_l;

  oldCnt_r = newCnt_r;
  newCnt_r = encoder_r->read();
  diffCnt_r = newCnt_r - oldCnt_r;
}

