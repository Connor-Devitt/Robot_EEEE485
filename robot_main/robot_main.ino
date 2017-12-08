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
#define MOTOR_SANIC_SPEED 40

#define SENSOR_IR_PIN A0
#define SERVO_IR_PIN 0
#define SERVO_MOVE_DELAY 200

#define SERVO_ARM 12

#define LINE_L_PIN 7
#define LINE_C_PIN 6
#define LINE_R_PIN 5


#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

//present state and state definition masks
byte current_state = 0;
const byte OBJECT_DETECTED  = 1;
const byte LINE_DETECTED = 2;
const byte CAMERA_DETECTED = 4;

//Servos
Servo servo_IR, servo_Arm;
int servo_center_offset = 0;

//Encoders and motors
Encoder *encoder_l, *encoder_r;
IntervalTimer velTimer;

Motor *motor_left, *motor_right;

volatile int oldCnt_l = 0, oldCnt_r = 0;
volatile int newCnt_l = 0, newCnt_r = 0;
volatile int diffCnt_l = 0, diffCnt_r = 0;

//Sensors
double threshold_distance = 20.0;
double current_distance = 0;
byte lines_detected = 0;
bool can_stop_following = false;
bool can_start_camera = true;

//Camera
PixyI2C pixy;
ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);


void calcVel_tisr(void);
double compute_avrg_dist();
double ir_cal_dist(int adc_val);
int search_for_path(int scan_width);
void object_state(void);
void line_state(void);
void camera_state(void);


void setup() {
  Serial.begin(9600);

  //Setup sensors
  pinMode(SENSOR_IR_PIN, INPUT);

  //Setup servos
  servo_IR.attach(SERVO_IR_PIN);
  servo_IR.write(90 - servo_center_offset);

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

  //Setup line follower
  pinMode(LINE_L_PIN, INPUT);
  pinMode(LINE_C_PIN, INPUT);
  pinMode(LINE_R_PIN, INPUT);

//  delay(1000);
}

//********************************************************************************************************************
//  MAIN
//********************************************************************************************************************
void loop() {
  if (current_state & OBJECT_DETECTED) {
    object_state();
  }
  else if (current_state & LINE_DETECTED) {
    line_state();
  }
  else if (current_state & CAMERA_DETECTED) {
    camera_state();
  }
  else {
    motor_left->setVel(MOTOR_MED_SPEED);
    motor_right->setVel(MOTOR_MED_SPEED);
    motor_left->setCurrentVel(diffCnt_l);
    motor_right->setCurrentVel(diffCnt_r);
  }

  //update motor's current_velocity
  motor_left->setCurrentVel(diffCnt_l);
  motor_right->setCurrentVel(diffCnt_r);

  //Update sensor values
  current_distance = ir_cal_dist(analogRead(SENSOR_IR_PIN));
  lines_detected = (!digitalRead(LINE_L_PIN) << 2) | (!digitalRead(LINE_C_PIN) << 1)
                   | (!digitalRead(LINE_C_PIN));

  //Update present state (for now make states mutually exclusive
  if (current_distance < threshold_distance && !(current_state & (!LINE_DETECTED))) {
    current_state |= OBJECT_DETECTED;
  }
  //  else if (lines_detected != 0 && !(current_state)) {
  //    Serial.println("LINE DETECTED");
  //
  //    current_state |= LINE_DETECTED;
  //
  //    //Stop moving
  //    motor_left->setVel_basic(0);
  //    motor_right->setVel_basic(0);
  //  }
  else if (pixy.getBlocks() && !(current_state)) {
    Serial.println("CAMERA DETECTED");

    current_state |= CAMERA_DETECTED;

    //Stop moving
    //    motor_left->setVel_basic(0);
    //    motor_right->setVel_basic(0);
    delay(100);
  }

  delay(25);
}

void calcVel_tisr(void) {
  oldCnt_l = newCnt_l;
  newCnt_l = encoder_l->read();
  diffCnt_l = newCnt_l - oldCnt_l;

  oldCnt_r = newCnt_r;
  newCnt_r = encoder_r->read();
  diffCnt_r = newCnt_r - oldCnt_r;
}

double ir_cal_dist(int adc_val) {
  return pow(adc_val / 6700.0, -1.09425) - 3.6;
}

int search_for_path(int scan_width) {
  Serial.println("Searching for path");

  //search left
  servo_IR.write(90 - servo_center_offset);
  for (int i = servo_IR.read(); i >= 90 - scan_width; i--) {
    current_distance = ir_cal_dist(analogRead(SENSOR_IR_PIN));

    if (current_distance > threshold_distance + 10) {
      return i;
    }

    servo_IR.write(i);
    delay(40);
  }

  //search right
  //  servo_IR.write(90 - servo_center_offset);
  for (int i = servo_IR.read(); i <= 90 + scan_width; i++) {
    current_distance = ir_cal_dist(analogRead(SENSOR_IR_PIN));

    if (current_distance > threshold_distance + 10) {
      return i;
    }

    servo_IR.write(i);
    delay(20);
  }

  //Could not find path
  return -1;
}
double compute_avrg_dist() {
  int sum = 0;
  int num = 15;
  for (int i = 0; i < num; i++) {
    sum += analogRead(SENSOR_IR_PIN);

    delay(10);
  }

  return ir_cal_dist(sum / num);
}
//********************************************************************************************************************
void object_state(void) {
  //Stop moving
  motor_left->setVel_basic(0);
  motor_right->setVel_basic(0);
  motor_left->setCurrentVel(diffCnt_l);
  motor_right->setCurrentVel(diffCnt_r);

  //Search for clear path and turn towards it
  int angle;
  if ((angle = search_for_path(60)) != -1) {
    Serial.print("Found path\t");
    Serial.println(angle);
    if ((angle - servo_IR.read()) > 0) { //Turn right
      servo_IR.write(90 - servo_center_offset - (90 - angle) / 2.0); //offset center by 90 - angle over 2.0

      //Turn servo to 30 degrees off center
      Serial.println("TURNING");

      delay(SERVO_MOVE_DELAY); //delay while servo moves

      //turn until object is avoided
      while ((current_distance = ir_cal_dist(analogRead(SENSOR_IR_PIN))) < threshold_distance + 5) {
        Serial.print("Current Distance: \t");
        Serial.println(current_distance);

        //Turn right
        motor_left->setVel(MOTOR_MED_SPEED, -1);
        motor_right->setVel(0);
        motor_left->setCurrentVel(diffCnt_l);
        motor_right->setCurrentVel(diffCnt_r);
        delay(70);
      }

    }
    else { //Turn left
      servo_IR.write(90 - servo_center_offset + (90 - angle) / 2.0); //offset center by 90 - angle over 2.0
      delay(SERVO_MOVE_DELAY); //delay while servo moves

      //turn until object is avoided
      while ((current_distance = ir_cal_dist(analogRead(SENSOR_IR_PIN))) < threshold_distance + 5) {
        Serial.print("Current Distance: \t");
        Serial.println(current_distance);

        motor_right->setVel(MOTOR_MED_SPEED, -1);
        motor_left->setVel(0);
        motor_left->setCurrentVel(diffCnt_l);
        motor_right->setCurrentVel(diffCnt_r);
        delay(70);
      }
    }

    //      motor_left->setVel(0);
    //      motor_right->setVel(0);

    motor_left->setCurrentVel(diffCnt_l);
    motor_right->setCurrentVel(diffCnt_r);

    delay(500);

    if ((current_distance = ir_cal_dist(analogRead(SENSOR_IR_PIN))) > threshold_distance) {
      //Reset state
      Serial.println("Reset State");
      current_state &= ~OBJECT_DETECTED;

      //Center servo
      servo_IR.write(90 - servo_center_offset);

      //Set Motors to med speed
      while (!(motor_left->setVel(MOTOR_MED_SPEED) & motor_right->setVel(MOTOR_MED_SPEED))) {
        motor_left->setCurrentVel(diffCnt_l);
        motor_right->setCurrentVel(diffCnt_r);

        Serial.println("Setting normal Speed");

        Serial.print("\t\tLEFT: \t");
        Serial.print(diffCnt_l);
        Serial.print("\tRIGHT: \t");
        Serial.println(diffCnt_r);

        delay(70);
      }
    }
  }
  //Turn robot to the left
  else {
    Serial.println("Did not find path");
    motor_right->setVel(MOTOR_MED_SPEED);

    delay(500);
    motor_right->setVel(0);
  }
}
//********************************************************************************************************************
void line_state(void) {
  //Update lines
  lines_detected = (!digitalRead(LINE_L_PIN) << 2) | (!digitalRead(LINE_C_PIN) << 1)
                   | (!digitalRead(LINE_R_PIN));

  switch (lines_detected) {
    case 0: //No line
      //Reset state if line could end
      if (can_stop_following) {
        current_state &= ~LINE_DETECTED;
        can_stop_following = false;
      }
      //else do nothing

      break;
    case 1: //Left line

      //Turn left
      motor_right->setVel(MOTOR_SLOW_SPEED);
      motor_left->setVel(0);

      break;
    case 2: //Middle line

      //Go forwards
      motor_right->setVel(MOTOR_SLOW_SPEED);
      motor_left->setVel(MOTOR_SLOW_SPEED);

      can_stop_following = true;

      break;
    case 3: //Left and Middle

      //Turn left
      motor_right->setVel(MOTOR_SLOW_SPEED);
      motor_left->setVel(0);

      break;
    case 4: //Right line

      //Turn right
      motor_right->setVel(0);
      motor_left->setVel(MOTOR_SLOW_SPEED);

      break;
    case 5: //Right and Left line

      //This should not happen but go forwards
      motor_right->setVel(MOTOR_SLOW_SPEED);
      motor_left->setVel(MOTOR_SLOW_SPEED);

      break;
    case 6: //Right and Middle line
      //Turn right
      motor_right->setVel(0);
      motor_left->setVel(MOTOR_SLOW_SPEED);

      break;
    case 7: //All lines

      //Turn right (robot is perpendicular to line)
      motor_right->setVel(0);
      motor_left->setVel(MOTOR_SLOW_SPEED);

      can_stop_following = true;

      break;
    default:
      //
      break;
  }
}
//********************************************************************************************************************
void camera_state(void) {
  if (pixy.getBlocks()) {

    panLoop.update(X_CENTER - pixy.blocks[0].x);
    tiltLoop.update(pixy.blocks[0].y - Y_CENTER);

    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

    if ((tiltLoop.m_pos - PIXY_RCS_CENTER_POS) > 350) {
      servo_Arm.write(0);

      current_state &= ~CAMERA_DETECTED;

    }
    else {
      servo_Arm.write(60);

      int camera_angle = panLoop.m_pos - PIXY_RCS_CENTER_POS;
      if (camera_angle > 100) { //camera is right
        motor_right->setVel(MOTOR_SANIC_SPEED, 1);
        motor_left->setVel(MOTOR_SANIC_SPEED, -1);
        motor_left->setCurrentVel(diffCnt_l);
        motor_right->setCurrentVel(diffCnt_r);

        servo_Arm.write(0);
      }
      else if (camera_angle < -100) { //camera to left
        motor_right->setVel(MOTOR_SANIC_SPEED, -1);
        motor_left->setVel(MOTOR_SANIC_SPEED, 1);
        motor_left->setCurrentVel(diffCnt_l);
        motor_right->setCurrentVel(diffCnt_r);

        servo_Arm.write(0);
      }
      else { //camera centered
        motor_right->setVel(MOTOR_SANIC_SPEED, -1);
        motor_left->setVel(MOTOR_SANIC_SPEED, -1);
        motor_left->setCurrentVel(diffCnt_l);

        servo_Arm.write(60);
        motor_right->setCurrentVel(diffCnt_r);
      }
    }

  }
  else {
    pixy.setServos(500, 500);
  }
}


