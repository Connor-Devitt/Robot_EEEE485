#include <Encoder.h>
#include <Servo.h>
#include <Motor.h>


//Pin definitions (3, 4, 16, 17 reserved for encoders)
#define MOTOR_L_PWM_PIN 20
#define MOTOR_L_DIR_PIN 11
#define MOTOR_L_ENBL_PIN 13

#define MOTOR_R_PWM_PIN 21
#define MOTOR_R_DIR_PIN 12
#define MOTOR_R_ENBL_PIN 13

#define MOTOR_SLOW_SPEED 8
#define MOTOR_MED_SPEED 15

#define SENSOR_IR_PIN A1
#define SERVO_IR_PIN 14
#define SERVO_MOVE_DELAY 200

#define LINE_L_PIN 2
#define LINE_C_PIN 3
#define LINE_R_PIN 4

//present state and state definition masks
byte current_state = 0;
const byte OBJECT_DETECTED  = 1;
const byte LINE_DETECTED = 2;

//Servos
Servo servo_IR;
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

void setup() {
  Serial.begin(9600);

  //Setup sensors
  pinMode(SENSOR_IR_PIN, INPUT);

  //Setup servos
  servo_IR.attach(SERVO_IR_PIN);
  servo_IR.write(90 - servo_center_offset);

  //Create each motor
  motor_left = new Motor(MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN, MOTOR_L_ENBL_PIN);
  motor_right = new Motor(MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN, MOTOR_R_ENBL_PIN);

  //Setup encoders
  encoder_l = new Encoder(9, 8);
  encoder_r = new Encoder(16, 17);

  encoder_l->write(0);
  encoder_r->write(0);

  velTimer.begin(calcVel_tisr, 10000);

  //Setup line follower
  pinMode(LINE_L_PIN, INPUT);
  pinMode(LINE_C_PIN, INPUT);
  pinMode(LINE_R_PIN, INPUT);

  delay(1000);
}

void loop() {
  if (current_state & OBJECT_DETECTED) {
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
  else if (current_state & LINE_DETECTED) {
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
        motor_left->setVel_basic(0);

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
        motor_left->setVel_basic(0);

        break;
      case 4: //Right line

        //Turn right
        motor_right->setVel_basic(0);
        motor_left->setVel(MOTOR_SLOW_SPEED);

        break;
      case 5: //Right and Left line

        //This should not happen but go forwards
        motor_right->setVel(MOTOR_SLOW_SPEED);
        motor_left->setVel(MOTOR_SLOW_SPEED);

        break;
      case 6: //Right and Middle line
        //Turn right
        motor_right->setVel_basic(0);
        motor_left->setVel(MOTOR_SLOW_SPEED);

        break;
      case 7: //All lines

        //Turn right (robot is perpendicular to line)
        motor_right->setVel_basic(0);
        motor_left->setVel(MOTOR_SLOW_SPEED);

        can_stop_following = true;

        break;
      default:
        //
        break;
    }
  }

  //update motor's current_velocity
  motor_left->setCurrentVel(diffCnt_l);
  motor_right->setCurrentVel(diffCnt_r);

  //Update sensor values
  current_distance = compute_avrg_dist();
  lines_detected = (!digitalRead(LINE_L_PIN) << 2) | (!digitalRead(LINE_C_PIN) << 1)
                   | (!digitalRead(LINE_R_PIN));

  //Update present state (for now make states mutually exclusive
  if (current_distance < threshold_distance && !(current_state)) {
    current_state |= OBJECT_DETECTED;
  }
  else if (lines_detected != 0 && !(current_state)) {
        Serial.println("LINE DETECTED");
    
        current_state |= LINE_DETECTED;
    
        //Stop moving
        motor_left->setVel_basic(0);
        motor_right->setVel_basic(0);
  }

  Serial.print("Current Distance: \t");
  Serial.println(current_distance);

  delay(5);
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
    delay(40);
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



