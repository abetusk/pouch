/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
//#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// once working...
//#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMIN  250 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  450 // this is the 'maximum' pulse length count (out of 4096)

//#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  650 // this is the 'maximum' pulse length count (out of 4096)

#define STATE_NONE      0
#define STATE_IDLEMOVE  1
#define STATE_JOYMOVE   2
#define STATE_JOYMOVE_ALT   3
#define STATE_LAST 4

#define INPUT_PIN 2


//#define COUNTDOWN 1024
//#define COUNTDOWN 2048
#define COUNTDOWN 8192
//#define DIRDEL 10
#define DIRDEL 5
int g_countdown=0;
int g_dir=DIRDEL;
uint16_t g_servopos = (SERVOMAX - SERVOMIN)/2;


//int g_state = STATE_IDLEMOVE;
int g_state = STATE_NONE;
int g_prev_button = 0;

int servomin[16];
int servomax[16];
int servopos[16];
int servodir[16];

int movingcounter[16];



  
//   right3          left3
// right2     ^-^      left2
//  right1   front   left1
//     right0     left0
  
//   (15,13)        (5,6)
// (14,12)            (3,7)
//  (9,10)           (2,1)
//     (8,11)     (4,0)
  

void setup() {
  int i;
  int r = (int)random(16);

  for (i=0; i<16; i++) { movingcounter[i] = 0; }

  Serial.begin(9600);
  Serial.println("Spider Jacket");

  pinMode(INPUT_PIN, INPUT);

  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  for (i=0; i<16; i++) {
    servomin[i] = SERVOMIN;
    servomax[i] = SERVOMAX;
    servopos[i] = (servomax[i] - servomin[i])/2 + servomin[i];
    servodir[i] = DIRDEL;
  }
  
  // special cases
  //servomin[3] = 250;
  //servopos[3] = (servomax[3] - servomin[3])/2 + servomin[3];
}


  
//   right3          left3
// right2     ^-^      left2
//  right1   front   left1
//     right0     left0

// old:
//   (15,13)        (5,6)
// (14,12)            (3,7)
//  (9,10)           (2,1)
//     (8,11)     (4,0)

// current:
// (lower,upper)
//
//   (7,6)        (12,14)
// (0,4)            (11,13)
//  (5,3)           (10,9)
//     (1,2)      (15,8)

int arm_map[16] = {15,13,14,12,9,10,8,11,5,6,3,7,2,1,4,0};

#define ARM_MOVING_COUNT 10

void init_idlemove() {
  int i;
  
  //DEBUG
  for (i=0;  i<16; i++) { servodir[i] = 0; }

  servopos[7] = servomin[7];  servodir[7] = DIRDEL;
  servopos[6] = servomax[6];  servodir[6] = -DIRDEL;
  
  // servo 0 reversed?
  servopos[0] = servomin[0];  servodir[0] = DIRDEL;
  servopos[4] = servomin[4];  servodir[4] = DIRDEL;

  servopos[5] = servomin[5];  servodir[5] = DIRDEL;
  servopos[3] = servomax[3];  servodir[3] = -DIRDEL;
  
  // servo 1 reversed?
  servopos[1] = servomin[1];  servodir[1] = DIRDEL;
  servopos[2] = servomin[2];  servodir[2] = DIRDEL;

  servopos[12] = servomin[12];  servodir[12] = DIRDEL;
  servopos[14] = servomax[14];  servodir[14] = -DIRDEL;

  servopos[11] = servomax[11];  servodir[11] = -DIRDEL;
  servopos[13] = servomin[13];  servodir[13] = DIRDEL;

  servopos[10] = servomin[10];  servodir[10] = DIRDEL;
  servopos[9] = servomax[9];  servodir[9] = -DIRDEL;

  servopos[15] = servomax[15];  servodir[15] = -DIRDEL;
  servopos[8] = servomin[8];  servodir[8] = DIRDEL;

  
  //for (i=0; i<16; i++) {   servodir[i] = 1;  }
  
}

void update_idlemove() {
  int i, mcount, r, r1;

  for (i=0; i<16; i++) {

    if (servodir[i]>0) {
      servopos[i] += servodir[i];
      if (servopos[i] > servomax[i]) {
        servodir[i] = -DIRDEL;
        servopos[i] = servomax[i];
      }
      
    }
    else {
      servopos[i] += servodir[i];
      if (servopos[i] < servomin[i]) {
        servodir[i] = DIRDEL;
        servopos[i] = servomin[i];
      }
    }
    
    pwm.setPWM(i , 0, servopos[i]);
    
  }
}

void update_randmove() {
  int i, mcount, r, r1;
  //int arm_map[16] = {15,13,14,12,9,10,8,11,5,6,3,7,2,1,4,0};
  
  //   right3          left3
  // right2     ^-^      left2
  //  right1   front   left1
  //     right0     left0
  
  // old:
  //   (15,13)        (5,6)
  // (14,12)            (3,7)
  //  (9,10)           (2,1)
  //     (8,11)      (4,0)



  mcount=0;
  for (i=0; i<16; i++) {
    if (movingcounter[i]>0) {
      mcount++;
    }
  }
  
  if (mcount < ARM_MOVING_COUNT) {
    r = (int)random(8);
    r1 = (int)random(100);
    movingcounter[ arm_map[2*r] ]     = 100 + r1;
    movingcounter[ arm_map[2*r + 1] ] = 100 + r1;
  }
  
  
  for (i=0; i<16; i++) {
    
    if (movingcounter[i] > 0) {
      movingcounter[i]--;
    } else { continue; }
    
    if (servodir[i]>0) {
      servopos[i] += servodir[i];
      if (servopos[i] > servomax[i]) {
        servodir[i] = -DIRDEL;
        servopos[i] = servomax[i];
      }
      
    }
    else {
      servopos[i] += servodir[i];
      if (servopos[i] < servomin[i]) {
        servodir[i] = DIRDEL;
        servopos[i] = servomin[i];
      }
    }
    
    pwm.setPWM(i , 0, servopos[i]);
    
  }
  
}

void update_joymove_alt() {
  int i;
  
  //   right3          left3
  // right2     ^-^      left2
  //  right1   front   left1
  //     right0     left0
  
  // current:
  // (lower,upper)
  //
  //   (7,6)        (12,14)
  // (0,4)            (11,13)
  //  (5,3)           (10,9)
  //     (1,2)      (15,8)

  
  // lower
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (1.0 / 1023.0);
  uint16_t pulselen0 = (uint16_t)((float)(SERVOMAX-SERVOMIN)*voltage) + SERVOMIN;

  servopos[7] = (int)((float)(servomax[7]-servomin[7])*voltage) + servomin[7];
  servopos[0] = (int)((float)(servomax[0]-servomin[0])*(voltage)) + servomin[0];
  servopos[5] = (int)((float)(servomax[5]-servomin[5])*voltage) + servomin[5];
  servopos[1] = (int)((float)(servomax[1]-servomin[1])*(voltage)) + servomin[1];

  servopos[12] = (int)((float)(servomax[12]-servomin[12])*(1.0-voltage)) + servomin[12];
  servopos[11] = (int)((float)(servomax[11]-servomin[11])*(voltage)) + servomin[11];
  servopos[10] = (int)((float)(servomax[10]-servomin[10])*(1.0-voltage)) + servomin[10];
  servopos[15] = (int)((float)(servomax[15]-servomin[15])*(voltage)) + servomin[15];

  // upper 
  sensorValue = analogRead(A1);
  voltage = sensorValue * (1.0 / 1023.0);
  uint16_t pulselen1 = (uint16_t)((float)(SERVOMAX-SERVOMIN)*voltage) + SERVOMIN;

  servopos[6] = (int)((float)(servomax[6]-servomin[6])*voltage) + servomin[6];
  servopos[4] = (int)((float)(servomax[4]-servomin[4])*(1.0-voltage)) + servomin[4];
  servopos[3] = (int)((float)(servomax[3]-servomin[3])*voltage) + servomin[3];
  servopos[2] = (int)((float)(servomax[2]-servomin[2])*(1.0-voltage)) + servomin[2];

  servopos[14] = (int)((float)(servomax[14]-servomin[14])*(voltage)) + servomin[14];
  servopos[13] = (int)((float)(servomax[13]-servomin[13])*(1.0-voltage)) + servomin[13];
  servopos[9] = (int)((float)(servomax[9]-servomin[9])*(voltage)) + servomin[9];
  servopos[8] = (int)((float)(servomax[8]-servomin[8])*(1.0-voltage)) + servomin[8];


  for (i=0; i<16; i++) {
    pwm.setPWM(i, 0, servopos[i]);
  }

}


void update_joymove() {
  int i;
  
  
  // lower
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (1.0 / 1023.0);
  uint16_t pulselen0 = (uint16_t)((float)(SERVOMAX-SERVOMIN)*voltage) + SERVOMIN;

  servopos[0] = (int)((float)(servomax[0]-servomin[0])*(1.0-voltage)) + servomin[0];
  servopos[1] = (int)((float)(servomax[1]-servomin[1])*(1.0-voltage)) + servomin[1];
  servopos[5] = (int)((float)(servomax[5]-servomin[5])*voltage) + servomin[5];
  servopos[7] = (int)((float)(servomax[7]-servomin[7])*voltage) + servomin[7];
  

  servopos[10] = (int)((float)(servomax[10]-servomin[10])*(1.0-voltage)) + servomin[10];
  servopos[11] = (int)((float)(servomax[11]-servomin[11])*(1.0-voltage)) + servomin[11];
  servopos[12] = (int)((float)(servomax[12]-servomin[12])*(1.0-voltage)) + servomin[12];
  servopos[15] = (int)((float)(servomax[15]-servomin[15])*(1.0-voltage)) + servomin[15];

  // upper 
  sensorValue = analogRead(A1);
  voltage = sensorValue * (1.0 / 1023.0);
  uint16_t pulselen1 = (uint16_t)((float)(SERVOMAX-SERVOMIN)*voltage) + SERVOMIN;


  servopos[2] = (int)((float)(servomax[2]-servomin[2])*voltage) + servomin[2];
  servopos[3] = (int)((float)(servomax[3]-servomin[3])*voltage) + servomin[3];
  servopos[4] = (int)((float)(servomax[4]-servomin[4])*voltage) + servomin[4];
  servopos[6] = (int)((float)(servomax[6]-servomin[6])*voltage) + servomin[6];

  servopos[8] = (int)((float)(servomax[8]-servomin[8])*(1.0-voltage)) + servomin[8];
  servopos[9] = (int)((float)(servomax[9]-servomin[9])*(1.0-voltage)) + servomin[9];
  servopos[13] = (int)((float)(servomax[13]-servomin[13])*(1.0-voltage)) + servomin[13];
  servopos[14] = (int)((float)(servomax[14]-servomin[14])*(1.0-voltage)) + servomin[14];


  //   right3          left3
  // right2     ^-^      left2
  //  right1   front   left1
  //     right0     left0
  
  // (lower,upper)
  //
  //   (7,6)        (12,14)
  // (0,4)            (11,13)
  //  (5,3)           (10,9)
  //     (1,2)      (15,8)

  for (i=0; i<16; i++) {
    pwm.setPWM(i, 0, servopos[i]);
  }

/* // old values

  // left0 lower
  pwm.setPWM(4 , 0, servopos[4]);
  // left0 upper
  pwm.setPWM(0, 0, servopos[0]);

  // left1 lower
  pwm.setPWM(2, 0, servopos[2]);
  // left1 upper 
  pwm.setPWM(1, 0, servopos[1]);
  
  
  // left2 lower
  pwm.setPWM(3, 0, servopos[3]);
  // left2 upper
  pwm.setPWM(7 , 0, servopos[7]);

  // left3 lower
  pwm.setPWM(5 , 0, servopos[5]);
  // left3 upper
  pwm.setPWM(6 , 0, servopos[6]);

  //--  

  // right0 lower
  pwm.setPWM(8, 0, servopos[8]);
  // right0 upper
  pwm.setPWM(11, 0, servopos[11]);

  // right1 lower
  pwm.setPWM(9, 0, servopos[9]);
  // right1 upper
  pwm.setPWM(10, 0, servopos[10]);

  // right2 lower
  pwm.setPWM(14 , 0, servopos[14]);
  // right2 upper
  pwm.setPWM(12 , 0, servopos[12]);

  // right3 lower
  pwm.setPWM(15 , 0, servopos[15]);
  // right3 upper
  pwm.setPWM(13 , 0, servopos[13]);
*/
}

// Debug function to help calibrate the motors and discover
// motor number.
//

void update_calibrate() {
  int i, k;
  int calib_a, calib_b;


  //   right3          left3
  // right2     ^-^      left2
  //  right1   front   left1
  //     right0     left0
  
  // (lower,upper)
  //
  //   (7,6)        (12,14)
  // (0,4)            (11,13)
  //  (5,3)           (10,9)
  //     (1,2)      (15,8)
  
  calib_a = 15; calib_b = 15;
  
  for (i=0; i<16; i++) {
    if (i==calib_a) { continue; }
    if (i==calib_b) { continue; }
    pwm.setPWM(i , 0, servopos[i]);
  }
  


  
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (1.0 / 1023.0);
  uint16_t pulselen0 = (uint16_t)((float)(SERVOMAX-SERVOMIN)*voltage) + SERVOMIN;

  servopos[calib_a] = (int)((float)(servomax[calib_a]-servomin[calib_a])*voltage) + servomin[calib_a];

  //servopos[2] = (int)((float)(servomax[2]-servomin[2])*voltage) + servomin[2];
  //servopos[3] = (int)((float)(servomax[3]-servomin[3])*voltage) + servomin[3];
  //servopos[4] = (int)((float)(servomax[4]-servomin[4])*voltage) + servomin[4];
  //servopos[5] = (int)((float)(servomax[5]-servomin[5])*voltage) + servomin[5];

  //servopos[8] = (int)((float)(servomax[8]-servomin[8])*(1.0-voltage)) + servomin[8];
  //servopos[9] = (int)((float)(servomax[9]-servomin[9])*(1.0-voltage)) + servomin[9];
  //servopos[14] = (int)((float)(servomax[14]-servomin[14])*(1.0-voltage)) + servomin[14];
  //servopos[15] = (int)((float)(servomax[15]-servomin[15])*(1.0-voltage)) + servomin[15];
 
  sensorValue = analogRead(A1);
  voltage = sensorValue * (1.0 / 1023.0);
  uint16_t pulselen1 = (uint16_t)((float)(SERVOMAX-SERVOMIN)*voltage) + SERVOMIN;

  if (calib_b != calib_a) {
    servopos[calib_b] = (int)((float)(servomax[calib_b]-servomin[calib_b])*voltage) + servomin[calib_b];
  }

  //servopos[0] = (int)((float)(servomax[0]-servomin[0])*voltage) + servomin[0];
  //servopos[1] = (int)((float)(servomax[1]-servomin[1])*voltage) + servomin[1];
  //servopos[6] = (int)((float)(servomax[6]-servomin[6])*voltage) + servomin[6];
  //servopos[7] = (int)((float)(servomax[7]-servomin[7])*voltage) + servomin[7];

  //servopos[10] = (int)((float)(servomax[10]-servomin[10])*(1.0-voltage)) + servomin[10];
  //servopos[11] = (int)((float)(servomax[11]-servomin[11])*(1.0-voltage)) + servomin[11];
  //servopos[12] = (int)((float)(servomax[12]-servomin[12])*(1.0-voltage)) + servomin[12];
  //servopos[13] = (int)((float)(servomax[13]-servomin[13])*(1.0-voltage)) + servomin[13];

 
  //   right3          left3
  // right2     ^-^      left2
  //  right1   front   left1
  //     right0     left0
  
  //   (15,13)        (5,6)
  // (14,12)            (3,7)
  //  (9,10)           (2,1)
  //     (8,11)      (4,0)


  pwm.setPWM(calib_a , 0, servopos[calib_a]);
  
  if (calib_a != calib_b) {
    pwm.setPWM(calib_b, 0, servopos[calib_b]);
  }

}

void loop() {
  int init=0;

  if (g_countdown>0) {
    g_countdown--;
    return;
  } else {
    g_countdown = COUNTDOWN;
  }

  int buttonState = 0;
  buttonState = digitalRead(INPUT_PIN);

  if ((g_prev_button == 1) && (buttonState == 0)) {
    g_state++;
    g_state %= STATE_LAST;
    init=1;
    
    Serial.print("state now ");
    Serial.println(g_state);
  }
  g_prev_button = buttonState;
  

  if (g_state == STATE_NONE) {
    init=0;
    return;
  }
  else if (g_state == STATE_IDLEMOVE) {
    if (init) { init_idlemove(); }
    init=0;
    update_idlemove();
  }
  else if (g_state == STATE_JOYMOVE) {
    
    Serial.println("joymove state");
    
    init=0;
    update_joymove();
  }
  else if (g_state == STATE_JOYMOVE_ALT) {
    
    Serial.println("joymove_alt state");
    
    init=0;
    update_joymove_alt();
  }


}
