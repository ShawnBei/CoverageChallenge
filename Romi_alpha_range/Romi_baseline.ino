
volatile long left_encoder_count; // used by encoder to count the rotation
volatile long right_encoder_count; // used by encoder to count the rotation

#include "pins.h"
#include "utils.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"
#include "interrupts.h"
#include "line_sensors.h"
#include "irproximity.h"
#include "mapping.h"
#include "RF_Interface.h"
#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"
#include "Pushbutton.h"

#define BAUD_RATE 38400

#define FORWARD 0
#define WALK    1
#define ROTATE  2
#define TIME    3

#define UP      1
#define DOWN    3
#define BACK    2

#define NORTH   0
#define SOUTH   1
#define WEST    2
#define EAST    3

#define SHARP_IR_PIN A0 //Pin for the IR Distance sensor
#define SHARP_IR_PIN_RIGHT A3 //Pin for the IR Distance sensor
#define SHARP_IR_PIN_LEFT A4 //Pin for the IR Distance sensor

Kinematics    Pose; //Kinematics class to store position and heading

LineSensor    LineLeft(LINE_LEFT_PIN); //Left line sensor
LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor
LineSensor    LineRight(LINE_RIGHT_PIN); //Right line sensor

SharpIR       LeftIR(SHARP_IR_PIN_LEFT); //Distance sensor
SharpIR       MidIR(SHARP_IR_PIN); //Distance sensor
SharpIR       RightIR(SHARP_IR_PIN_RIGHT); //Distance sensor

Imu           Imu;

Magnetometer  Mag; // Class for the magnetometer

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftSpeedControl( 3.5, 20.9, 0.04 );
PID           RightSpeedControl( 3.5, 20.9, 0.04 );
PID           HeadingControl( 1, 0, 0 );
PID           ForwardHeadingControl( 3, 0, 0 );

//PID           RightSpeedControl( 3.5, 20.9, 0.04 );
Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

//Use these variables to set the demand of the speed controller
bool use_speed_controller = true;
float left_speed_demand = 0;
float right_speed_demand = 0;

// State machine
int STATE = 1;

// Going forward
int COUNTS_PER_GRID = COUNTS_PER_MM * GRID_DIST;
long count = COUNTS_PER_GRID;

// Determine direction
int FLAG = FORWARD;
float dir;
int FLAG_SMALLEST;
bool FLAG_THETA;

// Going from 180 to -90
int MINUS90;

// For reading the surrounding map
char n_val;
char s_val;
char e_val;
char w_val;
float theta = 0;
int p = 0;

unsigned long timestamp;

unsigned long rotation_time = millis();

unsigned long T = 135000;
 float forward_bias;
 int finish;

float lar;
float sm =99;
int i;
float vals;

unsigned long read_time;
void setup()
{

  // These two function set up the pin
  // change interrupts for the encoders.
  setupLeftEncoder();
  setupRightEncoder();
  startTimer();

  //Set speed control maximum outputs to match motor
  LeftSpeedControl.setMax(100);
  RightSpeedControl.setMax(100);

  HeadingControl.setMax(5);

  //Calibrate the IR sensors
  float alpha = 0.01;
//  LeftIR.setAlpha(alpha);
  MidIR.setAlpha(alpha);
//  RightIR.setAlpha(alpha);
  
  LeftIR.calibrate();
  MidIR.calibrate();
  RightIR.calibrate();

  /*
   * SCALER
   */
  //Pose.setScaler(0.98);

  // Set the random seed for the random number generator
  // from A0, which should itself be quite random.
  randomSeed(analogRead(A0));
  
  
  // Initialise Serial communication
  Serial.begin( BAUD_RATE );
  delay(1000);
  Serial.println("Board Reset");

  // Romi will wait for you to press a button and then print
  // the current map.
  //
  // !!! A second button press will erase the map !!!
  ButtonB.waitForButton();  

  Map.printMap();
  Map.printMetrics();

  // Watch for second button press, then begin autonomous mode.
  ButtonB.waitForButton();  

  Map.resetMap();
  Serial.println("Map Erased - Mapping Started");

  // DRAW MAP
  // Map types: 
  // - "inverse-circular"
  // - "BF"
  // - "circular"
//  String mapType = "circular";
//  Map.initMap(mapType);

  LeftSpeedControl.reset();
  RightSpeedControl.reset();
  left_speed_demand = 0;
  right_speed_demand = 0;

  timestamp = millis();
  read_time = millis();
}

void loop() {

  Pose.update();
  
//  unsigned long elapsed_time = millis() - timestamp;
//  
//  if (elapsed_time > T){
//    STATE = 3;
//    stop_romi();
//    play_tone(3);
//  }
  //doMapping();
  
  float mid_distance   = MidIR.getDistanceInMM();
  
  unsigned long elapsed_time = millis()-read_time;
  
  if (elapsed_time > 1000*3){
   sm = 1000;
   lar= 0;
   read_time = millis();
  }
   Serial.print("Smallest: ");
   Serial.println(sm);
   Serial.print("Largest: ");
   Serial.println(lar);
  if (mid_distance > lar){
    lar = mid_distance;
  }
  if (mid_distance < sm){
    sm = mid_distance;
  }
  //randomwalk();
  
  delay(2);
}
void randomwalk(){
  switch(STATE){
    case 1: doMovement();
            break;
    case 2: rotate();
            break;
    case 3: outofbounds();
            break;
    case 4: Serial.println("TIME");
            break;
    default: Serial.println("ERROR");
      
  }

}

void rotate(){
  int condition = (millis() - rotation_time) > 1000;

  if (condition){
    STATE = 1;
  }else{
    left_speed_demand = -10;
    right_speed_demand = 10;
  }
}

void outofbounds(){
  int condition = (millis() - rotation_time) > 1000;
  
  if (condition){
    STATE = 1;
    left_speed_demand = 10;
    right_speed_demand = 10;
    delay(500);
    
  }else{
    left_speed_demand = -10;
    right_speed_demand = 10;
  }
}
void doMovement() {

  // Static means this variable will keep
  // its value on each call from loop()
  static unsigned long walk_update = millis();

  // used to control the forward and turn
  // speeds of the robot.
 
  float turn_bias;

    float left_distance  = LeftIR.getDistanceInMM();
  float mid_distance   = MidIR.getDistanceInMM();
  float right_distance = RightIR.getDistanceInMM();

  Serial.print("mid: ");
  Serial.println(mid_distance);
//  Serial.print("left: ");
//  Serial.println(left_distance);
//  Serial.print("right: ");
//  Serial.println(right_distance);
  
//    if( left_distance < 60 ) {
//      forward_bias = 0;
//      STATE = 2;
//    }else if( mid_distance < 100){
//      forward_bias = 0;
//      STATE = 2;
//    }else if( right_distance < 60){
//      forward_bias = 0;
//      STATE = 2;
//    }else {
//      forward_bias = 5;
//    }

  if (Pose.getX() >= 1800 or Pose.getY() >= 1800 or Pose.getX() <= 0 or Pose.getY() <= 0){
    STATE = 3;
    rotation_time = millis();
  }
//
//    if( mid_distance < 300 and  mid_distance >= 200) {
//      forward_bias = 0;
//      STATE = 2;
//      rotation_time = millis();
//    }else{
//      forward_bias = 6;
//    }

  // Check if we are about to collide.  If so,
  // zero forward speed

  // Periodically set a random turn.
  // Here, gaussian means we most often drive
  // forwards, and occasionally make a big turn.
  if( millis() - walk_update > 500 ) {
    walk_update = millis();

    // randGaussian(mean, sd).  utils.h
    turn_bias = randGaussian(0, 6.5 );

    // Setting a speed demand with these variables
    // is automatically captured by a speed PID 
    // controller in timer3 ISR. Check interrupts.h
    // for more information.
    left_speed_demand = forward_bias + turn_bias;
    right_speed_demand = forward_bias - turn_bias;
  } 

}

void doMapping() {
  
  //explored areas
  Map.updateMapFeature( (byte)'=', Pose.getY(), Pose.getX() );

  float left_distance  = LeftIR.getDistanceInMM();
  float mid_distance   = MidIR.getDistanceInMM();
  float right_distance = RightIR.getDistanceInMM();

  if ( 300 > mid_distance and mid_distance > 152){
    mid_distance += 80;
    
    // Here we calculate the actual position of the obstacle we have detected
    float projected_x = Pose.getX() + ( mid_distance * cos( Pose.getThetaRadians()  ) );
    float projected_y = Pose.getY() + ( mid_distance * sin( Pose.getThetaRadians() ) );

    Map.updateMapFeature( (byte)'O', projected_y, projected_x ); 
 
//    Map.mapBufferMid(projected_y, projected_x, FLAG);
    
  }

//  if ( 330 > left_distance and left_distance > 180){
//    left_distance += 80;
//        
//    // Here we calculate the actual position of the obstacle we have detected
//    float projected_x = Pose.getX() + ( left_distance * cos( Pose.getThetaRadians() + 0.602) );
//    float projected_y = Pose.getY() + ( left_distance * sin( Pose.getThetaRadians() + 0.602) );
//    Map.updateMapFeature( (byte)'O', projected_y, projected_x );
////    Map.mapBufferLeft(projected_y, projected_x, FLAG);
//    
//  }
  
//  if ( 330 > right_distance and right_distance > 180){
//    right_distance += 80;
//    
//    // Here we calculate the actual position of the obstacle we have detected
//    float projected_x = Pose.getX() + ( right_distance * cos( Pose.getThetaRadians() - 0.5847) );
//    float projected_y = Pose.getY() + ( right_distance * sin( Pose.getThetaRadians() - 0.5847) ); 
//    Map.updateMapFeature( (byte)'O', projected_y , projected_x );
////    Map.mapBufferRight(projected_y, projected_x, FLAG);
//  }
}
