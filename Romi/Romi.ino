
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Library Includes.                                                             *
 * Be sure to check each of these to see what variables/functions are made        *
 * global and accessible.                                                        *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "pins.h"
#include "utils.h"
#include "motors.h"
#include "pid.h"
#include "interrupts.h"
#include "kinematics.h"
#include "line_sensors.h"
#include "irproximity.h"
#include "mapping.h"
#include "RF_Interface.h"
#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"
#include "Pushbutton.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Definitions.  Other definitions exist in the .h files above.                  *
 * Also ensure you check pins.h for pin/device definitions.                      *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BAUD_RATE 9600



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class Instances.                                                              *
 * This list is complete for all devices supported in this code.                 *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Kinematics    Pose; //Kinematics class to store position and heading

LineSensor    LineLeft(LINE_LEFT_PIN); //Left line sensor
LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor
LineSensor    LineRight(LINE_RIGHT_PIN); //Right line sensor

#define SHARP_IR_PIN A0 //Pin for the IR Distance sensor
#define SHARP_IR_PIN_RIGHT A3 //Pin for the IR Distance sensor
#define SHARP_IR_PIN_LEFT A4
 //Pin for the IR Distance sensor

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
PID           HeadingControl( 1, 0, 0.001 );
PID           ForwardHeadingControl( 3, 0, 0 );


//PID           RightSpeedControl( 3.5, 20.9, 0.04 );
Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  
 * Global variables.                                                             *
 * These global variables are not mandatory, but are used for the example loop() *
 * routine below.                                                                *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define FORWARD 0
#define WALK    1
#define ROTATE  2

#define UP      1
#define DOWN    3
#define BACK    2

#define NORTH   0
#define SOUTH   1
#define WEST    2
#define EAST    3

//Use these variables to set the demand of the speed controller
bool use_speed_controller = true;
float left_speed_demand = 0;
float right_speed_demand = 0;

// State machine
int STATE;

// Going forward
int COUNTS_PER_GRID = COUNTS_PER_MM * GRID_DIST;
long count = COUNTS_PER_GRID;

// Determine direction
int FLAG = FORWARD;
float dir;
int FLAG_SMALLEST;
char lowest_neighbor;

// Going from 180 to -90
int MINUS90;

// For reading the surrounding map
char n_val;
char s_val;
char e_val;
char w_val;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * This setup() routine initialises all class instances above and peripherals.   *
 * It is recommended:                                                            *
 * - You keep this sequence of setup calls if you are to use all the devices.    *
 * - Comment out those you will not use.                                         *
 * - Insert new setup code after the below sequence.                             *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
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

  HeadingControl.setMax(7);

  // For this example, we'll calibrate only the 
  // centre sensor.  You may wish to use more.
  LineCentre.calibrate(); 

  //Setup RFID card
  setupRFID();

  //Calibrate the IR sensors
  float alpha = 0.24;
  LeftIR.setAlpha(alpha);
  MidIR.setAlpha(alpha);
  RightIR.setAlpha(alpha);
  
  LeftIR.calibrate();
  MidIR.calibrate();
  RightIR.calibrate();
  
  
  // These functions calibrate the IMU and Magnetometer
  // The magnetometer calibration routine require you to move
  // your robot around  in space.  
  // The IMU calibration requires the Romi does not move.
  // See related lab sheets for more information.
  /*
  Wire.begin();
  Mag.init();
  Mag.calibrate();
  Imu.init();
  Imu.calibrate();
  */

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

  // Watch for second button press, then begin autonomous mode.
  ButtonB.waitForButton();  

  Map.resetMap();
  Serial.println("Map Erased - Mapping Started");

  //Draw map
  Map.initMap();

  LeftSpeedControl.reset();
  RightSpeedControl.reset();
  left_speed_demand = 0;
  right_speed_demand = 0;
  
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * This loop() demonstrates all devices being used in a basic sequence.          
 * The Romi should:                                                                              
 * - move forwards with random turns 
 * - log lines, RFID and obstacles to the map.
 * 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void loop() {

  // Remember to always update kinematics!!
  Pose.update();

  //doMovement();
  doMapping();
  
  wavefront();
  
  delay(2);
}

void wavefront(){
  switch(STATE){
    case 0: forward();
            break;
    case 1: walk();
            break;
    case 2: rotate();
            break;
    default: Serial.println("r");
             break;
  }
}

void forward(){
  
  int condition = (count - right_encoder_count) < 0;
  float forward_heading_output  = ForwardHeadingControl.update(dir, Pose.getThetaDegrees());

  if(condition){
    stop_speed();
    STATE = WALK;
    
  }
  else{
    int demand = 12;
    left_speed_demand = demand - forward_heading_output;
    right_speed_demand = demand + forward_heading_output;
    
  }
  Serial.print("Direction: ");
  Serial.print( dir );
  Serial.print(", Degree: ");
  Serial.print(Pose.getThetaDegrees() );
  Serial.print(", Heading output:  ");
  Serial.print( forward_heading_output );
  Serial.print(", left speed: ");
  Serial.print( left_speed_demand );
  Serial.print(", right speed: ");
  Serial.print( right_speed_demand );
  Serial.print(", LEFT ENCODER: ");
  Serial.print( left_encoder_count );
  Serial.print(", RIGHT ENCODER: ");
  Serial.print( right_encoder_count );
  Serial.print(", count condition: ");
  Serial.println( count );
  
}

void walk(){
  getNeighborReadings();

  determineLowestNeighbor();
  
  determineTurningAngle();
  
  if(FLAG_SMALLEST == NORTH){
    Serial.println("FORWARD");
    STATE = FORWARD;
    count = right_encoder_count + COUNTS_PER_GRID;
  }else{
    // ROTATE
    Serial.println("ROTATE");
    STATE = ROTATE;
  }

  Serial.print(", FLAG_SMALLEST:  ");
  Serial.print( FLAG_SMALLEST );
  Serial.print(", FLAG: ");
  Serial.print( FLAG );
  Serial.print(", DIR: ");
  Serial.print( dir  );
  Serial.print("   ");
  Serial.print(", North: ");
  Serial.print( n_val );
  Serial.print(", South: ");
  Serial.print( s_val );
  Serial.print(", West: ");
  Serial.print( w_val  );
  Serial.print(", East: ");
  Serial.println( e_val );
}

void getNeighborReadings(){
  
  int x_index = Map.poseToIndex(Pose.getX(), MAP_X, MAP_RESOLUTION);
  int y_index = Map.poseToIndex(Pose.getY(), MAP_Y, MAP_RESOLUTION);

  char n  = (char) EEPROM.read( (x_index * MAP_RESOLUTION)       + (y_index + 1) );
  char s  = (char) EEPROM.read( (x_index * MAP_RESOLUTION)       + (y_index - 1) );
  char w  = (char) EEPROM.read( ((x_index - 1) * MAP_RESOLUTION) + y_index ); 
  char e  = (char) EEPROM.read( ((x_index + 1) * MAP_RESOLUTION) + y_index ); 

  switch (FLAG){
    // -> -> -> -> -> -> -> -> -> -> -> -> 
    case 0: w_val  =  n;
            e_val  =  s;
            s_val  =  w;
            n_val  =  e;
            Serial.println("forward");
            break;
            
    // UP UP UP UP UP UP UP UP UP UP UP UP 
    case 1: w_val  =  w;
            e_val  =  e;
            s_val  =  s;
            n_val  =  n;
            Serial.println("up");
            break;
            
    // <- <- <- <- <- <- <- <- <- <- <- <-
    case 2: w_val  =  s;
            e_val  =  n;
            s_val  =  e;
            n_val  =  w;
            Serial.println("back");
            break;
    
    // DN DN DN DN DN DN DN DN DN DN DN DN
    case 3: w_val  =  e;
            e_val  =  w;
            s_val  =  n;
            n_val  =  s;
            Serial.println("down");
            break;
    default:  Serial.println("ERROR");
    
  }
}

void determineLowestNeighbor(){
    lowest_neighbor  = n_val;
    FLAG_SMALLEST = NORTH; 
    
    if( lowest_neighbor > s_val ){
      Serial.println("s");
      lowest_neighbor = s_val;
      FLAG_SMALLEST = SOUTH; 
    }
    
    if( lowest_neighbor > w_val){
      Serial.println("w");
      lowest_neighbor = w_val;
      FLAG_SMALLEST = WEST; 
    }
  
    if( lowest_neighbor > e_val){
      Serial.println("e");
      lowest_neighbor = e_val;
      FLAG_SMALLEST = EAST; 
    }
}
/*
 * FLAG is the heading direction of Romi
 * 
 */
void determineTurningAngle(){
    MINUS90 = 0;
    if (FLAG_SMALLEST == NORTH){ //turn North/go forward
  
      if(FLAG == FORWARD){
        dir = 0;
      }
      else if(FLAG == UP){
        dir = 90;
      }
      else if(FLAG == DOWN){
        dir = -90;
      }
      else if(FLAG == BACK){
        dir = 180;
        //BUFFER
      }
  
    //TURN 180 DEGREE - CONDITION NOT MET
    }else if(FLAG_SMALLEST == SOUTH){ //turn South/back
      
      if(FLAG == FORWARD){
        FLAG = BACK;
        dir = 180;
        //BUFFER
      }
      else if(FLAG == UP){
        FLAG = DOWN;
        dir = -90;
      }
      else if(FLAG == DOWN){
        FLAG = UP;
        dir = 90;
      }
      else if(FLAG == BACK){
        FLAG = FORWARD;
        dir = 0;
      }
  
    }else if(FLAG_SMALLEST == WEST){ //turn West/left
  
      if(FLAG == FORWARD){
        FLAG = UP;
        dir = 90;
      }
      else if(FLAG == UP){
        FLAG = BACK;
        dir = 180;
      }
      else if(FLAG == DOWN){
        FLAG = FORWARD;
        dir = 0;
      }
      else if(FLAG == BACK){
        FLAG = DOWN;
        dir = -90;
        MINUS90 = 1;
      }
      
    }else if(FLAG_SMALLEST == EAST){ //turn East/right
  
      if(FLAG == FORWARD){
        FLAG = DOWN;
        dir = -90;
      }
      else if(FLAG == UP){
        FLAG = FORWARD;
        dir = 0;
      }
      else if(FLAG == DOWN){
        FLAG = BACK;
        dir = 180;
      }
      else if(FLAG == BACK){
        FLAG = UP;
        dir = 90; 
      }
    }
}

void rotate() {
  float theta = Pose.getThetaDegrees();
  float output;
  
  if (MINUS90 == 1 and 185 > theta and theta > 175 ){
    output = HeadingControl.update( dir, - theta); 
  }else{
    output = HeadingControl.update( dir, theta );
  }
  
  float error = HeadingControl.getError();

  if (error < 2){
    Pose.update();
    Serial.print(", right counts: ");
    Serial.print(right_encoder_count);
    Serial.print(", left counts: ");
    Serial.println(left_encoder_count);
    //stop_speed();
    //delay(100);
    Serial.print(", afterright counts: ");
    Serial.print(right_encoder_count);
    Serial.print(", afterleft counts: ");
    Serial.println(left_encoder_count);
    
    count = right_encoder_count + COUNTS_PER_GRID;
    STATE = FORWARD;
    
  } else {

    left_speed_demand  = -output;
    right_speed_demand = output;
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * We have implemented a random walk behaviour for you
 * with a *very* basic obstacle avoidance behaviour.  
 * It is enough to get the Romi to drive around.  We 
 * expect that in your first week, should should get a
 * better obstacle avoidance behaviour implemented for
 * your Experiment Day 1 baseline test.  
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//void doMovement() {
//
//  // Static means this variable will keep
//  // its value on each call from loop()
//  static unsigned long walk_update = millis();
//
//  // used to control the forward and turn
//  // speeds of the robot.
//  float forward_bias;
//  float turn_bias;
//
//  // Check if we are about to collide.  If so,
//  // zero forward speed
//  if( MidIR.getDistanceRaw() > 450 ) {
//    forward_bias = 0;
//  } else {
//    forward_bias = 5;
//  }
//
//  // Periodically set a random turn.
//  // Here, gaussian means we most often drive
//  // forwards, and occasionally make a big turn.
//  if( millis() - walk_update > 500 ) {
//    walk_update = millis();
//
//    // randGaussian(mean, sd).  utils.h
//    turn_bias = randGaussian(0, 6.5 );
//
//    // Setting a speed demand with these variables
//    // is automatically captured by a speed PID 
//    // controller in timer3 ISR. Check interrupts.h
//    // for more information.
//    left_speed_demand = forward_bias + turn_bias;
//    right_speed_demand = forward_bias - turn_bias;
//  } 
//
//}

void doMapping() {
  
  //explored areas
  Map.updateMapFeature( (byte)'=', Pose.getY(), Pose.getX() );


  float left_distance  = 0; //LeftIR.getDistanceInMM();
  float mid_distance   = 0; //MidIR.getDistanceInMM();
  float right_distance = 0; //RightIR.getDistanceInMM();

  Serial.print("Left: ");
  Serial.print( left_distance );
  Serial.print(", Mid ");
  Serial.print(mid_distance);
  Serial.print(", Right ");
  Serial.println(right_distance);

  if ( 300 > mid_distance and mid_distance > 152){
    mid_distance += 80;
    
    // Here we calculate the actual position of the obstacle we have detected
    float projected_x = Pose.getX() + ( mid_distance * cos( Pose.getThetaRadians()  ) );
    float projected_y = Pose.getY() + ( mid_distance * sin( Pose.getThetaRadians() ) );
    Serial.print("Projected_x ");
    Serial.print(projected_x);
    Serial.print(", projected_y");
    Serial.print(projected_y);
    Map.updateMapFeature( (byte)'O', projected_y, projected_x ); 
    Map.mapBufferMid(projected_y, projected_x, FLAG);
  }

  if ( 330 > left_distance and left_distance > 180){
    left_distance += 80;
    
    // Here we calculate the actual position of the obstacle we have detected
    float projected_x = Pose.getX() + ( left_distance * cos( Pose.getThetaRadians() + 0.602) );
    float projected_y = Pose.getY() + ( left_distance * sin( Pose.getThetaRadians() + 0.602) );
    Map.updateMapFeature( (byte)'O', projected_y, projected_x );
    Map.mapBufferLeft(projected_y, projected_x, FLAG);
    
  }
  if ( 330 > right_distance and right_distance > 180){
    right_distance += 80;
    
    // Here we calculate the actual position of the obstacle we have detected
    float projected_x = Pose.getX() + ( right_distance * cos( Pose.getThetaRadians() - 0.5847) );
    float projected_y = Pose.getY() + ( right_distance * sin( Pose.getThetaRadians() - 0.5847) ); 
    Map.updateMapFeature( (byte)'O', projected_y , projected_x );
    Map.mapBufferRight(projected_y, projected_x, FLAG);
  }
  Map.printMap();
}
