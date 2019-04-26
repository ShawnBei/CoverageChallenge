
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
int STATE;

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

int p = 0;

unsigned long timestamp;

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

  // For this example, we'll calibrate only the 
  // centre sensor.  You may wish to use more.
  LineCentre.calibrate(); 

  //Setup RFID card
  setupRFID();

  //Calibrate the IR sensors
  float alpha = 0.15;
  LeftIR.setAlpha(alpha);
  MidIR.setAlpha(alpha);
  RightIR.setAlpha(alpha);
  
  LeftIR.calibrate();
  MidIR.calibrate();
  RightIR.calibrate();

  /*
   * SCALER
   */
  //Pose.setScaler(0.98);
  
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
  String mapType = "circular";
  Map.initMap(mapType);

  LeftSpeedControl.reset();
  RightSpeedControl.reset();
  left_speed_demand = 0;
  right_speed_demand = 0;

  timestamp = millis();
}

void loop() {

  Pose.update();
  
  unsigned long elapsed_time = millis() - timestamp;
  
  if (elapsed_time > 1000*60*3){
    STATE = TIME; // Time has elapsed
  }
  
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
    case 3: stop_romi();
    default: Serial.println("Time");
             break;
  }
}

void forward(){
  
  int condition = (count - right_encoder_count) < 0;
  //float forward_heading_output  = ForwardHeadingControl.update(dir, Pose.getThetaDegrees());

  if(condition){
    // Forward finsihed
    stop_romi();
    STATE = WALK;
  }
  else{
    int demand = 6;
    left_speed_demand = demand - forward_heading_output;
    right_speed_demand = demand + forward_heading_output;
  }
  
}

void walk(){
  getNeighborReadings();

  determineLowestNeighbor();
  
  determineTurningAngle();
  
  if(FLAG_SMALLEST == NORTH){
    // Next step forward
    STATE = FORWARD;
    count = right_encoder_count + COUNTS_PER_GRID;
  }else{
    // Next step rotate
    STATE = ROTATE;
  }
}

void rotate() {
  float theta = Pose.getThetaDegrees();
//  float output;
//  
//  if ( MINUS90 == 1 and 185 > theta and theta > 175 ){
//    output = HeadingControl.update( dir, - theta); 
//    Serial.println( "MINUS90 == 1 && Theta positive" );
//    
//  }else if( MINUS90 == 1 and -185 < theta and theta < -175 ){
//    output = HeadingControl.update( dir, - theta); 
//    Serial.println( "MINUS90 == 1 && Theta negetive" );
//    
//  }else{
//    output = HeadingControl.update( dir, theta );
//    Serial.println( "Normal Rotate" );
//    
//  }
//
//  float error = HeadingControl.getError();

  int condition = (dir - theta) < 0

  //if (error <= 1){
  if (condition){
    count = right_encoder_count + COUNTS_PER_GRID;
    STATE = FORWARD;
    
  } else {
    left_speed_demand  = -output;
    right_speed_demand = output;
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
            break;
            
    // UP UP UP UP UP UP UP UP UP UP UP UP 
    case 1: w_val  =  w;
            e_val  =  e;
            s_val  =  s;
            n_val  =  n;
            break;
            
    // <- <- <- <- <- <- <- <- <- <- <- <-
    case 2: w_val  =  s;
            e_val  =  n;
            s_val  =  e;
            n_val  =  w;
            break;
    
    // DN DN DN DN DN DN DN DN DN DN DN DN
    case 3: w_val  =  e;
            e_val  =  w;
            s_val  =  n;
            n_val  =  s;
            break;
    default:  Serial.println("ERROR");
    
  }
}

void determineLowestNeighbor(){
    char lowest_neighbor  = n_val;
    FLAG_SMALLEST = NORTH; 
    
    if( lowest_neighbor > s_val ){
      lowest_neighbor = s_val;
      FLAG_SMALLEST = SOUTH; 
    }
    
    if( lowest_neighbor > w_val){
      lowest_neighbor = w_val;
      FLAG_SMALLEST = WEST; 
    }
  
    if( lowest_neighbor > e_val){
      lowest_neighbor = e_val;
      FLAG_SMALLEST = EAST; 
    }
}

// FLAG is the heading direction of Romi
void determineTurningAngle(){
  
    MINUS90 = 0;
    FLAG_THETA = true;

    if(Pose.getThetaDegrees())
      FLAG_THETA = true;
    else
      FLAG_THETA = false;
    
     
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
        
        //determine degree is positive or negtive
        //then determine the goal 
        if(FLAG_THETA)
          dir = 180;
        else
          dir = -180;
      }
  
    //TURN 180 DEGREE - CONDITION NOT MET
    }else if(FLAG_SMALLEST == SOUTH){ //turn South/back
      
      if(FLAG == FORWARD){
        FLAG = BACK;
        dir = 180;
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
      else if(FLAG == BACK){ //determine this back is positive or negtive 
        FLAG = DOWN;
        dir = -90;
        
        if(FLAG_THETA){
          MINUS90 = 1; //back is positive, inverse current theta
        }else{
          //back is negetive, do normal rotate
        }
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
        dir = -180;
      }
      else if(FLAG == BACK){
        FLAG = UP;
        dir = 90; 

        if(FLAG_THETA){
          //back is positive, do normal rotate
        }else{    
          MINUS90 = 1; //back is negetive, inverse current theta
        }
      }
    }
}
