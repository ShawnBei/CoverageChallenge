#ifndef _Kinematics
#define _Kinematics_h


#define GEAR_RATIO 120.0
#define COUNTS_PER_SHAFT_REVOLUTION 12.0
const float WHEEL_RADIUS = 35.0;
const float WHEEL_SEPARATION = 139;
const float COUNTS_PER_WHEEL_REVOLUTION = GEAR_RATIO * COUNTS_PER_SHAFT_REVOLUTION;
const float MM_PER_COUNT = ( 2 * WHEEL_RADIUS * PI ) / COUNTS_PER_WHEEL_REVOLUTION;
const float COUNTS_PER_MM = 6.55;
const float COUNTS_PER_DEGREE = 16;


class Kinematics
{
     public:

         void  update();
         float getThetaDegrees();
         float getThetaRadians();
         float getX();
         float getY();
         void  resetPose();
         void  setPose(float X, float Y, float theta);
         void  printPose();
         void  setDebug(bool state);
         float getDistanceFromOrigin();
         float getAngularVelocity();
         float rotationTheta;
         void updateTheta();
         void setScaler(float value);
         
         
        
    private:

         float x=900;
         float y=900;
         float theta=0;
         float last_theta = 0;
         float angular_velocity = 0;
         long  last_left_encoder_count = 0;
         long  last_right_encoder_count = 0;
         bool  debug=false;
         unsigned long last_update = 0;
         float scale = 1;

};


void Kinematics::update()
{
    //Calculate delta since last update
    float left_delta = (left_encoder_count - last_left_encoder_count)*MM_PER_COUNT;
    float right_delta = (right_encoder_count - last_right_encoder_count)*MM_PER_COUNT;
    float mean_delta = (left_delta + right_delta) / 2;  
    
    //Store counts
    last_left_encoder_count = left_encoder_count;
    last_right_encoder_count = right_encoder_count;  

    //Update position
    x+= mean_delta * cos(theta);
    y+= mean_delta * sin(theta);
    theta -=  (left_delta-right_delta) / (WHEEL_SEPARATION) * scale ;  

    rotationTheta  = theta;
    
    float time_elapsed = millis() - last_update;
    last_update = millis();

    angular_velocity = ( (left_delta-right_delta) / WHEEL_SEPARATION );
    angular_velocity -= last_theta;
    angular_velocity /= time_elapsed;
    

    if (theta > PI) //buffer 175 --> 185 degrees
    {
        theta -=2*PI;
    }
    else if(theta < -PI) //buffer -175 --> -185 degrees
    {
        theta += 2*PI;
    } 

    last_theta = theta;

    if (debug)
    {
        printPose();
    }
  
}


float Kinematics::getThetaDegrees()
{
    return rad2deg(theta);
}
float Kinematics::getThetaRadians()
{
    return (theta);
}
void Kinematics::updateTheta()
{
    left_encoder_count += 16*4;
    right_encoder_count -= 16*4;
}

float Kinematics::getAngularVelocity() {
    return rad2deg( angular_velocity );
}


float Kinematics::getX()
{
    return x;
}


float Kinematics::getY()
{
    return y;
}


void Kinematics::resetPose()
{

    x = 0;
    y = 0;
    theta = 0;

}


void Kinematics::setPose(float newX, float newY, float newTheta)
{

    x = newX;
    y = newY;
    theta = newTheta;

}

void Kinematics::printPose()
{

    Serial.print(F("X: "));
    Serial.print(x);
    Serial.print(F(" Y: "));
    Serial.print(y);
    Serial.print(F(" H: "));
    Serial.println(rad2deg(theta));

}


void Kinematics::setDebug(bool state)
{
    debug = state;
}

void Kinematics::setScaler(float value){
  scale = value;
}


float Kinematics::getDistanceFromOrigin()
{
    return sqrt(x*x + y*y);
}

#endif
