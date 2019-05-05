#ifndef _IRProximity_h
#define _IRProximity_h

class SharpIR
{
    public:
        SharpIR(byte pin);
        int  getDistanceRaw();
        float  getDistanceInMM();
        int calibrate();
        void setMax();
        void setAlpha(float alpha);
        

    private:
        byte pin;
        float new_max;
        
        // IR sensor variables
        float a = 1;
        float current_distance;
        float past_distance;
        float calibrated_reading;
};

SharpIR::SharpIR(byte _pin)
{
  pin = _pin;
}

int SharpIR::getDistanceRaw()
{
    return analogRead(pin);
}

//does it make sense to calibrate this way???
//int SharpIR::calibrate()
//{
//  int sum;
//  for(int i = 0; i < 100; i++){
//    sum += getDistanceRaw();
//  }
//  calibrated_reading = sum / 100;
//
//  return 0;
//}

int SharpIR::calibrate()
{
  float dummy;
  for(int i = 0; i < 50; i++){
    dummy = getDistanceInMM();
  }
  return 0;
}
/*
 * This piece of code is quite crucial to mapping
 * obstacle distance accurately, so you are encouraged
 * to calibrate your own sensor by following the labsheet.
 * Also remember to make sure your sensor is fixed to your
 * Romi firmly and has a clear line of sight!
 */
float SharpIR::getDistanceInMM()
{
    
    float distance = (float) getDistanceRaw();
    
    // map this to 0 :x 5v range.
    //distance *= 0.0048;

    const float exponent = (1/-0.643);
    current_distance = pow( ( distance / 2538 ), exponent);
//    current_distance = 183959 * pow(distance, -1.537);

    //Filtering 
    current_distance = (a * current_distance) + ((1.0 - a) * past_distance);
    past_distance = current_distance;
    
    past_distance = constrain(past_distance, 0, 100);
    distance = constrain(current_distance, 0, 100);

    // Convert to MM by multiplying with 10
    return distance * 10;
}


void SharpIR::setAlpha(float alpha)
{
    a = alpha;
}

#endif
