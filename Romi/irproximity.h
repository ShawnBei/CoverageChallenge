#ifndef _IRProximity_h
#define _IRProximity_h

class SharpIR
{
    public:
        SharpIR(byte pin);
        int  getDistanceRaw();
        float  getDistanceInMM();
        void calibrate();
        void setMax();
        

    private:
        byte pin;
        float new_max;
        
        // IR sensor variables
        float a = 0.15;
        float current_distance;
        float past_distance;
};

SharpIR::SharpIR(byte _pin)
{
  pin = _pin;
}

int SharpIR::getDistanceRaw()
{
    return analogRead(pin);
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
    
    float distance = (float) analogRead( pin );
    
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
    
    return distance;
}


#endif
