#ifndef _Mapping_h
#define _Mapping_h
#include <EEPROM.h>

const byte MAP_RESOLUTION = 25;
const byte MAP_DEFAULT_FEATURE = '#';
const int MAP_X=1800;
const int MAP_Y=1800;
const float GRID_DIST = 72;

class Mapper
{
    public:
        void resetMap();
        void printMap();
        void initMap();
        
        void mapBuffer(float y, float x);
        void mapBufferRight(float y, float x, int FLAG);
        void mapBufferMid(float y, float x, int FLAG);
        void mapBufferLeft(float y, float x, int FLAG);
        
        void updateMapFeature(byte feature, int y, int x);
        void updateMapFeature(byte feature, float y, float x);
        
        int  indexToPose(int i, int map_size, int resolution);
        int  poseToIndex(int x, int map_size, int resolution);

        
    
    private:
        int X_size;
        int Y_size;
};

void Mapper::resetMap()
{

    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for (int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            
            if (eeprom_address > 1023)
            {
                Serial.println(F("Error: EEPROM Address greater than 1023"));
            }
            else
            {
                EEPROM.update(eeprom_address, MAP_DEFAULT_FEATURE );
                
            }
        }
    }

}

// : = 10
// ; = 11
// < = 12
void Mapper::initMap()
{
    Serial.println("<Begin init>");
    char c= '<';
    
    for (int i=0; i < 12; i++)
    {
        int m = i + 1;
        int n = 24 - i;
        for(; m < n; m++)
        { 
            int top = (i*MAP_RESOLUTION)+m;
            int bot = (n*MAP_RESOLUTION)+m;

            EEPROM.update(top, (byte)c);
            EEPROM.update(bot, (byte)c);
            
        }
        
        int o = i;
        int p = 25 - i;
        for(; o < p; o++)
        { 
            int left = (o*MAP_RESOLUTION)+i;
            int right = (o*MAP_RESOLUTION)+n;

            EEPROM.update(left, (byte)c);
            EEPROM.update(right, (byte)c);
            
        }
        c--; //change character
    }
}

void Mapper::printMap()
{

    Serial.println("Map");
    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            Serial.print( (char)value );
            Serial.print(" ");
        }
        Serial.println("");
    }
  
}

int Mapper::poseToIndex(int x, int map_size, int resolution)
{
    return x / (map_size / resolution);
}

int Mapper::indexToPose(int i, int map_size, int resolution)
{
    return i* (map_size / resolution);
}


void Mapper::updateMapFeature(byte feature, float y, float x) {
  updateMapFeature( feature, (int)y, (int)x );  
}

void Mapper::updateMapFeature(byte feature, int y, int x)
{
    if (x > MAP_X || x < 0 || y > MAP_Y || y < 0)
    {
      Serial.println(F("Error:Invalid co-ordinate"));
      return;
    }

    int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
    int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);  

    int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;  

    if (eeprom_address > 1023)
    {
        Serial.println(F("Error: EEPROM Address greater than 1023"));
    }
    else
    {
        EEPROM.update(eeprom_address, feature);
    }
}

void Mapper::mapBuffer(float y, float x){
  
  updateMapFeature((byte)'?', y + GRID_DIST, x + GRID_DIST);
  updateMapFeature((byte)'?', y + GRID_DIST, x);
  updateMapFeature((byte)'?', y, x + GRID_DIST);
  
  updateMapFeature((byte)'?', y - GRID_DIST, x - GRID_DIST);
  updateMapFeature((byte)'?', y - GRID_DIST, x);
  updateMapFeature((byte)'?', y, x - GRID_DIST);
  
  updateMapFeature((byte)'?', y - GRID_DIST, x + GRID_DIST);
  updateMapFeature((byte)'?', y + GRID_DIST, x - GRID_DIST);


}
//#define FORWARD 0
//#define UP      1
//#define DOWN    3
//#define BACK    2

void Mapper::mapBufferRight(float y, float x, int FLAG){
  if (FLAG == 0){
    updateMapFeature((byte)'?', y            , x - GRID_DIST);
    updateMapFeature((byte)'?', y - GRID_DIST, x - GRID_DIST);
    
  }else if (FLAG == 1 ){
    updateMapFeature((byte)'?', y + GRID_DIST, x            );
    updateMapFeature((byte)'?', y + GRID_DIST, x - GRID_DIST);
  }else if (FLAG == 3){
    updateMapFeature((byte)'?', y - GRID_DIST, x + GRID_DIST);
    updateMapFeature((byte)'?', y - GRID_DIST, x            );
  }else if (FLAG == 2){
    updateMapFeature((byte)'?', y            , x + GRID_DIST);
    updateMapFeature((byte)'?', y - GRID_DIST, x + GRID_DIST);
  }
}

void Mapper::mapBufferLeft(float y, float x, int FLAG){
  if (FLAG == 0){
    updateMapFeature((byte)'?', y            , x - GRID_DIST);
    updateMapFeature((byte)'?', y + GRID_DIST, x - GRID_DIST);
  }else if (FLAG == 1 ){
    updateMapFeature((byte)'?', y + GRID_DIST, x            );
    updateMapFeature((byte)'?', y + GRID_DIST, x + GRID_DIST);
  }else if (FLAG == 3){
    updateMapFeature((byte)'?', y - GRID_DIST, x - GRID_DIST);
    updateMapFeature((byte)'?', y - GRID_DIST, x            );
  }else if (FLAG == 2){
    updateMapFeature((byte)'?', y            , x + GRID_DIST);
    updateMapFeature((byte)'?', y + GRID_DIST, x + GRID_DIST);
  }
}

void Mapper::mapBufferMid(float y, float x, int FLAG){
  if (FLAG == 0){
    Serial.print("Buffer x: ");
    Serial.print(x);
    Serial.print(", Buffer y");
    Serial.println(y + GRID_DIST);
    updateMapFeature((byte)'?', y            , x - GRID_DIST);
  }else if (FLAG == 1){
//    Serial.print("Buffer x: ");
//    Serial.print(x);
//    Serial.print(", Buffer y");
//    Serial.println(y + GRID_DIST);
    updateMapFeature((byte)'?', y + GRID_DIST, x            );
  }else if (FLAG == 3){
    updateMapFeature((byte)'?', y - GRID_DIST, x            );
  }else if (FLAG == 2){
    updateMapFeature((byte)'?', y            , x + GRID_DIST);
  }
}



#endif
