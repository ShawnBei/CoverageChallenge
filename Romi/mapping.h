#ifndef _Mapping_h
#define _Mapping_h
#include <EEPROM.h>

const byte MAP_RESOLUTION = 27;
const byte MAP_DEFAULT_FEATURE = '@';
const int MAP_X=1944;
const int MAP_Y=1944;
const float GRID_DIST = 72;

class Mapper
{
    public:
        void resetMap();
        void printMap();
        void initMap(String mapType);
        void printMetrics();
        
        void mapBufferCircle(float y, float x);
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
void Mapper::initMap(String mapType)
{
    Serial.println("<Begin init>");
    

    if (mapType == "circular"){
      char c= '<';
      for (int i=1; i < 13; i++)
      {
          int m = i + 1;
          int n = 26 - i;
          for(; m < n; m++)
          { 
              int top = (i*MAP_RESOLUTION)+m;
              int bot = (n*MAP_RESOLUTION)+m;
  
              EEPROM.update(top, (byte)c);
              EEPROM.update(bot, (byte)c);
          }
          
          int o = i;
          int p = 27 - i;
          for(; o < p; o++)
          { 
              int left = (o*MAP_RESOLUTION)+i;
              int right = (o*MAP_RESOLUTION)+n;
  
              EEPROM.update(left, (byte)c);
              EEPROM.update(right, (byte)c);
              
          }
          c--; //change character
      }
    }else if(mapType == "BF"){
      char c = '$';
      for (int i=2; i < 25; i++)
      {
          for(int j=2; j < 25; j++)
          {
              int pos = (j*MAP_RESOLUTION)+i;
              EEPROM.update(pos, (byte)c);
            
              
          }
          c++; //change character
      }

      
    }else if(mapType == "inverse-circular"){
      char c= '0';
       for (int i=2; i < 12; i++)
      {
          int m = i + 1;
          int n = 26 - i;
          for(; m < n; m++)
          { 
              int top = (i*MAP_RESOLUTION)+m;
              int bot = (n*MAP_RESOLUTION)+m;
  
              EEPROM.update(top, (byte)c);
              EEPROM.update(bot, (byte)c);
          }
          
          int o = i;
          int p = 27 - i;
          for(; o < p; o++)
          { 
              int left = (o*MAP_RESOLUTION)+i;
              int right = (o*MAP_RESOLUTION)+n;
  
              EEPROM.update(left, (byte)c);
              EEPROM.update(right, (byte)c);
              
          }
          c++; //change character
      }
      EEPROM.update((13*MAP_RESOLUTION)+13, (byte)c);
    }else{
      Serial.println("Please input a valid map type");
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

void Mapper::printMetrics()
{

    Serial.println("Metrics");
    float buffers = 0;
    float obstacle = 0;
    float coverage = 0;
    float uncovered = 0;

    float grids = 625;
    
    for (int i=1;i<MAP_RESOLUTION-1;i++)
    {
        for(int j=1;j<MAP_RESOLUTION-1;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//
            char c = (char) value;
            if (c == 'O'){
              obstacle++;
            }else if(c == '?'){
              buffers++;
            }else if(c == '='){
              coverage++;
            }else{
              uncovered++;
            }
        }
    }
    Serial.print("Coverage: ");
    Serial.print(coverage);
    Serial.print(", ");
    Serial.println((float) (coverage / grids * 100) );

    Serial.print("Obstacles: ");
    Serial.print(obstacle);
    Serial.print(", ");
    Serial.println((float) (obstacle / grids * 100));

    Serial.print("Buffers: ");
    Serial.print(buffers);
    Serial.print(", ");
    Serial.println((float) (buffers / grids * 100));

    Serial.print("Uncovered: ");
    Serial.print(uncovered);
    Serial.print(", ");
    Serial.println((float) (uncovered / grids * 100));


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

void Mapper::mapBufferCircle(float y, float x){
  
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
    updateMapFeature((byte)'?', y + GRID_DIST, x - GRID_DIST);
    
  }else if (FLAG == 1 ){
    updateMapFeature((byte)'?', y - GRID_DIST, x            );
    updateMapFeature((byte)'?', y - GRID_DIST, x - GRID_DIST);
  }else if (FLAG == 3){
    updateMapFeature((byte)'?', y + GRID_DIST, x + GRID_DIST);
    updateMapFeature((byte)'?', y + GRID_DIST, x            );
  }else if (FLAG == 2){
    updateMapFeature((byte)'?', y            , x + GRID_DIST);
    updateMapFeature((byte)'?', y - GRID_DIST, x + GRID_DIST);
  }
}

void Mapper::mapBufferLeft(float y, float x, int FLAG){
  if (FLAG == 0){
    updateMapFeature((byte)'?', y            , x - GRID_DIST);
    updateMapFeature((byte)'?', y - GRID_DIST, x - GRID_DIST);
  }else if (FLAG == 1 ){
    updateMapFeature((byte)'?', y - GRID_DIST, x            );
    updateMapFeature((byte)'?', y - GRID_DIST, x + GRID_DIST);
  }else if (FLAG == 3){
    updateMapFeature((byte)'?', y + GRID_DIST, x - GRID_DIST);
    updateMapFeature((byte)'?', y + GRID_DIST, x            );
  }else if (FLAG == 2){
    updateMapFeature((byte)'?', y            , x - GRID_DIST);
    updateMapFeature((byte)'?', y + GRID_DIST, x - GRID_DIST);
  }
}

void Mapper::mapBufferMid(float y, float x, int FLAG){
  if (FLAG == 0){
    Serial.print("Buffer_x: ");
    Serial.print(x - GRID_DIST);
    Serial.print(", Buffer_y: ");
    Serial.println(y);
    updateMapFeature((byte)'?', y            , x - GRID_DIST);
  }else if (FLAG == 1){
    Serial.print("Buffer_x: ");
    Serial.print(x);
    Serial.print(", Buffer_y: ");
    Serial.println(y - GRID_DIST);
    updateMapFeature((byte)'?', y - GRID_DIST, x            );
  }else if (FLAG == 3){
    Serial.print("Buffer_x: ");
    Serial.print(x);
    Serial.print(", Buffer_y: ");
    Serial.println(y + GRID_DIST);
    updateMapFeature((byte)'?', y + GRID_DIST, x            );
  }else if (FLAG == 2){
    Serial.print("Buffer_x: ");
    Serial.print(x + GRID_DIST);
    Serial.print(", Buffer_y: ");
    Serial.println(y);
    updateMapFeature((byte)'?', y            , x + GRID_DIST);
  }
}



#endif
