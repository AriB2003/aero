#include <Wire.h>
//#include <Servo.h>
uint8_t ADDR = 0x38;
uint8_t counter = 4;
uint8_t val = 0;
uint8_t inc = 0;
byte segments[20] = 
{
18,
52,
13,
0,
0,
0,
100,
119,
40,
5,
1,
68,
81,
232,
13,
5,
8,
0,
48,
0
};

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while(!Serial)
  {
    
  }
  delay(1000);
  Wire.beginTransmission(ADDR);
  Wire.write(0x02); // Address 2
  Wire.write(0x01); // Toggle DE
  Wire.endTransmission();
  Wire.beginTransmission(ADDR);
  Wire.write(0x03); // Address 2
  Wire.write(0x00); // Toggle Blink 1 Hz
  Wire.endTransmission();
}

void loop() {
  Wire.beginTransmission(ADDR);
  Wire.write(counter);
  if(inc)
  {
    Wire.write(segments[counter-4]); // Set all bits high
  }
  else
  {
//    Wire.write(0x00); // Set all bits low
  }
  Wire.endTransmission();
  delay(10);
  counter++;
  if(counter>23)
  {
    counter = 4;
    inc = ~inc;
  }

  // Read and print
//  counter = 0;
//  Wire.beginTransmission(0x38);
//  Wire.write(0x00);
//  Wire.endTransmission();
////  Wire.beginTransmission(113);
////  for(uint8_t i=0; i<24; i++)
////  {
////    Serial.print(counter);
////    Serial.print(",");
////    Serial.println(Wire.read());
////    counter++;
////  }
////  Wire.endTransmission();
//  for(uint8_t i=0; i<24; i++)
//  {
//    Serial.print(counter);
//    Serial.print(",");
//    Wire.requestFrom(0x38, 1);
//    val = Wire.read();
//    Serial.println(val);
//    counter++;
//  }
//  delay(1);
}
