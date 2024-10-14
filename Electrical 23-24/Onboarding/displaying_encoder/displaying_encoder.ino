#include <Wire.h>
#include <Encoder.h>

Encoder myEnc(A2, A6);

char buf[16];

uint32_t last_encoder_update = 0;
bool encoder_updated = true;
long encoder_position = 0;

uint8_t ADDR = 0x38;
uint8_t reg_counter = 4;
//uint8_t char_counter = 7;
//char received = ' ';

struct MapCharacterToBinary
{
  char character;
  uint16_t binary;
};

//https://en.wikipedia.org/wiki/Fourteen-segment_display
#define VALID_CHAR_COUNT 40
MapCharacterToBinary ctb[VALID_CHAR_COUNT] = {
  {'a', 0x00F7},
  {'b', 0x128F},
  {'c', 0x0039},
  {'d', 0x120F},
  {'e', 0x00F9},
  {'f', 0x00F1},
  {'g', 0x00BD},
  {'h', 0x00F6},
  {'i', 0x1209},
  {'j', 0x001E},
  {'k', 0x2470},
  {'l', 0x0038},
  {'m', 0x0536},
  {'n', 0x2136},
  {'o', 0x003F},
  {'p', 0x00F3},
  {'q', 0x203F},
  {'r', 0x20F3},
  {'s', 0x018D},
  {'t', 0x1201},
  {'u', 0x003E},
  {'v', 0x0C30},
  {'w', 0x2836},
  {'x', 0x2D00},
  {'y', 0x1500},
  {'z', 0x0C09},
  {'0', 0x0C3F},
  {'1', 0x0406},
  {'2', 0x00DB},
  {'3', 0x008F},
  {'4', 0x00E6},
  {'5', 0x00ED},
  {'6', 0x00FD},
  {'7', 0x1401},
  {'8', 0x00FF},
  {'9', 0x00E7},
  {'.', 0x4000},
  {',', 0x8000},
  {'-', 0x00C0},
  {' ', 0x0000}
};

// [7:3] = register address (starting at 04h), [2:0] bit address
uint8_t bta[8][16] = {
  {0xB5, 0x8D, 0x65, 0x36, 0x5E, 0x86, 0x8E, 0x5D, 0xB6, 0xAD, 0x85, 0x66, 0x3E, 0x35, 0x3D, 0xAE},
  {0xB3, 0x8B, 0x63, 0x34, 0x5C, 0x84, 0x8C, 0x5B, 0xB4, 0xAB, 0x83, 0x64, 0x3C, 0x33, 0x3B, 0xAC},
  {0xB8, 0x90, 0x68, 0x32, 0x5A, 0x82, 0x8F, 0x59, 0xB7, 0xA9, 0x81, 0x67, 0x3F, 0x31, 0x40, 0xAA},
  {0xBA, 0x92, 0x6A, 0x30, 0x58, 0x80, 0x91, 0x57, 0xB9, 0xA7, 0x7F, 0x69, 0x41, 0x2F, 0x42, 0xA8},
  {0x98, 0x70, 0x48, 0x2E, 0x56, 0x7E, 0x93, 0x55, 0xBB, 0xA5, 0x7D, 0x6B, 0x43, 0x2D, 0x20, 0xA6},
  {0x9A, 0x72, 0x4A, 0x2C, 0x54, 0x7C, 0x71, 0x53, 0x99, 0xA3, 0x7B, 0x49, 0x21, 0x2B, 0x22, 0xA4},
  {0x9C, 0x74, 0x4C, 0x2A, 0x52, 0x7A, 0x73, 0x51, 0x9B, 0xA1, 0x79, 0x4B, 0x23, 0x29, 0x24, 0xA2},
  {0x9E, 0x76, 0x4E, 0x28, 0x50, 0x78, 0x75, 0x4F, 0x9D, 0x9F, 0x77, 0x4D, 0x25, 0x27, 0x26, 0xA0},
};


uint8_t display_memory[20] = {
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00
};

void setup() {
  Wire.begin();
//  Serial.begin(115200);
//  while(!Serial)
//  {
//    
//  }
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
  encoder_position = myEnc.read();
//  encoder_position = constrain(encoder_position, -255, 255);
//  myEnc.write(encoder_position);
  if(encoder_updated)
  {
    write_to_display();
  }
  else if(it_is_time(millis(),last_encoder_update,30))
  {
    ltoa(encoder_position,buf,10);
    for(int i=0; i<8; i++)
    {
      put_character_at_segment(buf[i], i);
    }
    encoder_updated = true;
    last_encoder_update = millis();
  }
  else
  {
    // Needed to stop instability with the encoder.
    delay(1);
  }
}

void put_character_at_segment(char character, uint8_t segment)
{
  uint16_t binary = get_character_binary(character);
  uint8_t val, address;
  for(int i=0; i<16; i++)
  {
    address = bta[segment][i];
    val = (binary>>i)&0b1;
    update_display_memory(address, val);
  }
}

uint16_t get_character_binary(char character)
{
  uint16_t binary = 0x0000;
  for(int i = 0; i<VALID_CHAR_COUNT; i++)
  {
    if(ctb[i].character == character)
    {
      binary = ctb[i].binary;
      break;
    }
  }
  return binary;
}

void update_display_memory(uint8_t address, uint8_t val)
{
  uint8_t reg = (address >> 3)-4;
  uint8_t loc = (address&0b111);
  if(val)
  {
    display_memory[reg]|= 0b1<<loc;
  }
  else
  {
    display_memory[reg]&= ~(1<<loc);
  }
}

// TOO SLOW FOR ENCODER
//void write_to_display_looped()
//{
//  for(int i=0; i<20; i++)
//  {
//    write_to_display();
//  }
//}

void write_to_display()
{
  Wire.beginTransmission(ADDR);
  Wire.write(reg_counter);
  Wire.write(display_memory[reg_counter-4]); // Set all bits high
  Wire.endTransmission();
  reg_counter++;
  if(reg_counter>23)
  {
    reg_counter = 4;
    encoder_updated = false;
  }
}

/*
** Returns a boolean value that indicates whether the current time, t, is later than some prior 
** time, t0, plus a given interval, dt.  The condition accounts for timer overflow / wraparound.
*/
bool it_is_time(uint32_t t, uint32_t t0, uint16_t dt) {
  return ((t >= t0) && (t - t0 >= dt)) ||         // The first disjunct handles the normal case
            ((t < t0) && (t + (~t0) + 1 >= dt));  //   while the second handles the overflow case
}
