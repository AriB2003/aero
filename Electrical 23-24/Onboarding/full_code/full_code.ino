#include <Wire.h>
#include <Encoder.h>
#include <Servo.h>

/*
Add the following to the Encoder library interrupt pin configuration file:

{DOCUMENTS}\Arduino\libraries\Encoder\utility\interrupt_pins.h

#elif defined(__AVR_ATtiny3224__)
  #define CORE_NUM_INTERRUPT	2
  #define CORE_INT0_PIN		2
  #define CORE_INT1_PIN		6
*/


// Pin definitions
#define ENC_A A2
#define ENC_B A6
#define BUTTON A1
#define PWM1 A5
#define PWM2 A4
#define PWM3 A3
#define VREAD A7

// Class definitions
Encoder myEnc(ENC_A, ENC_B);
Servo servo1;
Servo servo2;
Servo servo3;

// Servo tester state
// 0 = Servo 1
// 1 = Servo 2
// 2 = Servo 3
// 3 = All
uint8_t servo_state = 0;
String state_name = "srv1";
// 0 = Knob
// 1 = Zero
// 2 = Sweep
uint8_t knob_state = 0;

// Button state variables
uint32_t button_depressed_time = 0;
bool last_button_state = true;
uint32_t last_button_update = 0;


// Position state variables
long encoder_position = 0;
long sweep_position = 0;
bool sweep_direction = false;
long zero_position = 0;
long write_value = 0;

// Display driver configurations
uint8_t ADDR = 0x38;
uint8_t reg_counter = 4;

// Display state variables
bool update_display = true;
uint32_t last_display_update = 0;
uint8_t voltage_or_name = 0;

// Voltage read state variables
uint32_t last_voltage_update = 0;
float rail_voltage = 0;

// Servo write state variables
uint32_t last_servo_update = 0;

// Character to binary mapping
struct MapCharacterToBinary
{
  char character;
  uint16_t binary;
};

//https://en.wikipedia.org/wiki/Fourteen-segment_display
// Character mappings for 14-segment display
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
// Physical register mapping to segment position
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

// Display character buffer
char buf[16];

// Display binary buffer
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
  // Initialize I2C and configure the display driver
  // https://www.nxp.com/docs/en/data-sheet/PCF8551.pdf
  Wire.begin();
  delay(50);
  Wire.beginTransmission(ADDR);
  Wire.write(0x02); // Address 2
  Wire.write(0x01); // Toggle DE
  Wire.endTransmission();
  Wire.beginTransmission(ADDR);
  Wire.write(0x03); // Address 2
  Wire.write(0x00); // Toggle Blink 1 Hz
  Wire.endTransmission();

  // Pin configurations (all others auto configured)
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(VREAD, INPUT);
  
  // Write person name on startup
  const char person_name[] = "bobesh";
  for(uint8_t i=0; i<8; i++)
  {
    if (i<strlen(person_name))
    {
      put_character_at_segment(person_name[i], i);
    }
    else
    {
      put_character_at_segment(' ', i);
    }
  }
  for(uint8_t i=0; i<20; i++)
  {
    write_to_display();
  }
  delay(1500);
}

void loop() {
  update_encoder(); // Should be working
  drive_servo(); // Should be working
  drive_display(); // Should be working
  read_voltage(); // Should be working
  compute_state(); // Should be working
}

void compute_state()
{
  if(it_is_time(millis(),last_button_update,25))
  {
    bool button_state = !digitalRead(BUTTON);
    if(last_button_state)
    {
      // Was Depressed
      if(!button_state)
      {
        // Now Released
        if(millis()-button_depressed_time>1000)
        {
          // If held for a second you transition servo states
          increment_servo_state();
        }
        else
        {
          // If clicked quickly you transition knob states
          increment_knob_state();
        }
      }
    }
    else
    {
      // Is Released
      button_depressed_time = millis();
    }
    last_button_state = button_state;
    last_button_update = millis();
  }
}

void increment_servo_state()
{
  servo_state++;
  servo_state = servo_state % 4;
  switch(servo_state)
  {
    case 0:
      // 0 = Servo 1
      servo2.detach();
      servo3.detach();
      servo1.attach(PWM1);
      state_name = "srv1";
      break;
    case 1:
      // 1 = Servo 2
      servo1.detach();
      servo2.attach(PWM2);
      state_name = "srv2";
      break;
    case 2:
      // 2 = Servo 3
      servo2.detach();
      servo3.attach(PWM3);
      state_name = "srv3";
      break;
    case 3:
      // 3 = All
      servo1.attach(PWM1);
      servo2.attach(PWM2);
      state_name = " all";
      break;
    default:
      break;
  }

  // Display name next
  voltage_or_name = 0;
  // Go back to knob
  knob_state = 0;
  // Reset the encoder position
  myEnc.write(0);
}

void increment_knob_state()
{
  knob_state++;
  knob_state = knob_state % 3;
  
  // Reset the sweep
  sweep_direction = false;
  sweep_position = 0;
}

void drive_servo()
{
  if(it_is_time(millis(),last_servo_update,15))
  {

    // Choose which value to write
    switch(knob_state)
    {
      case 0:
        // 0 = Knob
        write_value = encoder_position;
        break;
      case 1:
        // 1 = Zero
        write_value = zero_position;
        break;
      case 2:
        // 2 = Sweep
        // Modify sweep value
        if(sweep_direction)
        {
          sweep_position++;
          if(sweep_position>=90)
          {
            sweep_direction = false;
          }
        }
        else
        {
          sweep_position--;
          if(sweep_position<=-90)
          {
            sweep_direction = true;
          }
        }
        write_value = sweep_position;
        break;
      default:
        break;
    }

    // Choose what servos to write to
    switch(servo_state)
    {
      case 0:
        // 0 = Servo 1
        servo1.write(write_value+90);
        break;
      case 1:
        // 1 = Servo 2
        servo2.write(write_value+90);
        break;
      case 2:
        // 2 = Servo 3
        servo3.write(write_value+90);
        break;
      case 3:
        // 3 = All
        servo1.write(write_value+90);
        servo2.write(write_value+90);
        servo3.write(write_value+90);
        break;
      default:
        break;
    }

    last_servo_update = millis();
  }
}

void read_voltage()
{
  if(it_is_time(millis(),last_voltage_update,100))
  {
    /*
      ADC raw value range 0-3.3V is 0-1024
      R1 = 15k, R2 = 4.7k
      VREAD = R2/(R1+R2) * VRAIL
      VRAIL = VREAD * (R1+R2)/R2
      VREAD = RAW / 1024 * 3.3
      VRAIL = RAW / 1024 * 3.3 * (R1+R2)/R2
      VRAIL = RAW / 1024 * 3.3 * (4700+15000)/4700
      VRAIL = RAW * 0.01350772938829787234042553191489
    */
    rail_voltage = analogRead(VREAD) * 0.01350772938829787234042553191489;
    last_voltage_update = millis();
  }
}

void update_encoder()
{
  // Read the encoder on every loop cycle
  encoder_position = myEnc.read();
  encoder_position = constrain(encoder_position, -90, 90);
  myEnc.write(encoder_position);
}

void drive_display()
{
  if(update_display)
  {
    // Only write to the display when update triggered
    write_to_display();
  }
  else if(it_is_time(millis(),last_display_update,30))
  {
    // Slow down display refresh rate to around 30 Hz

    // Reset display
    for(uint8_t i=0; i<8; i++)
    {
      put_character_at_segment(' ', i);
    }

    // Write the position value
    ltoa(write_value,buf,10);
    if(write_value<0)
    {
      put_character_at_segment('-', 0);
      put_character_at_segment(buf[1], 1);
      put_character_at_segment(buf[2], 2);
    }
    else
    {
      put_character_at_segment(' ', 0);
      put_character_at_segment(buf[0], 1);
      put_character_at_segment(buf[1], 2);
    }

    // Write the voltage reading or the state name
    voltage_or_name++;
    // Use bit 6 with representation 64 to slow to 2 sec
    if(voltage_or_name>>6 & 0b1)
    {
      // Write voltage
      dtostrf(rail_voltage, 4, 2, buf);
      put_character_at_segment(buf[0], 4, buf[1]);
      put_character_at_segment(buf[2], 5);
      put_character_at_segment(buf[3], 6);
      put_character_at_segment('v', 7);
    }
    else
    {
      // Write name
      state_name.toCharArray(buf, 10);
      // srv1, srv2, srv3, All
      put_character_at_segment(buf[0], 4);
      put_character_at_segment(buf[1], 5);
      put_character_at_segment(buf[2], 6);
      put_character_at_segment(buf[3], 7);
    }

    update_display = true;
    last_display_update = millis();
  }
  else
  {
    // Needed to stop instability with the encoder.
    delay(1);
  }
}

void put_character_at_segment(char character, uint8_t segment) 
{
    put_character_at_segment(character, segment, ' ');
}
void put_character_at_segment(char character, uint8_t segment, char punctuation)
{
  uint16_t binary = get_character_binary(character) | get_character_binary(punctuation);
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
    update_display = false;
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
