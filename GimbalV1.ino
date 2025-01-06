#include <Wire.h>

#define UART_BAUD 115200           // Set baud rate for UART
#define BNO086_ADDR 0x4A         // BNO086 I2C address
#define HW_INT A2
#define STAT A6

uint8_t seqNum = 0;
uint8_t dataPacket[100];
bool receivedNewData = false;
uint32_t print_last = 0;
uint16_t print_delay = 100;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(UART_BAUD);
  Wire.begin();
  // attachInterrupt(digitalPinToInterrupt(HW_INT), readGyro, FALLING);
  digitalWrite(STAT, HIGH);
  while(!Serial)
  {
    delay(1);
  }
  delay(100);
  Wire.beginTransmission(BNO086_ADDR);
  setHeader(0x15, 0x02); // Gyro set header
  setFeature(0x05, 0xEA60); // 60ms interval gryo set 0xEA60
  Wire.endTransmission();
  digitalWrite(STAT, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
  if(digitalRead(HW_INT)== 0)
  {
    // Serial.println("Interrupt triggered");
    readGyro();
  }
  if(it_is_time(millis(),print_last,print_delay))
  {
    if(receivedNewData)
    {
      receivedNewData = false;
      printGyroAngles();
      //printDataPacket();
    }
    else
    {
      Serial.println("No data received");
    }
    print_last = millis();
  }
}

void printGyroAngles() {
  /* //Prints sequence length + type of output (should be 23, 5) for rot vector
    //Tests if code is screwy
  Serial.print(String((dataPacket[1]<<8) | dataPacket[0]));
  Serial.print(" ");
  Serial.println(String(dataPacket[9]));*/
  /*Serial.print(String((dataPacket[14]<<8) | dataPacket[13]));
  Serial.print(", ");
  Serial.print(String((dataPacket[16]<<8) | dataPacket[15]));
  Serial.print(", ");
  Serial.print(String((dataPacket[18]<<8) | dataPacket[17]));
  Serial.print(", ");
  Serial.print(String((dataPacket[20]<<8) | dataPacket[19]));
  Serial.print(", ");
  Serial.print(String((dataPacket[22]<<8) | dataPacket[21]));
  Serial.println();*/

  float qw = ((dataPacket[14]<<8) | dataPacket[13]) / 16384.0;
  float qx = ((dataPacket[16]<<8) | dataPacket[15]) / 16384.0;
  float qy = ((dataPacket[18]<<8) | dataPacket[17]) / 16384.0;
  float qz = ((dataPacket[20]<<8) | dataPacket[19]) / 16384.0;

  float roll = atan2(2.0 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)) * (180.0 / M_PI);
  float pitch = asin(2.0 * (qw * qy - qz * qx)) * (180.0 / M_PI);
  float yaw = atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) * (180.0 / M_PI); 

  Serial.print(roll);     
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(yaw);
}

void printDataPacket()
{
  uint16_t length = (dataPacket[1]<<8) | dataPacket[0];
  for(uint16_t i=0; i<length; i++)
  {
    Serial.print(String(dataPacket[i]));
    Serial.print(", ");
  }
  Serial.println();
}

void readGyro() {
  digitalWrite(STAT, HIGH);
  Wire.requestFrom(BNO086_ADDR, 23);
  uint8_t index = 0;
  while(Wire.available()) {
    dataPacket[index] = Wire.read();
    index++;
  }
  digitalWrite(STAT, LOW);
  receivedNewData = true;
}

// setHeader(0x15, 0x02); gyro set header
void setHeader(uint16_t length, uint8_t channel) {
  Wire.write((length & 0xFF));        // Length (LSB)
  Wire.write((length >> 8) & 0xFF);  // Length (MSB)
  Wire.write(channel);                // Channel
  Wire.write(seqNum);                // Sequence Number
  seqNum++;
}

// setFeature(0x02, 0xEA60); 60ms gryo set
void setFeature(uint8_t featureReportID, uint32_t reportInterval) {
  Wire.write(0xFD);                // Set Feature Command ID
  Wire.write(featureReportID);            // Report ID (0x05 = quaternion)
  Wire.write(0x00);                // Feature flags
  Wire.write(0x00);                // Change sensitivity (LSB)
  Wire.write(0x00);                // Change sensitivity (MSB)
  Wire.write((reportInterval & 0xFF));        // Report Interval (LSB)
  Wire.write((reportInterval >> 8) & 0xFF);  // Report Interval
  Wire.write((reportInterval >> 16) & 0xFF); // Report Interval
  Wire.write((reportInterval >> 24) & 0xFF); // Report Interval (MSB)
  Wire.write(0x00);                // Batch Interval (LSB)
  Wire.write(0x00);                // Batch Interval
  Wire.write(0x00);                // Batch Interval
  Wire.write(0x00);                // Batch Interval (MSB)
  Wire.write(0x00);                // Sensor-specific Configuration Word (LSB)
  Wire.write(0x00);                // Sensor-specific Configuration Word
  Wire.write(0x00);                // Sensor-specific Configuration Word
  Wire.write(0x00);                // Sensor-specific Configuration Word (MSB)
}

/*
** Returns a boolean value that indicates whether the current time, t, is later than some prior 
** time, t0, plus a given interval, dt.  The condition accounts for timer overflow / wraparound.
*/
bool it_is_time(uint32_t t, uint32_t t0, uint16_t dt) {
  return ((t >= t0) && (t - t0 >= dt)) ||         // The first disjunct handles the normal case
            ((t < t0) && (t + (~t0) + 1 >= dt));  //   while the second handles the overflow case
}