volatile uint16_t p = 0;
//volatile last_a = true;
//volatile last_b = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){}
  pinMode(A2, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A2), enc_a, RISING);
  attachInterrupt(digitalPinToInterrupt(A6), enc_b, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(p);
  delay(100);
}

void enc_a()
{
  if(digitalRead(A6))
  {
    p--;
  }
  else
  {
    p++;
  }
  delayMicroseconds(10);
}

void enc_b()
{
  if(digitalRead(A2))
  {
    p++;
  }
  else
  {
    p--;
  }
  delayMicroseconds(10);
}
