#include <AK10_9.h>

AK10_9 Motor(0x01);
static int limitL = 46;
static int limitR = 32;


void setup() {
  Serial.begin(115200);
  while(!Serial); // wait for Serial
  // put your setup code here, to run once:
  delay(1000);
  Motor.begin();
  delay(1000);
  delay(1000);
  Motor.setZero();
  Motor.p_in = 0;
  Motor.v_in = 0;
  Motor.t_in = 0;
  Motor.kp_in = 3;
  Motor.kd_in = 0;
  pinMode(limitL, INPUT);
  pinMode(limitR, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Motor.sendCommand();
  Motor.getReply();
  float p_out = Motor.getPosition();
  float v_out = Motor.getVelocity();
  float t_out = Motor.getTorque();
  Serial.print("p:");
  Serial.print(p_out);
  // Serial.print(",");
  Serial.print(",v:");
  Serial.print(v_out);
  // Serial.print(",");
  Serial.print(",t:");
  Serial.print(t_out);

  byte stateL = digitalRead(limitL);
  byte stateR = digitalRead(limitR);
  
  Serial.print(",left:");
  Serial.print(stateL);
  Serial.print(",right:");
  Serial.println(stateR);


  delay(100);
}
