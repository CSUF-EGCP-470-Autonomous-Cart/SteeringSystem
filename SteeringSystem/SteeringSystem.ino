/* Arduino DC Motor Control */

#define direc 2
#define pwm   3
//#define gnd   4

int motorSpeed = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode (direc, OUTPUT);
  pinMode (pwm, OUTPUT);
  Serial.begin(115200);
  //pinMode (gnd, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  //int xAxis = analogRead(A0); //Read Joystick X-axis
  int yAxis = analogRead(A1); //Read Joystick Y-axis

  //Serial.print("\n");
  Serial.print("Y-axis: ");
  Serial.print(analogRead(yAxis));
  Serial.println("\n");
  //delay(500);
  //Y-axis used for forward and backward control
  if (yAxis < 470) {
    //Set Motor A forward
    digitalWrite (direc, HIGH);

    //Convert the declining Y-axis readings from going backward from 
    motorSpeed = map(yAxis, 470, 0, 0, 150);
    
  }
  else if (yAxis > 550) {
    //Set Motor backward
    digitalWrite (direc, LOW);

    //Convert the increasing Y-axis reading from going forward from 550
    motorSpeed = map (yAxis, 550, 1023, 0, 150);
    
  }
  //If Joystick stays in middle the motors are not moving
  else {
    motorSpeed = 0;
  }

  analogWrite (pwm, motorSpeed);  //Send PWM signal to motor

   delay(100);
}
