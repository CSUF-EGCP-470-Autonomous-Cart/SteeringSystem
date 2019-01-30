/*
  Steering
  Braking
*/

//Must be before ros.h is imported
#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>

#include <PID_v1.h>


//#define DEBUG
#define LOOP_RATE 10 //Hz

#define BRAKE_A_PIN 7
#define BRAKE_B_PIN 8
#defien BRAKE_PID_P 2
#define BRAKE_PID_I 5
#define BRAKE_PID_D 1
double brakeSetpoint, brakeInput, brakeOutput;

#define STEER_DIRECTION_PIN 4
#define STEER_STEP_PIN 5
#define STEER_POT_PIN A0
#defien STEER_PID_P 2
#define STEER_PID_I 5
#define STEER_PID_D 1
double steerSetpoint, steerInput, steerOutput;


byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 3, 23);
IPAddress server(192 , 168, 3, 11);
const uint16_t serverPort = 11411;
ros::NodeHandle nh;

void CmdVelCb(const geometry_msgs::Twist& cmd_vel) {
  //Update steering PID setpoint
  steerSetpoint = cmd_vel.angular[0];

  //Update braking PID setpoint
  brakeSetpoint = cmd_vel.velocity[0];
}

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", CmdVelCb);


PID steerPID(&steerInput, &steerOutput, &steerSetpoint, STEER_PID_P, STEER_PID_I, STEER_PID_D, DIRECT);
PID brakePID(&brakeInput, &brakeOutput, &brakeSetpoint, BRAKE_PID_P, BRAKE_PID_I, BRAKE_PID_D, DIRECT);
void setup() {
  Serial.begin(115200);

  pinMode(STEER_DIRECTION_PIN, OUTPUT);
  pinMode(STEER_STEP_PIN, OUTPUT);
  pinMode(STEER_ENABLE_PIN, OUTPUT);

  pinMode(BRAKE_A_PIN, OUTPUT);
  pinMode(BRAKE_B_PIN, OUTPUT);

  Ethernet.begin(mac, ip);
  Serial.print("Client IP: ");
  Serial.println(Ethernet.localIP());

  //wait for ethernet shield to initalize
  delay(1000);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  nh.subscribe(twist_sub);

  steerPID.SetMode(AUTOMATIC);
  brakePID.SetMode(AUTOMATIC);

}

unsigned long prevRateTime = 0;
void loop() {
  if((millis() - prevRateTime) > (1000 / LOOP_RATE)) {
    prevPollTime = millis();

    #ifdef DEBUG

    #endif
  }

  //TODO steering PID
  //read steering potentiometer
  steerInput = map(analogRead(STEER_POT_PIN), 0, 1024, 0, 100);
  //update steering PID
  steerPID.Compute();
  //update steering output
  digitalWrite(STEER_DIRECTION_PIN, steerOutput > 0);
  analogWrite(STEER_STEP_PIN, steerOutput);

  //TODO braking PID
  //read brake pot/endstop/current speed?
  //brakeInput = ??
  //update braking PID
  brakePID.Compute();
  //update braking output
  if(brakeOutput > 0) {
    analogWrite(BRAKE_A_PIN, 0);
    analogWrite(BRAKE_B_PIN, brakeOutput);
  }
  else {
    analogWrite(BRAKE_A_PIN, brakeOutput);
    analogWrite(BRAKE_B_PIN, 0);
  }

  nh.spinOnce();
  delay(100);
}
