
#define USE_USBCON
#include <vector>
#include <ros.h>
// #include <arduino_pkg/MotorState.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include "sensors_project.h"

void readEncoder1();
void readEncoder2();

void setPositionCallback(const std_msgs::Int32& _msg);
void setVelCallback(const std_msgs::Int32& _msg);


ros::NodeHandle nh;
motor_control_msgs::MotorState motorState;
motor_control_msgs::ControlMsg controlState;
ros::Publisher motorStatePublisher("/motor_state", &motorState);
ros::Publisher controlStatePublisher("/control_state", &controlState);
ros::Subscriber<std_msgs::Int32> positionSubscriber("set_position", &setPositionCallback);
ros::Subscriber<std_msgs::Int32> velSubscriber("set_vel", &setVelCallback);


const float pwm_resolution = 4095;
float t_new, t_old, t_old_serial;
float dT = 0.001;
float dT_serial = 0.05;
float current;

std::vector<MotorController> motorControllers;

void setup() {
  
  
  motorControllers.push_back(
    MotorController(
      EncoderStates(51 ,52 ,3, 12, A0, 0), 
      PIDParameters(40.0, 0.0, 0.1, 0.0, pwm_resolution, 500, 0, 0), 
      ControlStates(0, 0, 0, 0, 0, 0, 1.0f, 0, false)));

  t_old = micros()/1000000.0f;
  t_old_serial = micros()/1000000.0f;
  
  analogReadResolution(12);
  analogWriteResolution(12);

  //Serial.begin(9600);

  
  attachInterrupt(digitalPinToInterrupt(motorControllers[0].encoder.pin1), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorControllers[0].encoder.pin2), readEncoder2, CHANGE);

  
  nh.initNode();
  nh.advertise(motorStatePublisher);
  nh.advertise(controlStatePublisher);
  nh.subscribe(positionSubscriber);
  nh.subscribe(velSubscriber);
  
  
  analogWrite(motorControllers[0].encoder.motorPWMPin,1095);
}

void MotorController::calibrate ()
{
  encoder.minValue = encoder.minValue < encoder.value ? encoder.minValue : encoder.value;
  encoder.maxValue = encoder.maxValue > encoder.value ? encoder.maxValue : encoder.value; 
}
 
// void calibrate (int num) {
//   //analogWrite(encoders[0].motorPWMPin,2095);
//   digitalWrite(encoders[num].motorDirPin, HIGH);
//   delay(1000);
//   float t_old = 0;
//   float t_new = 0;
//   initVelocityControl(controllers[num],  encoders[num],  10,  3,  t_new);
//   while(true) {
//     delay(200);
// //     if(encoders[num].value == encoders[num].prevValue)
// //       break;
//     encoders[num].prevValue = encoders[num].value;
//     velocityControl(controllers[num], encoders[num], PIDs[num], t_new, t_new - t_old);
//     
//     actuate(controllers[num].u_,  encoders[num]);
//     updateState();
//     state_publisher.publish(&state);
//     t_old = t_new;
//     t_new = micros() / 1000000.0f;
//   }
//   analogWrite(encoders[num].motorPWMPin,0);
//   delay(1000);
// //   encoders[num].maxValue = encoders[num].value;
//   encoders[num].value = 0;
//   encoders[num].minValue = 0;
//   
//   
//   analogWrite(encoders[0].motorPWMPin,2095);
//   digitalWrite(encoders[num].motorDirPin, LOW);
//   
//   delay(1000);
//   while(true) {
//     if(encoders[num].value == encoders[num].prevValue)
//       break;
//     encoders[num].prevValue = encoders[num].value;
//     
//     updateState();
//     state_publisher.publish(&state);
// //     delay(1000);
//   }
//   
//   encoders[num].maxValue = encoders[num].value;
//   analogWrite(encoders[num].motorPWMPin,0);
//   
// //   calibration::active = false;
// //   calibration::motorNum = 0;
// }

void readEncoder1() {
  motorControllers[0].encoder.readEncoder1();
}

void EncoderStates::readEncoder1() {
  
  if (tickTack)
  {
    tickTack = !tickTack;
    int encoderValue1=digitalRead(this->pin1);
    int encoderValue2=digitalRead(this->pin2);
    this->value += (encoderValue2==0) ? encoderValue1 : -encoderValue1;
  }
}

void readEncoder2() {
  motorControllers[0].encoder.readEncoder2();
}

void EncoderStates::readEncoder2() {
  
  if (!tickTack)
  {
    tickTack = !tickTack;
  }
}

float sign(float value) {
 return ((value>0)-(value<0));
}

void setPositionCallback(const std_msgs::Int32& _msg) {
  
  MotorController& mc = motorControllers[0];
  
  mc.controller.rf_ = _msg.data * mc.encoder.maxValue/187.0f;
  mc.controller.T_= 10.0f;
  mc.controller.ri_ = mc.encoder.value;
  mc.controller.ti_ = t_new;
  mc.PID.I_ = 0;
  mc.controller.velControlMode = false;
  mc.controller.active_ = true;
}

void setVelCallback(const std_msgs::Int32& _msg) {
  
  MotorController& mc = motorControllers[0];
  
  mc.initVelocityControl(_msg.data,  3,  t_new );
  mc.controller.active_ = true;
}

void MotorController::actuate() {
  
  float control = controller.u_;
  int motor_dir = control > 0 ? LOW : HIGH;
  digitalWrite(encoder.motorDirPin, motor_dir);
  
  if (controller.active_ && abs(control) > 1.0)
  {
    analogWrite(encoder.motorPWMPin, abs(control) + PID.u_min_);
  }
  else 
  {
    analogWrite(encoder.motorPWMPin,0);
  }
}

//--------------------------------------------------------------------------
void MotorController::positionControl()
{
  if (controller.active_){
    float current_reference = minimumJerk(controller.ti_, t_new, controller.T_, controller.ri_, controller.rf_);
    float error = current_reference-encoder.value;
    float de_error=(error - controller.e_) / dT_serial;
    controller.u_ = pid(error, de_error);
    controller.r_ = current_reference;
    controller.e_= error;
    controller.de_= de_error;
  }
  else{controller.u_=0;}
    encoder.prevValue = encoder.value;
}

void MotorController::velocityControl(const float t_new,  const float dT)
{
  if (!controller.active_)
    return;

  float current_reference = minimumJerk(controller.ti_, t_new, controller.T_, controller.ri_, controller.rf_);
  
  encoder.dp_ = (encoder.value - encoder.prevValue) / dT_serial;
  // * encoder.filterAlpha + (encoder.dp_ * (1.0 - encoder.filterAlpha));

  float error = current_reference-encoder.dp_;
  float de_error=(error - controller.e_) / dT_serial;
  controller.u_ = pid(error, de_error);
  controller.r_ = current_reference;
  controller.e_= error;
  controller.de_= de_error;
  
  encoder.prevValue = encoder.value;
}

void MotorController::initVelocityControl(float velocity,  float time,  float t_new) {
  controller.rf_ = velocity;
  controller.T_= time;
  controller.ri_ = encoder.dp_;
  controller.ti_ = t_new;
}

float MotorController::minimumJerk(float t0, float t, float T, float q0, float qf)
{
  if((t - t0) / T >= 1.0)
    return qf;
  return q0 + (qf - q0) * (10 * pow((t - t0) / T, 3) - 15 * pow((t - t0) / T, 4) + 6 * pow((t - t0) /T, 5));
}

float MotorController::pid(float e, float de)
{
  PID.I_ = PID.I_ + PID.Ki_ * e + PID.Kc_ * PID.SatErr;
  float control = PID.Kp_ * e + PID.Kd_ * de + PID.I_;
  float controlClamped = abs(control) < PID.u_max_ - PID.u_min_ ? abs(control) : PID.u_max_ - PID.u_min_;
  controlClamped *= sign(control);
  PID.SatErr = controlClamped-control;
  return controlClamped;
}

//--------------------------------------------------------------------------
void MotorController::updateState() {
  motorState.position = encoder.value;
  motorState.velocity = encoder.dp_;
  
  motorState.current = motorState.current*0.90 + analogRead(encoder.motorCurPin) * 0.10;
//     motorState.current = analogRead(motorControllers[0].encoder.motorCurPin);
//   motorState.current = analogRead(encoder.motorCurPin) * (2.0f/(4095.0f));
  
  motorState.id = 0;
  
  controlState.id = 0;
  controlState.r = controller.r_;

  controlState.r_final = controller.rf_;
  controlState.r_initial = controller.ri_;
  controlState.u = controller.u_;
  controlState.e = controller.e_;
}


void loop() {
  

  t_new = micros()/1000000.0f;
  nh.spinOnce();
  
  if (abs(t_new - t_old_serial) > dT_serial) {
    
    
    MotorController& mc = motorControllers[0];
    
//     if (mc.encoder.calibrate)
      mc.calibrate();
    
//     velocityControl(controllers[0], encoders[0], PIDs[0], t_new, t_new - t_old_serial);
//     velocityControl(controllers[0], encoders[0], PIDs[0], t_new, t_new - t_old_serial);
//     mc.positionControl();
    mc.updateState();
    
    motorStatePublisher.publish(&motorState);
    controlStatePublisher.publish(&controlState);
    
//     mc.actuate();
    t_old_serial = t_new;
  }
}
