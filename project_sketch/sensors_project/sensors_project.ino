#define ROS_SUPPORT 1

#if ROS_SUPPORT
#define USE_USBCON
#include <vector>
#include <ros.h>
// #include <unistd.h>
#include <arduino_pkg/MotorState.h>
#include <arduino_pkg/SetPosition.h>
#include <arduino_pkg/SetPID.h>
#include <arduino_pkg/SetVelocity.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>

#include "sensors_project.h"
// #include <algorithm>
#endif
/*

struct ControlStates
{
  float r_; //setpoint value at current iteration
  float rf_; //final setpoint value
  float ri_; //initial setpoint value
  float e_; //error
  float de_; //error derivative

  float ti_; //time when we initialized motion (seconds)
  float T_; //time for executing the loop

  float u_; //computed control

  bool active_; //flag indicating whether the corresponding controller is active or not
  bool velControlMode;

//   ControlStates(float r,  float e) 
//   : r_(r), rf_(rf), ri_(ri), e_(e), de_(de), ti_(ti), T_(T), u_(u), active_(active), velControlMode(false) {};
  
  ControlStates(float r, float rf, float ri, float e, float de, float ti, float T, int u = 0, bool active = 0) 
  : r_(r), rf_(rf), ri_(ri), e_(e), de_(de), ti_(ti), T_(T), u_(u), active_(active), velControlMode(false) {};
};

//this struct holds the variables for a PID controller
struct PIDParameters {
  float Kp_;
  float Kd_;
  float Ki_;
  float Kc_;            
  float u_max_; //Maximum controller output (<= max PWM)
  float u_min_; //Minimum controller output [>= -(max PWM)]
  float SatErr;  
  float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]

  PIDParameters(float Kp, float Ki, float Kd,float Kc, float u_max, float u_min, float SatErr, float I) 
  : Kp_(Kp), Kd_(Kd), Ki_(Ki), Kc_(Kc), u_max_(u_max), u_min_(u_min), I_(I), SatErr(SatErr) {};
};

//this struct holds the pin numbers for the motor shield
struct MotorShieldPins {
  int DIR_; //direction pin
  int PWM_; //pwm pin
  int BRK_; //brake pin
  int CUR_; //current sensor

  MotorShieldPins(int DIR, int BRK, int PWM_pin, int CUR) 
  : DIR_(DIR), BRK_(BRK), PWM_(PWM_pin), CUR_(CUR) {};
};

//this struct holds the pin for the encoder and the current number of ticks
struct EncoderStates
{
  int pin1; 
  int pin2;
  int motorPWMPin;
  int motorDirPin;
  int motorCurPin;
  
  int value;
  int prevValue;
  
  int maxValue;
  int minValue;
  
  int value_debug1;
  int value_debug2;
  bool tickTack;
  
//   int p1_;
//   int p2_;
  
  float dp_;//time-derivative of the encoder pulses
//   float filterAlpha;

  EncoderStates(int PIN1, int PIN2, int motorPWMPin, int motorDirPin, int pos) 
  : pin1(PIN1), pin2(PIN2), motorPWMPin(motorPWMPin), motorDirPin(motorDirPin), value(pos) 
  {
    pinMode(pin1, INPUT_PULLUP);
    pinMode(pin2, INPUT_PULLUP);

    pinMode(motorPWMPin, OUTPUT);
    pinMode(motorDirPin, OUTPUT);


    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, HIGH);
    digitalWrite(motorDirPin, HIGH);
    
    dp_ = 0;
  };
  
  void readEncoder1();
  void readEncoder2();
};*/

void readEncoder1();
void readEncoder2();


// void actuate(float control, const EncoderStates& encoder);
// void positionControl(ControlStates& controller, EncoderStates& encoder, PIDParameters& PID);
// void velocityControl(ControlStates& controller, EncoderStates& encoder, PIDParameters& PID, const float t_new,  const float dT);
// void initVelocityControl(ControlStates & controller, EncoderStates& encoder,  float velocity,  float time,  float t_new);
// 
// 
// float minimumJerk(float t0, float t, float T, float q0, float qf);
// float pid(float e, float de, PIDParameters& PID);
// void updateState();
// void calibrate (int num);
void setPositionCallback(const std_msgs::Int32& _msg);
void setVelCallback(const std_msgs::Int32& _msg);


ros::NodeHandle nh;
arduino_pkg::MotorState state;
ros::Publisher state_publisher("/motor_state", &state);
ros::Subscriber<std_msgs::Int32> positionSubscriber("set_position", &setPositionCallback);
ros::Subscriber<std_msgs::Int32> velSubscriber("set_vel", &setVelCallback);


const float pwm_resolution = 4095;
//storage for timers
float t_new, t_old, t_old_serial;
//sampling time in seconds
float dT = 0.001;
float dT_serial = 0.05;
//storage of the electric current through the motors
float current;

std::vector<MotorController> motorControllers;

// std::vector<EncoderStates> encoders;
// std::vector<ControlStates> controllers;
// std::vector<PIDParameters> PIDs; ControlStates(0, 0, 0, 0, 0, 0, 1.0f, 0, false), 

void setup() {
  
  
  motorControllers.push_back(
    MotorController(
      EncoderStates(51 ,52 ,3, 12, A0, 0), 
      PIDParameters(40.0, 0.0, 0.1, 0.0, pwm_resolution, 500, 0, 0), 
      ControlStates(0, 0, 0, 0, 0, 0, 1.0f, 0, false)));
  
//   encoders.push_back(EncoderStates(51 ,52 ,3, 12, 0));
//   controllers.push_back(ControlStates(0, 0, 0, 0, 0, 0, 1.0f, 0, false));
//   PIDs.push_back(PIDParameters(40.0, 0.0, 0.1, 0.0, pwm_resolution, 500, 0, 0));
//   encoders.push_back(EncoderStates(51,0));

  t_old = micros()/1000000.0f;
  t_old_serial = micros()/1000000.0f;
  
  analogReadResolution(12);
  analogWriteResolution(12);

  //Serial.begin(9600);

  
  attachInterrupt(digitalPinToInterrupt(motorControllers[0].encoder.pin1), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorControllers[0].encoder.pin2), readEncoder2, CHANGE);


  nh.initNode();
  nh.advertise(state_publisher);
  nh.subscribe(positionSubscriber);
  nh.subscribe(velSubscriber);
  
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
//     this->value_debug1 = encoderValue1;
//     this->value_debug2 = encoderValue2;
  }
}

void readEncoder2() {
  motorControllers[0].encoder.readEncoder2();
}

void EncoderStates::readEncoder2() {
  
  if (!tickTack)
  {
    tickTack = !tickTack;
//     int encoderValue1=digitalRead(this->pin1);
//     int encoderValue2=digitalRead(this->pin2);
//     this->value += (encoderValue1==0) ? encoderValue2 : -encoderValue2;
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


// void setVelCallback(const std_msgs::Int32& _msg) {
//   
//   MotorController& mc = motorControllers[0];
//   
//   initVelocityControl(controllers[0],  encoders[0],  _msg.data,  3,  t_new );
//   controllers[0].active_ = true;
// }

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

// void actuate(float control, const EncoderStates& encoder) {
//   
//   int motor_dir = control > 0 ? LOW : HIGH;
//   digitalWrite(encoder.motorDirPin, motor_dir);
//   
// 
//   if (controllers[0].active_ && abs(control) > 1.0)
//   {
//     analogWrite(encoder.motorPWMPin, abs(control) + PIDs[0].u_min_);
//   }
//   else 
//   {
//     analogWrite(encoder.motorPWMPin,0);
//   }
// }

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

// void positionControl(ControlStates& controller, EncoderStates& encoder, PIDParameters& PID)
// {
//   if (controller.active_){
//     float current_reference = minimumJerk(controller.ti_, t_new, controller.T_, controller.ri_, controller.rf_);
//     float error = current_reference-encoder.value;
//     float de_error=(error - controller.e_) / dT_serial;
//     controller.u_ = pid(error, de_error, PID);
//     controller.r_ = current_reference;
//     controller.e_= error;
//     controller.de_= de_error;
//   }
//   else{controller.u_=0;}
//     encoder.prevValue = encoder.value;
// }

// void velocityControl(ControlStates& controller, EncoderStates& encoder, PIDParameters& PID, const float t_new,  const float dT)
// {
//   if (!controller.active_)
//     return;
// 
//   float current_reference = minimumJerk(controller.ti_, t_new, controller.T_, controller.ri_, controller.rf_);
//   
//   encoder.dp_ = (encoder.value - encoder.prevValue) / dT_serial;
//   // * encoder.filterAlpha + (encoder.dp_ * (1.0 - encoder.filterAlpha));
// 
//   float error = current_reference-encoder.dp_;
//   float de_error=(error - controller.e_) / dT_serial;
//   controller.u_ = pid(error, de_error, PID);
//   controller.r_ = current_reference;
//   controller.e_= error;
//   controller.de_= de_error;
//   
//   encoder.prevValue = encoder.value;
// }

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

// void initVelocityControl(ControlStates& controller, EncoderStates& encoder,  float velocity,  float time,  float t_new) {
//   controller.rf_ = velocity;
//   controller.T_= time;
//   controller.ri_ = encoder.dp_;
//   controller.ti_ = t_new;
// }

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

// float pid(float e, float de, PIDParameters& PID)
// {
//   PID.I_ = PID.I_ + PID.Ki_ * e + PID.Kc_ * PID.SatErr;
// 
//   float control = PID.Kp_ * e + PID.Kd_ * de + PID.I_;
//   
//   float controlClamped = abs(control) < PID.u_max_ - PID.u_min_ ? abs(control) : PID.u_max_ - PID.u_min_;
//   
//   controlClamped *= sign(control);
// 
//   PID.SatErr = controlClamped-control;
//   
//   return controlClamped;
// }

//--------------------------------------------------------------------------
void MotorController::updateState() {

  
  state.encoder = encoder.value;

  state.time = (t_new - controller.ti_)/controller.T_;
  state.r_ = controller.r_;
  state.ri_ = controller.ri_;
  state.rf_ = controller.rf_;
  state.T_ = controller.T_;
  state.ref = encoder.dp_;
  state.control = controller.u_;
  state.p_I = PID.I_;
  state.error = controller.e_;
  
  
//   state.ri_ = encoder.maxValue;
//   state.rf_ = encoder.minValue;
}

// void updateState() {
// 
//   
//   state.encoder = encoders[0].value;
// 
//   state.time = (t_new - controllers[0].ti_)/controllers[0].T_;
//   state.r_ = controllers[0].r_;
//   state.ri_ = controllers[0].ri_;
//   state.rf_ = controllers[0].rf_;
//   state.T_ = controllers[0].T_;
//   state.ref = encoders[0].dp_;
//   state.control = controllers[0].u_;
//   state.p_I = PIDs[0].I_;
//   state.error = controllers[0].e_;
// }

void loop() {
  

  t_new = micros()/1000000.0f;
  nh.spinOnce();
  if (abs(t_new - t_old_serial) > dT_serial) {

    
    MotorController& mc = motorControllers[0];
    
//     if (mc.encoder.calibrate)
      mc.calibrate();
    
//     velocityControl(controllers[0], encoders[0], PIDs[0], t_new, t_new - t_old_serial);
//     velocityControl(controllers[0], encoders[0], PIDs[0], t_new, t_new - t_old_serial);
    mc.positionControl();
    mc.updateState();
    state_publisher.publish(&mc.state);
    mc.actuate();
    t_old_serial = t_new;
  }
}
