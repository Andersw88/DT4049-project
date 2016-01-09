
#define USE_USBCON
#include <vector>
#include <ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <motor_control_msgs/Position.h>
#include "sensors_project.h"

void setPositionCallback(const motor_control_msgs::Position& _msg);
void setVelCallback(const std_msgs::Int32& _msg);
void onOffCallback(const std_msgs::Empty& _msg);
void calibrateCallback(const std_msgs::Int32& msg);


ros::NodeHandle nh;
motor_control_msgs::MotorState motorState;
motor_control_msgs::ControlMsg controlState;
ros::Publisher motorStatePublisher("/motor_state", &motorState);
ros::Publisher controlStatePublisher("/control_state", &controlState);
ros::Subscriber<motor_control_msgs::Position> positionSubscriber("set_position", &setPositionCallback);
ros::Subscriber<std_msgs::Int32> velSubscriber("set_vel", &setVelCallback);
ros::Subscriber<std_msgs::Empty> onOffSubscriber("on_off", &onOffCallback);
ros::Subscriber<std_msgs::Int32> calSubscriber("calibrate", &calibrateCallback);


const float pwm_resolution = 4095;
float t_new, t_old, t_old_serial;
float dT = 0.001;


float current;

bool motor_on = false;

std::vector<MotorController> motorControllers;




void EncoderStates::readEncoder1(){

  // look for a low-to-high on channel A
  if (digitalRead(pin1) == HIGH) { 

    // check channel B to see which way encoder is turning
    if (digitalRead(pin2) == LOW) {  
      value = value + 1;         // CW
    } 
    else {
      value = value - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(pin2) == HIGH) {   
      value = value + 1;          // CW
    } 
    else {
      value = value - 1;          // CCW
    }
  }
}

void EncoderStates::readEncoder2(){

  // look for a low-to-high on channel B
  if (digitalRead(pin2) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(pin1) == HIGH) {  
      value = value + 1;         // CW
    } 
    else {
      value = value - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(pin1) == LOW) {   
      value = value + 1;          // CW
    } 
    else {
      value = value - 1;          // CCW
    }
  }
} 

template <int I>
void readEncoder1() {
  motorControllers[I].encoder.readEncoder1();
}

template <int I>
void readEncoder2() {
  motorControllers[I].encoder.readEncoder2();
}

void setup() {
  dT_serial = 0.05;
  
  
    motorControllers.push_back(
    MotorController(motorControllers.size(),
      EncoderStates(53 ,52 ,11, 13, A1, 0, 320), 
      PIDParameters(5.0, 5.0, 1.0, 0.0, pwm_resolution, 0, 0, 0), 
      ControlStates(0, 0, 0, 0, 0, 0, 1.0f, 0, false)));
	motorControllers[0].encoder.ratio = 30.72;
	motorControllers[0].calvel = 9.4; // Should change to 4.7;

  
  motorControllers.push_back(
    MotorController(motorControllers.size(),
      EncoderStates(51 ,50 ,3, 12, A0, 0, 187), 
      PIDParameters(5.0, 5.0, 1.0, 0.0, pwm_resolution, 0, 0, 0), 
      ControlStates(0, 0, 0, 0, 0, 0, 1.0f, 0, false)));
	motorControllers[1].encoder.ratio = 17.00;
	motorControllers[1].calvel = 4.7;
  

  t_old = micros()/1000000.0f;
  t_old_serial = micros()/1000000.0f;
  
  analogReadResolution(12);
  analogWriteResolution(12);

  
  attachInterrupt(digitalPinToInterrupt(motorControllers[1].encoder.pin1), readEncoder1<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorControllers[1].encoder.pin2), readEncoder2<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorControllers[0].encoder.pin1), readEncoder1<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorControllers[0].encoder.pin2), readEncoder2<0>, CHANGE);
  
  for(int i = 0; i < motorControllers.size(); i++)
  {
    analogWrite(motorControllers[i].encoder.motorPWMPin,0);
  }
  
  nh.initNode();
  nh.advertise(motorStatePublisher);
  nh.advertise(controlStatePublisher);
  nh.subscribe(positionSubscriber);
  nh.subscribe(velSubscriber);
  nh.subscribe(onOffSubscriber);
  nh.subscribe(calSubscriber);
}

void calibrateCallback(const std_msgs::Int32& msg) {
	// Calibration will start once calibration() is called.
	motorControllers[msg.data].calibrationStage = CalibrationStage::START;
	motorControllers[msg.data].positionController.active =  false;
}

// void MotorController::calibrate ()
// {
//   encoder.minValue = encoder.minValue < encoder.value ? encoder.minValue : encoder.value;
//   encoder.maxValue = encoder.maxValue > encoder.value ? encoder.maxValue : encoder.value; 
// }

float sign(float value) {
 return ((value>0)-(value<0));
}

void onOffCallback(const std_msgs::Empty& _msg) {
  
  for(int i = 0; i < motorControllers.size(); i++)
  {
    MotorController& mc = motorControllers[i];
    if(motor_on) {
      mc.positionController.active = false;
      mc.velocityController.active = false;
      motor_on = false;
      analogWrite(mc.encoder.motorPWMPin, 0);
    }
    else {
      motor_on = true;
      analogWrite(mc.encoder.motorPWMPin, 1000);
    }
  }
}

void setPositionCallback(const motor_control_msgs::Position& _msg) {
  MotorController& mc = motorControllers[_msg.joint_number];

  // 	// Reject msg if out of bounds.
  if(_msg.position < mc.encoder.minValue || _msg.position > mc.encoder.maxValue)
		  return;

	
  if((_msg.position*mc.encoder.ratio) > mc.encoder.value) {
	  mc.initVelocityControl(5.29*mc.encoder.ratio, 5, t_new);
  }
  else
	  mc.initVelocityControl(-5.29*mc.encoder.ratio, 5, t_new);

  // In degrees.
  mc.positionController.rf = _msg.position;
  mc.velocityController.active = true;
  mc.positionController.active = true;
//   mc.controller.rf_ = _msg.position * mc.encoder.maxValue/187.0f;
//   mc.controller.T_= 10.0f;
//   mc.controller.ri_ = mc.encoder.value;
//   mc.controller.ti_ = t_new;
//   mc.PID.I_ = 0;
//   mc.controller.velControlMode = false;
//   mc.controller.active_ = true;
}

void setVelCallback(const std_msgs::Int32& _msg) {
  MotorController& mc = motorControllers[0];
  mc.initVelocityControl(_msg.data,  5,  t_new );
  mc.velocityController.active = true;
}

void MotorController::calibration() {
	// Move the joint at one velocity.
	if(calibrationStage == CalibrationStage::INACTIVE) {
		return;
	}
	else if(calibrationStage == CalibrationStage::START) {
			initVelocityControl(static_cast<int>(-1*calvel*encoder.ratio), 5, t_new);
		velocityController.active = true;
		calibrationStage = CalibrationStage::FIRST_HALF;
	}
	else if(calibrationStage == CalibrationStage::FIRST_HALF) {
		if(encoder.current > 1.5) {
			velocityController.active = false;
			calibrationStage = CalibrationStage::SWITCHING;
		}
	}
	else if(calibrationStage == CalibrationStage::SWITCHING) {
		if(encoder.current < 0.1){
			encoder.value = 0;
			encoder.velocity = 0.0f;
			initVelocityControl(static_cast<int>(calvel*encoder.ratio), 5, t_new);
			velocityController.active = true;
			calibrationStage = CalibrationStage::SECOND_HALF;
		}
	}
	else {
		if(encoder.current > 1.5) {
			velocityController.active = false;
			encoder.maxValue = encoder.value;
			calibrationStage = CalibrationStage::INACTIVE;
		}
	}
	
}

void MotorController::actuate() {
  
  float control = velocityController.u_;
  int motor_dir = control > 0 ? LOW : HIGH;
  digitalWrite(encoder.motorDirPin, motor_dir);
  
  if (velocityController.active && abs(control) > 1.0)
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
  if (!positionController.active)
	  return;

  positionController.e = positionController.rf - (encoder.value/encoder.ratio);

  if(abs(positionController.e) > 2.0) {
    if(velocityController.active == false) //Only call initVelocityControl if velocityController was inactive.
      initVelocityControl((4.88*encoder.ratio)*(positionController.e >0?1:-1), 2, t_new);
    velocityController.active = true;
  }
  else 
    velocityController.active = false;
  
  if(encoder.current > 1.5) //Disables controller if current is to high, if the motor is unable to move futher.
  {
    velocityController.active = false;
    positionController.active = false;
  }  
}

void MotorController::velocityControl(const float t_new,  const float dT)
{
  if (!velocityController.active)
    return;

  float current_reference = minimumJerk(velocityController.ti_, t_new, velocityController.T_, velocityController.ri_, velocityController.rf_);

  // Step response instead of Minimum Jerk
  //float current_reference = controller.rf_;  
  
  float error = current_reference-encoder.velocity;
  float de_error=(error - velocityController.e_) / dT_serial;
  velocityController.u_ = pid(error, de_error);
  velocityController.r_ = current_reference;
  velocityController.e_= error;
  velocityController.de_= de_error;
}

void MotorController::initVelocityControl(float velocity,  float time,  float t_new) {
  velocityController.rf_ = velocity;
  velocityController.T_= time;
  velocityController.ri_ = encoder.velocity;
  velocityController.ti_ = t_new;
  PID.I_ = 0.0;
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
  
//   encoder.current = encoder.current*0.90 + analogRead(encoder.motorCurPin) * 0.10;
  
//   lastTime = micros()/1000000.0f - dT_serial;
  
  float currTime = micros()/1000000.0f;
  encoder.velocity = (encoder.velocity*0.90) + (((encoder.value - encoder.prevValue) / (currTime - encoder.lastTime)) * 0.10);

  float current = analogRead(encoder.motorCurPin) * (2.0f/(4095.0f));
  encoder.current = (encoder.current*0.90) + (current*0.10);
  
  encoder.lastTime = currTime;
  encoder.prevValue = encoder.value;
  
  motorState.position = encoder.value / encoder.ratio;
  motorState.velocity = encoder.velocity / encoder.ratio;
  motorState.current = encoder.current;
  motorState.id = id;
}

void MotorController::updateControl(){
  controlState.id = id;
  controlState.vel_control_active = velocityController.active;
  controlState.r = velocityController.r_ / encoder.ratio;

  controlState.r_final = velocityController.rf_ / encoder.ratio;
  controlState.r_initial = velocityController.ri_ / encoder.ratio;
  controlState.u = velocityController.u_;
  controlState.e = velocityController.e_ / encoder.ratio;

  controlState.position_control_active = positionController.active;
  controlState.position_error = positionController.e;
  controlState.position_setpoint = positionController.rf;
}


void loop() {
  

  t_new = micros()/1000000.0f;
  nh.spinOnce();
  
  if (abs(t_new - t_old_serial) > dT_serial) {
    
    for(int i = 0; i < motorControllers.size(); i++)
    {
      MotorController& mc = motorControllers[i];
      mc.updateState();

      mc.calibration();
      mc.velocityControl(t_new, t_new - t_old_serial);
      mc.positionControl();
      mc.updateControl();
      
      motorStatePublisher.publish(&motorState);
      controlStatePublisher.publish(&controlState);
      
      mc.actuate();
    }
    t_old_serial = t_new;
  }
}
