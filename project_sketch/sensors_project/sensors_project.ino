
#define USE_USBCON
#include <vector>
#include <ros.h>
// #include <arduino_pkg/MotorState.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "sensors_project.h"

void readEncoder1();
void readEncoder2();

void setPositionCallback(const std_msgs::Float32& _msg);
void setVelCallback(const std_msgs::Int32& _msg);
void onOffCallback(const std_msgs::Empty& _msg);
void calibrateCallback(const std_msgs::Empty& msg);


ros::NodeHandle nh;
motor_control_msgs::MotorState motorState;
motor_control_msgs::ControlMsg controlState;
ros::Publisher motorStatePublisher("/motor_state", &motorState);
ros::Publisher controlStatePublisher("/control_state", &controlState);
ros::Subscriber<std_msgs::Float32> positionSubscriber("set_position", &setPositionCallback);
ros::Subscriber<std_msgs::Int32> velSubscriber("set_vel", &setVelCallback);
ros::Subscriber<std_msgs::Empty> onOffSubscriber("on_off", &onOffCallback);
ros::Subscriber<std_msgs::Empty> calSubscriber("calibrate", &calibrateCallback);


const float pwm_resolution = 4095;
float t_new, t_old, t_old_serial;
float dT = 0.001;
float dT_serial = 0.05;
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

// void readEncoder1_mc1() {
//   motorControllers[I].encoder.doEncoderA();
// }
// 
// void readEncoder2_mc1() {
//   motorControllers[I].encoder.doEncoderB();
// }

void setup() {
  
  
  motorControllers.push_back(
    MotorController(
      EncoderStates(51 ,50 ,3, 12, A0, 0), 
      PIDParameters(5.0, 5.0, 1.0, 0.0, pwm_resolution, 0, 0, 0), 
      ControlStates(0, 0, 0, 0, 0, 0, 1.0f, 0, false)));

  t_old = micros()/1000000.0f;
  t_old_serial = micros()/1000000.0f;
  
  analogReadResolution(12);
  analogWriteResolution(12);

  
  attachInterrupt(digitalPinToInterrupt(motorControllers[0].encoder.pin1), readEncoder1<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorControllers[0].encoder.pin2), readEncoder2<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorControllers[1].encoder.pin1), readEncoder1<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorControllers[1].encoder.pin2), readEncoder2<1>, CHANGE);
  
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

void calibrateCallback(const std_msgs::Empty& msg) {
	// Calibration will start once calibration() is called.
	motorControllers[0].calibrationStage = CalibrationStage::START;
}

void MotorController::calibrate ()
{
  encoder.minValue = encoder.minValue < encoder.value ? encoder.minValue : encoder.value;
  encoder.maxValue = encoder.maxValue > encoder.value ? encoder.maxValue : encoder.value; 
}
/*
void readEncoder1_mc0() {
  motorControllers[0].encoder.readEncoder1();
}

void readEncoder2_mc0() {
  motorControllers[0].encoder.readEncoder1();
}

void readEncoder2_mc1() {
  motorControllers[1].encoder.readEncoder2();
}

void readEncoder2_mc1() {
  motorControllers[1].encoder.readEncoder2();
}

void EncoderStates::readEncoder1() {
  
  if (tickTack)
  {
    tickTack = !tickTack;
    int encoderValue1=digitalRead(this->pin1);
    int encoderValue2=digitalRead(this->pin2);
//     this->value += (encoderValue2==0) ? encoderValue1 : -encoderValue1;
    
    
    if(encoderValue1)
    {
      tickTack = !tickTack;
      this->value += (encoderValue2==0) ? 1 : -1;
    }
//     this->value += (encoderValue2==0) ? 1 : -1;
//     this->value++;
  }
}



void EncoderStates::readEncoder2() {
  
  if (!tickTack)
  {

    int encoderValue1=digitalRead(this->pin1);
    int encoderValue2=digitalRead(this->pin2);
//     this->value += (encoderValue1!=0) ? encoderValue2 : -encoderValue2;
    
    if(encoderValue2)
    {
      tickTack = !tickTack;
      this->value += (encoderValue1!=0) ? 1 : -1;
    }
//     this->value += (encoderValue1!=0) ? 1 : -1;
  }
}*/


float sign(float value) {
 return ((value>0)-(value<0));
}

void onOffCallback(const std_msgs::Empty& _msg) {

	MotorController& mc = motorControllers[0];
	  
	if(motor_on) {
		motor_on = false;
		analogWrite(mc.encoder.motorPWMPin, 0);
	}
	else {
		motor_on = true;
		analogWrite(mc.encoder.motorPWMPin, 1000);
	}
}

void setPositionCallback(const std_msgs::Float32& _msg) {
  MotorController& mc = motorControllers[0];

  if(_msg.data < 0.0 || _msg.data > 187.00)
	  return;
  if((_msg.data*17) > mc.encoder.value) {
	  mc.initVelocityControl(90, 5, t_new);
  }
  else
	  mc.initVelocityControl(-90, 5, t_new);

  // In degrees.
  mc.positionController.rf = _msg.data;
  mc.controller.active_ = true;
  mc.positionController.active = true;
//   mc.controller.rf_ = _msg.data * mc.encoder.maxValue/187.0f;
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
  mc.controller.active_ = true;
}

void MotorController::calibration() {
	// Move the joint at one velocity.
	if(calibrationStage == CalibrationStage::INACTIVE) {
		return;
	}
	else if(calibrationStage == CalibrationStage::START) {
		initVelocityControl(-80, 5, t_new);
		controller.active_ = true;
		calibrationStage = CalibrationStage::FIRST_HALF;
	}
	else if(calibrationStage == CalibrationStage::FIRST_HALF) {
		if(encoder.current > 1.5) {
			encoder.value = 0;
			encoder.velocity = 0.0f;
			initVelocityControl(80, 5, t_new);
			calibrationStage = CalibrationStage::SWITCHING;
		}
	}
	else if(calibrationStage == CalibrationStage::SWITCHING) {
		if(encoder.current > 1.5) {
			// we need to wait for the arm to turn around.
			return;
		}
		else {
			calibrationStage = CalibrationStage::SECOND_HALF;
		}
	}
	else {
		if(encoder.current > 1.5) {
			controller.active_ = false;
			calibrationStage = CalibrationStage::INACTIVE;
		}
	}
	
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
  if (!positionController.active)
	  return;

  positionController.e = positionController.rf - (encoder.value/17.0f);
  // Close to finish.
  if(abs(positionController.e) < 2.0) {
	  initVelocityControl(83*(positionController.e >0?1:-1), 2, t_new);
  }
  // Have we reached the position?
  // else if(abs(positionController.e) < 1.0) {
  // 	  controller.active_ = false;
  // 	  positionController.active = false;
  // }
  
}

void MotorController::velocityControl(const float t_new,  const float dT)
{
  if (!controller.active_)
    return;

  float current_reference = minimumJerk(controller.ti_, t_new, controller.T_, controller.ri_, controller.rf_);

  // Step response instead of Minimum Jerk
  //float current_reference = controller.rf_;  
  
  float error = current_reference-encoder.velocity;
  float de_error=(error - controller.e_) / dT_serial;
  controller.u_ = pid(error, de_error);
  controller.r_ = current_reference;
  controller.e_= error;
  controller.de_= de_error;
}

void MotorController::initVelocityControl(float velocity,  float time,  float t_new) {
  controller.rf_ = velocity;
  controller.T_= time;
  controller.ri_ = encoder.velocity;
  controller.ti_ = t_new;
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
  
  static float lastTime = micros()/1000000.0f - dT_serial;
  
  float currTime = micros()/1000000.0f;
  encoder.velocity = (encoder.velocity*0.90) + (((encoder.value - encoder.prevValue) / (currTime - lastTime)) * 0.10);

  float current = analogRead(encoder.motorCurPin) * (2.0f/(4095.0f));
  encoder.current = (encoder.current*0.90) + (current*0.10);
  
  lastTime = currTime;
  encoder.prevValue = encoder.value;
  
  motorState.position = encoder.value / 17.00;
  motorState.velocity = encoder.velocity / 17.00;
  motorState.current = encoder.current;
  motorState.id = 0;
}

void MotorController::updateControl(){
  controlState.id = 0;
  controlState.vel_control_active = controller.active_;
  controlState.r = controller.r_ / 17.00;

  controlState.r_final = controller.rf_ / 17.00;
  controlState.r_initial = controller.ri_ / 17.00;
  controlState.u = controller.u_;
  controlState.e = controller.e_ / 17.00;

  controlState.position_control_active = positionController.active;
  controlState.position_error = positionController.e;
  controlState.position_setpoint = positionController.rf;
}


void loop() {
  

  t_new = micros()/1000000.0f;
  nh.spinOnce();
  
  if (abs(t_new - t_old_serial) > dT_serial) {
    
    
    MotorController& mc = motorControllers[0];
    
    mc.updateState();

	mc.calibration();
    mc.velocityControl(t_new, t_new - t_old_serial);
	mc.positionControl();

	mc.updateControl();
    
    motorStatePublisher.publish(&motorState);
    controlStatePublisher.publish(&controlState);
    
    mc.actuate();
    t_old_serial = t_new;
  }
}
