#include <motor_control_msgs/MotorState.h>
#include <motor_control_msgs/ControlMsg.h>


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

struct PositionControl {
	float rf;
	float e;
	bool active;

	PositionControl() {
		rf = 0.0;
		e = 0.0;
		active = false;
	}
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

struct EncoderStates
{
  int pin1; 
  int pin2;
  int motorPWMPin;
  int motorDirPin;
  int motorCurPin;
  
  float current;
  
  int value;
  int prevValue;
  
  int maxValue;
  int minValue;
  
  bool tickTack;
  bool calibrate;
  
  float velocity;

  EncoderStates(int PIN1, int PIN2, int motorPWMPin, int motorDirPin, int motorCurPin, int pos) 
  : pin1(PIN1), pin2(PIN2), motorPWMPin(motorPWMPin), motorDirPin(motorDirPin), motorCurPin(motorCurPin), value(pos), calibrate(false)
  {
    pinMode(pin1, INPUT_PULLUP);
    pinMode(pin2, INPUT_PULLUP);
    
//     pinMode(pin1, INPUT);
//     pinMode(pin2, INPUT);

    pinMode(motorPWMPin, OUTPUT);
    pinMode(motorDirPin, OUTPUT);

//     digitalWrite(pin1, HIGH);
//     digitalWrite(pin2, HIGH);
    digitalWrite(motorDirPin, HIGH);

	current = 0.0f;
    velocity = 0.0f;
  };
  
  void readEncoder1();
  void readEncoder2();
};

// Below c++11 we don't have scoped enums. Hence we add a ns here.
namespace CalibrationStage {
enum CalibrationStage {
	INACTIVE = 0,
	START = 1,
	FIRST_HALF = 2,
	SWITCHING = 3,
	SECOND_HALF = 4
};
}
class MotorController
{
	
public:
  CalibrationStage::CalibrationStage calibrationStage;
  EncoderStates encoder;
  PIDParameters PID;
  ControlStates controller;
	PositionControl positionController;

  
	MotorController(EncoderStates encoder, PIDParameters PID, ControlStates controller) :  encoder(encoder), PID(PID), controller(controller), calibrationStage(CalibrationStage::INACTIVE)
  {
    
  }
  
  void actuate();
  void positionControl();
  void velocityControl(const float t_new,  const float dT);
  void initVelocityControl(float velocity,  float time,  float t_new);
  void calibration();

  float minimumJerk(float t0, float t, float T, float q0, float qf);
  float pid(float e, float de);
  void updateState();
  void updateControl();
  void calibrate ();
  
};
