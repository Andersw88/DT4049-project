
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

// struct MotorShieldPins {
//   int dir; //direction pin
//   int pwm; //pwm pin
//   int cur; //current sensor
// 
//   MotorShieldPins(int dir, int pwm, int cur) 
//   : dir(dir), pwm(pwm), cur(cur) {};
// };

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
  
//   int value_debug1;
//   int value_debug2;
  bool tickTack;
  bool calibrate;
  
  
  float dp_;
//   float filterAlpha;

  EncoderStates(int PIN1, int PIN2, int motorPWMPin, int motorDirPin, int motorCurPin, int pos) 
  : pin1(PIN1), pin2(PIN2), motorPWMPin(motorPWMPin), motorDirPin(motorDirPin), motorCurPin(motorCurPin), value(pos), calibrate(false)
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
};


class MotorController
{
public:
  EncoderStates encoder;
  PIDParameters PID;
  ControlStates controller;
  arduino_pkg::MotorState state;
  
  
  MotorController(EncoderStates encoder, PIDParameters PID, ControlStates controller) :  encoder(encoder), PID(PID), controller(controller)
  {
    
  }
  
  void actuate();
  void positionControl();
  void velocityControl(const float t_new,  const float dT);
  void initVelocityControl(float velocity,  float time,  float t_new);


  float minimumJerk(float t0, float t, float T, float q0, float qf);
  float pid(float e, float de);
  void updateState();
  void calibrate ();
  
};