/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
/*
PWM Pin In Arduino Nano
D3 // Interrupt
D5
D6
D9
D10
D11
*/
#ifdef L298_MOTOR_DRIVER
  #define LEFT_MOTOR_ENABLE   13 // 
  #define RIGHT_MOTOR_ENABLE  11 // 
  #define RIGHT_MOTOR_BACKWARD 5 // in4 // Motor B
  #define RIGHT_MOTOR_FORWARD  9 // in3 // Motor B
  #define LEFT_MOTOR_BACKWARD  6 // in2 // Motor A
  #define LEFT_MOTOR_FORWARD  10 // in1 // Motor A 

#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
