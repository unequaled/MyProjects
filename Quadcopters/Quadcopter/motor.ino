#include <Servo.h>

// This block is define the relatrionship between input and motor behavior
#define FIRST_INIT     ((char) 'v')
#define REG_INIT       ((char) 'b')
#define MOVE_FORWARD   ((char) 'w')
#define MOVE_BACKWARD  ((char) 's')
#define MOVE_TURN_R    ((char) 'd')
#define MOVE_TURN_L    ((char) 'a')
#define MOVE_UP        ((char) 'u')
#define MOVE_DOWN      ((char) 'i')
#define MOVE_STOP      ((char) 'm')
#define MOVE_SHOW      ((char) 'n')
#define MOVE_RESET      ((char) '_')
#define MOVE_TIME      20
#define MOTOR_UNIT     1


#define INIT_DIRECTION_SAMPLE_NUM 100
boolean MOTOR_INIT_STATUS = false;
//float Accel[3]= {0, 0, 0}; 
double Direct[3]= {0, 0, 0};                // Those are update for the latest pitch, rool and yaw. 
double Init_direct[3]= {0, 0, 0};           // Those are the setpoint for PID calculation. 
double Diff_motor_dir[3];                   // This are recorded the output of the PID calculation.
int Init_direct_count[3]= {0, 0, 0};        // For counting the setpoint sample numbers.

// This block is for PID Parameters
#define PID_Dir_Kp 1
#define PID_Dir_Ki 0.5
#define PID_Dir_Kd 0.5
#define PID_MIN_OUT -150
#define PID_MAX_OUT 150
//#define PID_SAMPLE_TIME 100

double PID_integral_dir[3]= {0,0,0};
double PID_previous_error_dir[3]= {0,0,0};

Servo motor[4];

int Total_power = 100;

//PWM power
int throttle[4] = {
	20, 20 ,20 ,20};

// PWM pins
int motor_pin[4] = {
	3, 5, 6, 9};

//This function is for the very first ESC initial.
void first_init_motors(){          

  for(int c=0;c<3;c++){
    Init_direct[c] = Direct[c];
  }
  
  for(int c=0; c<4; c++){
    throttle[c] = 150;
    motor[c].attach(motor_pin[c]);
  }
 Serial.println("motors are doing the first initializing....." );
  // stay in high speed about 2 secs
 sendSignalToMotor(2000);
 
 // stay in low speed about 1 secs
 for(int c=0; c<4; c++)
    throttle[c] = 20;
 sendSignalToMotor(1000);
 
 // Ready set the speed to 70
 for(int c=0; c<4; c++)
    throttle[c] = 30; 
 Total_power = 30*4;
 sendSignalToMotor(1000);
 
 MOTOR_INIT_STATUS = true; 
 Serial.println("motors are initialized." );
}

//This function is for regular motors start up.
void reg_init_motors(){          
  for(int c=0;c<3;c++){
    Init_direct[c] = Direct[c];
    Init_direct_count[c]++;
  }
  
 for(int c=0; c<4; c++){
   throttle[c] = 20;
   motor[c].attach(motor_pin[c]);
 }
 sendSignalToMotor(2000);
 Serial.println("motors are doing the reg initializing....." );
 
 for(int c=0; c<4; c++)
    throttle[c] = 30; 
 Total_power = 30*4;
 
 sendSignalToMotor(1000);
 MOTOR_INIT_STATUS = true;

 Serial.println("motors are initialized." );
}

// This is for updating motor speed.
void sendSignalToMotor(int time) {  
  // Sent data to ESC
  for(int d=0; d<time; d+=10){
    for(int c=0; c<4; c++) 
      motor[c].write(throttle[c]);
    delay(10);
  }
}

// Display the motors inforamtion
void showMotorsSpeed(){
  for(int c=0; c<4; c++){
    Serial.print("The motor# is "); Serial.print(c,DEC); Serial.print(" and the spped  is "); Serial.println(throttle[c]); 
  }  
  //Serial.print("The Accel x is "); Serial.print(Accel[0]); Serial.print(" y is "); Serial.print(Accel[1]); Serial.print(" z is "); Serial.println(Accel[2]);
  Serial.print("The pitch is "); Serial.print(Direct[0]); Serial.print(" roll is "); Serial.print(Direct[1]); Serial.print(" yaw is "); Serial.println(Direct[2]);
  Serial.print("The init pitch is "); Serial.print(Init_direct[0]);Serial.print(" roll is "); Serial.println(Init_direct[1]);
  Serial.print("The diff of x is "); Serial.print(Diff_motor_dir[0]); Serial.print(" y is "); Serial.println(Diff_motor_dir[1]); Serial.println();
  //Serial.print("The diff of x is "); Serial.print(Diff_motor_accel[0]); Serial.print(" y is "); Serial.println(Diff_motor_accel[1]);
}

// Check the motor is initialed or not
boolean check_motor_int() { return MOTOR_INIT_STATUS; }

//Keep balance of the motors
void motor_balance(){
    int ave= (Total_power - abs(Diff_motor_dir[0]) - abs(Diff_motor_dir[1]))/4;

//    throttle[0] = (int) ave - Diff_motor_accel[0]/2;
//    throttle[1] = (int) ave - Diff_motor_accel[1]/2;
//    throttle[2] = (int) ave + Diff_motor_accel[1]/2;
//    throttle[3] = (int) ave + Diff_motor_accel[0]/2;   
  
    throttle[0] = (int) ave - Diff_motor_dir[0]/2;
    throttle[1] = (int) ave - Diff_motor_dir[1]/2;
    throttle[2] = (int) ave + Diff_motor_dir[1]/2;
    throttle[3] = (int) ave + Diff_motor_dir[0]/2;   
}

//Update the motors informations and compute the PID result.
boolean update_motors_dir(float *dir, long sample_time){  
  double error[3];   
  
//  if(!MOTOR_INIT_STATUS)
//    return false;
    
  for (int c=0; c<2; c++){            // Only update pitch and roll
    Direct[c] = dir[c];
    error[c] = Direct[c] - Init_direct[c];
    if(MOTOR_INIT_STATUS)
      PID_integral_dir[c] += error[c];
    Diff_motor_dir[c] = PID_Dir_Kp*error[c] + PID_Dir_Ki*PID_integral_dir[c]*sample_time*0.001 + PID_Dir_Kd*(error[c] - PID_previous_error_dir[c])/(sample_time*0.001);   
    //Serial.print("The error is "); Serial.print(error[c]); Serial.print(" PID_integral_dir is "); Serial.print(PID_integral_dir[c]); Serial.print(" PID_previous_error_dir is "); Serial.println(PID_previous_error_dir[c]); 
    PID_previous_error_dir[c] = error[c];     
    
    //This bloc is for getting the average values for the setpoints of PID before the motor is not yet initial.
    //Set the integral to 0 after motors are initialed and the values has accumulation for INIT_DIRECTION_SAMPLE_NUM times.
    if(Init_direct_count[c] == INIT_DIRECTION_SAMPLE_NUM){
      if(!MOTOR_INIT_STATUS)
        Init_direct[c] += PID_integral_dir[c]/INIT_DIRECTION_SAMPLE_NUM;
      PID_integral_dir[c] = 0;
      Init_direct_count[c] = 0;      
    }
    Init_direct_count[c]++;    
  }
}

//Response the input/ 
void motor_contr(char val){
  int ave_speed = (throttle[0] + throttle[1] + throttle[2] + throttle[3])/4;

  switch(val){    
    case FIRST_INIT:
      first_init_motors();
      break;
    
    case REG_INIT:
      reg_init_motors();
      break;
    
    case MOVE_FORWARD:
      throttle[0] += MOTOR_UNIT;
      throttle[1] += MOTOR_UNIT;
      throttle[2] -= MOTOR_UNIT;
      throttle[3] -= MOTOR_UNIT;        
      Serial.println("Moving forward");
      sendSignalToMotor(MOVE_TIME);
      break;
      
    case MOVE_BACKWARD:
      throttle[0] -= MOTOR_UNIT;
      throttle[1] -= MOTOR_UNIT;
      throttle[2] += MOTOR_UNIT;
      throttle[3] += MOTOR_UNIT;  
      Serial.println("Moving backward");
      sendSignalToMotor(MOVE_TIME);
      break;
      
    case MOVE_TURN_R:
      throttle[0] += MOTOR_UNIT;
      throttle[1] -= MOTOR_UNIT;
      throttle[2] += MOTOR_UNIT;
      throttle[3] -= MOTOR_UNIT;  
      Serial.println("Turning right");
      sendSignalToMotor(MOVE_TIME);
      break;
      
    case MOVE_TURN_L:
      throttle[0] -= MOTOR_UNIT;
      throttle[1] += MOTOR_UNIT;
      throttle[2] -= MOTOR_UNIT;
      throttle[3] += MOTOR_UNIT;   
      Serial.println("Turning left");
      sendSignalToMotor(MOVE_TIME);    
      break;
      
    case MOVE_UP:    
      for(int c=0; c<4; c++)
        throttle[c] = ave_speed + MOTOR_UNIT; 
      Total_power += MOTOR_UNIT*4;
      Serial.println("motors are speedup");      
      sendSignalToMotor(MOVE_TIME);      
      break;
      
    case MOVE_DOWN:      
      for(int c=0; c<4; c++)
        throttle[c] = ave_speed - MOTOR_UNIT;
      Total_power -= MOTOR_UNIT*4*2;      
      Serial.println("motors are slowdown");
      sendSignalToMotor(MOVE_TIME);
      break;      
    
    case MOVE_STOP:      
      for(int c=0; c<4; c++)
        motor[c].detach();        
      Serial.println("motors are stop");
      sendSignalToMotor(MOVE_TIME);
      MOTOR_INIT_STATUS = false;
      break; 
 
//   case MOVE_SHOW:
//     showMotorsSpeed();
//     delay(100);
//     break;    
     
  case MOVE_RESET:
    for(int c=0; c<4; c++)
        throttle[c] = ave_speed; 
    Serial.println("motors set to average speed");
    sendSignalToMotor(MOVE_TIME);
    delay(100);
    break; 
  }
}
