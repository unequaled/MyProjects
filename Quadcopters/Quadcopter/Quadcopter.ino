#define DATA_INPUT_INTERVAL 1000
#define OUTPUT_BAUD_RATE 9600

long timestamp_data_input;     // timestamp_data_input record the input interval from com port.
long sample_time;              // sample_time record the interval for sampling sensor input.

void setup(){
  Serial.begin(OUTPUT_BAUD_RATE);
  init_9DOF();                // Initial the sensor.
}

void loop(){
  char val = Serial.read();
  float dir[3];    //acc[3],
  
  // This block is updating the sensor info and senting to the motors.
  sample_time  = millis();  
  update_sensor(); 
  output_sensor_dir(dir,3);
  update_motors_dir(dir, millis() - sample_time);
  motor_balance();
    
  // This block is for recieving the input from COM port or ACP220
  if(val != -1){
    timestamp_data_input = millis();
    //if(check_motor_int()){         
      motor_contr(val);  
    //}
  }
  else if(check_motor_int() && (millis() - timestamp_data_input) >= DATA_INPUT_INTERVAL){
    timestamp_data_input = millis();
    showMotorsSpeed();
    //motor_contr('_');
  }  
}
