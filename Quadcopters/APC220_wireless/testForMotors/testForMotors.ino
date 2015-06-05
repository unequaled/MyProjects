#include <SoftwareSerial.h>
SoftwareSerial apc(2,3);  // rx/tx
char txt;
void setup()
{
  apc.begin(9600); // match APC communication-rate
  Serial.begin(9600);
}

void loop()
{  
  int value = 0;
  value = Serial.read();
  if(value!= -1){    
      apc.write(value);   
      apc.println();    
  }
  else{
    if(apc.available()){
      txt=apc.read();
      Serial.print(txt);
    }  
  }
}
