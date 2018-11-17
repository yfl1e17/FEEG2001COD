#include<Encoder.h>
#include<Shield2AMotor.h>

int n_Left_Motor = 0;
int n_Right_Motor = 0;
int i = 0;
float avg = 0.0;
//int j = 0;


Encoder Left_Motor(2,9);
Encoder Right_Motor(3,8);
Shield2AMotor motor(SIGNED_MAGNITUDE);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000);
  
//  Move_Straight(720);
 
 // Move_Straight(720);
}


void Stop(){
  motor.control(0,0);
  

  //j++;
  //i = constrain(j, 0, i++);
  
  //Serial.println(i);

  //delay(5000); 

  //attachInterrupt(digitalPinToInterrupt(2), loop, HIGH);
  //attachInterrupt(digitalPinToInterrupt(3), loop, LOW);  
}

void Move_Arc(int n){
  motor.control(0,70);

  Left_Motor.write(0);
  Right_Motor.write(0);
  
  while (n_Right_Motor <= n){
    int n_Left = Left_Motor.read();
    int n_Right = Right_Motor.read();
  
    if (n_Left != n_Left_Motor||n_Right != n_Right_Motor){
      //Serial.println(n_Left, n_Right);
      n_Left_Motor = abs(n_Left);
      n_Right_Motor = n_Right;
    }

    if (n_Right_Motor == n){
      break;
    }
    
    Serial.println(n_Left_Motor);
    Serial.println(n_Right_Motor);
  }
  
    attachInterrupt(digitalPinToInterrupt(2), Stop, HIGH);
    attachInterrupt(digitalPinToInterrupt(3), Stop, LOW);
}

void Move_Straight(int n){

  Left_Motor.write(0);
  Right_Motor.write(0);
  
 while (n_Right_Motor <= 720){
     motor.control(-70,70);
    int n_Left = Left_Motor.read();
    int n_Right = Right_Motor.read();
  
    if (n_Left != n_Left_Motor||n_Right != n_Right_Motor){
      //Serial.println(n_Left, n_Right);
      n_Left_Motor = (n_Left);
      n_Right_Motor = n_Right;
    }

    avg = n_Right_Motor + n_Left_Motor;
    
    if (n_Right_Motor >= 720){
      break;
    }

    else if (n_Left_Motor >= 720){
      break;
    }

    String SL = "L " + String(n_Left_Motor) + "  R " + String(n_Right_Motor);
    Serial.println(SL);
    Serial.println(avg);
    //Serial.println(n_Left_Motor);
    //Serial.println(n_Right_Motor);
  }
    motor.control(0,0);
    //return;
    //Stop();
    //String S = "Lalala";
    //Serial.println(S);
    
    //attachInterrupt(2, Stop, CHANGE);
    //attachInterrupt(3, Stop, CHANGE);
  motor.control(100,0);
 

}
void loop() {
  // put your main code here, to run repeatedly:

  
 
Move_Straight(720);

  int a = 0;
  int b = 0;

  if (n_Right_Motor != 0){
    n_Right_Motor = a;
    n_Left_Motor = b;
  }
Move_Straight(720);
 motor.control(0,0);
    //return;
    //Stop();
    //String S = "Lalala";
    //Serial.println(S);
    
    //attachInterrupt(2, Stop, CHANGE);
    //attachInterrupt(3, Stop, CHANGE);
while(1);
}
