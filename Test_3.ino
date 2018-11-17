#include<Encoder.h>
#include<Shield2AMotor.h>

int n_Left_Motor = 0;
int n_Right_Motor = 0;
int i = 0;
int j = 0;


Encoder Left_Motor(2,7);
Encoder Right_Motor(3,8);
Shield2AMotor motor(SIGNED_MAGNITUDE);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000);

  //Move_Straight(720);
}


void Stop(){
  motor.control(0,0);

  j++;
  i = constrain(j, 0, i++);
  
  Serial.println(i);

  delay(5000); 

  attachInterrupt(digitalPinToInterrupt(2), loop, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), loop, LOW);  
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
      n_Left_Motor = n_Left;
      n_Right_Motor = n_Right;
    }

    if (n_Right_Motor == n){
      break;
    }
    
    //Serial.println(n_Left_Motor);
    Serial.println(n_Right_Motor);
  }
  
    attachInterrupt(digitalPinToInterrupt(2), Stop, HIGH);
    attachInterrupt(digitalPinToInterrupt(3), Stop, LOW);
}

void Move_Straight(int n){
  
  motor.control(-70,70);

  Left_Motor.write(0);
  Right_Motor.write(0);
  
  while (n_Right_Motor <= n){
    int n_Left = Left_Motor.read();
    int n_Right = Right_Motor.read();
  
    if (n_Left != n_Left_Motor||n_Right != n_Right_Motor){
      //Serial.println(n_Left, n_Right);
      n_Left_Motor = n_Left;
      n_Right_Motor = n_Right;
    }

    if (n_Right_Motor == n){
      break;
    }
    
    //Serial.println(n_Left_Motor);
    Serial.println(n_Right_Motor);
  }
  
    attachInterrupt(digitalPinToInterrupt(2), Stop, HIGH);
    attachInterrupt(digitalPinToInterrupt(3), Stop, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (i == 0){
      Move_Straight(720); 
  }
  //if (i == 1){
      //Move_Arc(500);
  //}
  //else{
      //motor.control(0,0);
  //}
}
