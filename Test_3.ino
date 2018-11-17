#include<Encoder.h>
#include<Shield2AMotor.h>

int n_Left_Motor = 0;
int n_Right_Motor = 0;
int a = 0;
int b = 0;


Encoder Left_Motor(2,9);
Encoder Right_Motor(3,8);
Shield2AMotor motor(SIGNED_MAGNITUDE);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000);

  Move_Straight(720, 70);
  delay(2000);
  Move_Straight(720, 70);
}


void Stop(){
  motor.control(0,0);
}

void Move_Straight(int n, int spd){

  Left_Motor.write(0);
  Right_Motor.write(0);
  
  while (n_Right_Motor <= 720){
    motor.control(-spd, spd);
    int n_Left = Left_Motor.read();
    int n_Right = Right_Motor.read();
  
    if (n_Left != n_Left_Motor||n_Right != n_Right_Motor){
      n_Left_Motor = (n_Left);
      n_Right_Motor = n_Right;
    }
    
    if (n_Right_Motor >= 720){
      break;
    }

    else if (n_Left_Motor >= 720){
      break;
    }

    String SL = "L " + String(n_Left_Motor) + "  R " + String(n_Right_Motor);
    Serial.println(SL);
  }
    motor.control(0,0);
    delay(1000);

    if (n_Left_Motor != 0 || n_Right_Motor != 0){
      n_Left_Motor = a;
      n_Right_Motor = b;
    }
}

void Move_Arc(){
}

void loop() {
  // put your main code here, to run repeatedly:
}
