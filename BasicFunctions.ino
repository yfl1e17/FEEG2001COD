

#include <Encoder.h> /* Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.  */

volatile int n_Left_Motor = 0; // encoder values(degrees)
volatile int n_Right_Motor = 0;
int a = 0; //n reset at waypoint
int b = 0;
//pins
int pin_EN1 = 4; //motor pins locked anti-phase
int pin_EN2 = 7;
int pin_Dir1 = 5;
int pin_Dir2 = 6;

Encoder Left_Motor(2,8); // encoder pins for library
Encoder Right_Motor(3,9); 



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pin_EN1, OUTPUT);
  pinMode(pin_EN2, OUTPUT);
  pinMode(pin_Dir1, OUTPUT);
  pinMode(pin_Dir2, OUTPUT);
  delay(2000);
  

  Move_Forward(458);
  delay(200);
  Turn_CW(60);
  delay(200);
  Move_Forward(346);
  delay(200);
  Turn_CCW(60);
  delay(200);
  Move_Forward(671);
  delay(200);
  Turn_CCW(60);
  delay(200);
  Move_Arc_1();
  delay(200);
  Turn_CW(60);
  delay(200);
  Move_Forward(887);
  delay(200);
  Turn_CCW(60);
  delay(200);
  Move_Forward(540);
  delay(200);
  Turn_CCW(60);
  delay(200);
  Move_Forward(546);
  delay(200);
  Turn_CCW(60);
  delay(200);
  Move_Forward(540);
  delay(200);
  Turn_CCW(27);
  delay(200);
  Move_Forward(838);
  delay(200);
  Turn_CCW(27);
  delay(200);
  Move_Backward(240);
  delay(200);
  Turn_CCW(60);
  delay(200);
  Move_Arc_2();
  delay(200);
  Turn_CW(25);
  delay(200);
  Move_Backward(475);
  delay(200);
  Move_Backward(574);
  delay(200);
  
}




void Move_Forward(int n) {

	Left_Motor.write(0);
	Right_Motor.write(0);

	while (n_Right_Motor <= n) {
		digitalWrite(pin_Dir1, HIGH);
		digitalWrite(pin_Dir2, LOW);
		analogWrite(pin_EN1, 220);
		analogWrite(pin_EN2, 220);
		volatile int n_Left = Left_Motor.read();
		volatile int n_Right = Right_Motor.read();

		if (n_Left != n_Left_Motor || n_Right != n_Right_Motor) {
			n_Left_Motor = n_Left;
			n_Right_Motor = n_Right;
		}

		if (abs(n_Right_Motor) >= n) {
			break;
		}

		else if (abs(n_Left_Motor) >= n) {
			break;
		}

		String SL = "L " + String(n_Left_Motor) + "  R " + String(n_Right_Motor);
		Serial.println(SL);
	}
	digitalWrite(pin_EN1,LOW);
	digitalWrite(pin_EN2,LOW);
	
	if (n_Left_Motor != 0 || n_Right_Motor != 0) {
		n_Left_Motor = a;
		n_Right_Motor = b;
	}
}


void Move_Backward(int n) {

  Left_Motor.write(0);
  Right_Motor.write(0);

  while (n_Right_Motor <= n) {
    digitalWrite(pin_Dir1, LOW);
    digitalWrite(pin_Dir2, HIGH);
    analogWrite(pin_EN1, 220);
    analogWrite(pin_EN2, 220);
    volatile int n_Left = Left_Motor.read();
    volatile int n_Right = Right_Motor.read();

    if (n_Left != n_Left_Motor || n_Right != n_Right_Motor) {
      n_Left_Motor = n_Left;
      n_Right_Motor = n_Right;
    }

    if (abs(n_Right_Motor) >= n) {
      break;
    }

    else if (abs(n_Left_Motor) >= n) {
      break;
    }

    String SL = "L " + String(n_Left_Motor) + "  R " + String(n_Right_Motor);
    Serial.println(SL);
  }
  digitalWrite(pin_EN1,LOW);
  digitalWrite(pin_EN2,LOW);
  
  if (n_Left_Motor != 0 || n_Right_Motor != 0) {
    n_Left_Motor = a;
    n_Right_Motor = b;
  }
}


void Turn_CW(int theta){

  float arc = 185 * 3.142;
  float distance = arc * theta / 360;
  float Cir_Wheels = 85 * 3.142;
  
  int clicks = distance / Cir_Wheels * 360;
  
  Left_Motor.write(0);
  Right_Motor.write(0);

  while (n_Right_Motor <= clicks) {
    digitalWrite(pin_Dir1, HIGH);
    digitalWrite(pin_Dir2, HIGH);
    analogWrite(pin_EN1, 220);
    analogWrite(pin_EN2, 220);
    volatile int n_Left = Left_Motor.read();
    volatile int n_Right = Right_Motor.read();

    if (n_Left != n_Left_Motor || n_Right != n_Right_Motor) {
      n_Left_Motor = n_Left;
      n_Right_Motor = n_Right;
    }

    if (abs(n_Right_Motor) >= clicks) {
      break;
    }

    else if (abs(n_Left_Motor) >= clicks) {
      break;
    }

    String SL = "L " + String(n_Left_Motor) + "  R " + String(n_Right_Motor);
    Serial.println(SL);
  }
  digitalWrite(pin_EN1,LOW);
  digitalWrite(pin_EN2, LOW);
  
  if (n_Left_Motor != 0 || n_Right_Motor != 0) {
    n_Left_Motor = a;
    n_Right_Motor = b;
  }
}


void Turn_CCW(int theta){

  float arc = 185 * 3.142;
  float distance = arc * theta / 360;
  float Cir_Wheels = 85 * 3.142;
  
  int clicks = distance / Cir_Wheels * 360;
  
  Left_Motor.write(0);
  Right_Motor.write(0);

  while (n_Right_Motor <= clicks) {
    digitalWrite(pin_Dir1, LOW);
    digitalWrite(pin_Dir2, LOW);
    analogWrite(pin_EN1, 220);
    analogWrite(pin_EN2, 220);
    volatile int n_Left = Left_Motor.read();
    volatile int n_Right = Right_Motor.read();

    if (n_Left != n_Left_Motor || n_Right != n_Right_Motor) {
      n_Left_Motor = n_Left;
      n_Right_Motor = n_Right;
    }

    if (abs(n_Right_Motor) >= clicks) {
      break;
    }

    else if (abs(n_Left_Motor) >= clicks) {
      break;
    }

    String SL = "L " + String(n_Left_Motor) + "  R " + String(n_Right_Motor);
    Serial.println(SL);
  }
  digitalWrite(pin_EN1,LOW);
  digitalWrite(pin_EN2, LOW);
  
  if (n_Left_Motor != 0 || n_Right_Motor != 0) {
    n_Left_Motor = a;
    n_Right_Motor = b;
  }
}


void Turn_Fixed(int theta){

  float arc = 370 * 3.142;
  float distance = arc * theta / 360;
  float Cir_Wheels = 85 * 3.142;
  
  int clicks = distance / Cir_Wheels * 360;
  
  Left_Motor.write(0);
  Right_Motor.write(0);

  while (n_Right_Motor <= clicks) {
    digitalWrite(pin_Dir1, HIGH);
    digitalWrite(pin_Dir2, HIGH);
    analogWrite(pin_EN1, 220);
    analogWrite(pin_EN2, 0);
    volatile int n_Left = Left_Motor.read();
    volatile int n_Right = Right_Motor.read();

    if (n_Left != n_Left_Motor || n_Right != n_Right_Motor) {
      n_Left_Motor = n_Left;
      n_Right_Motor = n_Right;
    }

    if (abs(n_Right_Motor) >= clicks) {
      break;
    }

    else if (abs(n_Left_Motor) >= clicks) {
      break;
    }

    String SL = "L " + String(n_Left_Motor) + "  R " + String(n_Right_Motor);
    Serial.println(SL);
  }
  digitalWrite(pin_EN1,LOW);
  digitalWrite(pin_EN2, LOW);
  
  if (n_Left_Motor != 0 || n_Right_Motor != 0) {
    n_Left_Motor = a;
    n_Right_Motor = b;
  }
}


void Move_Arc_1(){

  Move_Forward(75);
  delay(500);
  Turn_Fixed(70);
  delay(500);
  Move_Forward(75);
}

void Move_Arc_2(){

  Move_Forward(75);
  delay(500);
  Turn_Fixed(70);
  delay(500);
  Move_Forward(140);
  delay(500);
  Turn_Fixed(70);
  delay(500);
  Move_Forward(140);
  delay(500);
  Turn_Fixed(70);
  delay(500);
  Move_Forward(75);
}

void loop() {
//Move_Forward(283);
    //while(1);
}
