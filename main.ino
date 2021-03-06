
/* Group F
insert names here

*/
// Libraries :
#include <Shield2AMotor.h> /*Cytron Dual Channel 2A Motor Driver Shield
 *https://github.com/CytronTechnologies/Cytron-Shield2AMotor */

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
#include <AutoPID.h> /*
* /**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/


//Pins :
Encoder LEnc(2,7);
Encoder REnc(3,4);
Shield2AMotor motor(SIGNED_MAGNITUDE);
//variables :
//double Setpoint, Input, Output;
//double Kp=2, Ki=5, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}
long LEncRO = -34;
long REncRO = -12;

void forward() {
//  motor.control(60,60)
  
}
void loop() {
  // put your main code here, to run repeatedly:
  long LEncR = LEnc.read();
  if (LEncR != LEncRO) {
    LEncRO = LEncR;
    String SL1 = "LeftM";
    String SL2 = SL1 + (LEncRO);
    Serial.println(SL2);
  }
  long REncR = REnc.read();
  if (REncR != REncRO) {
    REncRO = REncR;
    String SR1 = "RightM";
    String SR2 = SR1 + (REncR);
    Serial.println(SR2);
    motor.control(100,100);
    delay(1000);
    motor.control(0,0);
    delay(500);
    Serial.println(SR2);
    motor.control(-100,-100);
    delay(1000);
    motor.control(0,0);
    delay(500);
    
  }
}
