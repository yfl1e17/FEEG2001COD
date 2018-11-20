#include <Shield2AMotor.h>
#include <Encoder.h> 
Shield2AMotor motor(SIGNED_MAGNITUDE);
int pin_EN1 = 4; //motor pins locked anti-phase
int pin_Dir1 = 6;
int pin_EN2 = 5;
int pin_Dir2 = 7;
Encoder Left_Motor(2, 8); // encoder pins for library
Encoder Right_Motor(3, 9);


void setup()
{
	Serial.begin(9600);
	pinMode(pin_EN1, OUTPUT);
	pinMode(pin_EN2, OUTPUT);
	pinMode(pin_Dir1, OUTPUT);
	pinMode(pin_Dir2, OUTPUT);
	delay(1000);

}


void Forward(int Setpoint) { 

	Left_Motor.write(0);
	Right_Motor.write(0);
	int t = 3;
	int tol = 5;
	int n_errorP = 0;
	int eLp = 0; // previous setpoint error
	int eRp = 0;
	// L-R
	double Kp = 0.3;//0.4,1.0.0.65 ,0.4// (0.65,0.3)(0.65,0.4)
	double Ki = 0.0;
	double Kd = 0.2;//,0.9,0.4
  // encoder - Setpoint
	double kp = 0.5; //0.5
	double kint = 0.005;//0.0001
	double kd = 0.1;//0.1

	while (t>0) { //t >0
		int spd = 80;

		volatile int n_Left = -Left_Motor.read();
		volatile int n_Right = Right_Motor.read();
		int n_error = n_Left - n_Right;
		int eL = n_Left - Setpoint;
		int eR = n_Right - Setpoint;
		int ieL;
		int ieR;
		int Ie = 0;
		ieL += eL;
		ieR += eR;
		//ieL = constrain(ieL, -200, 200);
		//ieR = constrain(ieR, -200, 200);
		Ie += n_error;
		//if ((abs(eL) < 50) || (abs(eR) < 50)) n_error = 0;
		if (ieL > 1000) ieL = 1000;
		else if (ieL < -1000) ieL = -1000;
		if (ieR > 1000) ieR = 1000;
		else if (ieR< -1000) ieR = -1000;
		if (eL < -100) ieL = 0;
		if (eR < -100) ieL = 0;
		if (eL > 100) ieL = 0;
		if (eR > 100) ieL = 0;
		int Lv;
		int Rv;
		int Lspd;
		int Rspd;
		Lv = constrain(Lv, -100, 100);
		Rv = constrain(Rv, -100, 100);
		//Serial.println(n_error);
		//String rv = "L " + String(eL) + "  R " + String(eR) + " Spd  " + String(Lv) + "  " + String(Rv) + " b " + String(b);
		//String rv = "n_error" + String(n_error)+ "Lspd" + String(Lspd) + "Rspd" + String(Rspd) + "  I " + String(ieL) + " " + String(ieR);
		//Serial.println(rv);
		motor.control(Lv, Rv);
		//Serial.println(n_Left); //Lv = -(Kp* (eL)+(Ki*IeL) + (Kd*(eL - eLp)));
		Lspd = -(kp* (eL)+(kint*ieL) + (kd*(eL - eLp)));
		Rspd = -(kp* (eR)+(kint*ieR) + (kd*(eR - eRp)));
		Lspd = constrain(Lspd, -80, 80);
		Rspd = constrain(Rspd, -80, 80);
		Lv = Lspd - (Kp* (n_error)+(Ki*Ie) + (Kd*(n_error - n_errorP)));
		Rv = Rspd + (Kp* (n_error)+(Ki*Ie) + (Kd*(n_error - n_errorP)));

		n_errorP = n_error;
		eLp = eL;
		eRp = eR;
		//String qs = "L " + String(n_Left - nlp) + " R " + String(n_Right - nrp) + "  E " + String(n_error);
		String hs = "L " + String(n_Left) + " R " + String(n_Right) + "  E " + String(n_error) + "  " + String(Lv);
		//Serial.println(hs);


		if ((abs(eL)<tol)&& (abs(eR) < tol)) {
			t--;
		}
		/*if (((n_Left <Setpoint+tol) && (n_Left > Setpoint - tol))) {
			t--;
		}*/
		int b=0;
		if ((eL == eLp)&& (eR == eRp)) {
			b++;
		}
		if (b > 1000) {
			motor.control(100, 100);
			}

	}
	motor.control(0, 0);
	delay(800);
}


// Add the main program code into the continuous loop() function
void loop()
{
	Forward(500);
	while (1);
}
