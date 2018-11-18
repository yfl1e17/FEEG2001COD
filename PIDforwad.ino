#include <Shield2AMotor.h>



#include <Encoder.h> 

Shield2AMotor motor(SIGNED_MAGNITUDE);
volatile int n_Left_Motor = 0; // encoder values(degrees)
volatile int n_Right_Motor = 0;
int a = 0; //n reset at waypoint
int b = 0;
//pins
int pin_EN1 = 4; //motor pins locked anti-phase
int pin_Dir1 = 6;
int pin_EN2 = 5;
int pin_Dir2 = 7;


Encoder Left_Motor(2, 8); // encoder pins for library
Encoder Right_Motor(3, 9);



void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	pinMode(pin_EN1, OUTPUT);
	pinMode(pin_EN2, OUTPUT);
	pinMode(pin_Dir1, OUTPUT);
	pinMode(pin_Dir2, OUTPUT);
	delay(1000);


}
void print(int Setpoint) {
	Left_Motor.write(0);
	Right_Motor.write(0);
	int i = 3;

	while (i > 0) {
		int spd = 100;
		float Kp = 0.0;
		volatile int n_Left = -Left_Motor.read();
		volatile int n_Right = Right_Motor.read();
		int n_error = n_Left - n_Right;
		int Lv;
		int Rv;
		motor.control(Lv, Rv);

		if ((n_Left < Setpoint + 6) && (n_Left > Setpoint - 6)) {  //stop
			Lv = 0;
		}
		if ((n_Right < Setpoint + 6) && (n_Right > Setpoint - 6)) {  //stop
			Rv = 0;
		}
		if (n_Left < Setpoint - 5) { //fwd
			Lv = spd-(Kp*(n_error));
		}
		if (n_Right < Setpoint - 5) { //fwd
			Rv = spd + (Kp*(n_error));
		}
		if (n_Left > Setpoint + 5) { //bwd
			Lv = -spd + (Kp*(n_error));
		}
		if (n_Right > Setpoint + 5) { //bwd
			Rv = -spd - (Kp*(n_error));
		}


		String hs = "L " + String(n_Left) + " R " + String(n_Right) + "  " + String(i);
		Serial.println(hs);
		if ((Lv == 0) && (Rv == 0)) {
			i--;
		}
	}
	motor.control(0, 0);
}

void Forwardnoki(int Setpoint) {
	Left_Motor.write(0);
	Right_Motor.write(0);
	int t = 3;

	

	while (t > 0) {
		int spd = 80;
		float Kp = 2.2;//2.2
		volatile int n_Left = -Left_Motor.read();
		volatile int n_Right = Right_Motor.read();
		int n_error = n_Left - n_Right;
		int n_errorP = 0;
		int Lv;
		int Rv;
		//Lvc = spd - (Kp* (n_error)); // error = L - R
		//Rvc = spd + (Kp * (n_error));
		constrain(Lv, -100, 100);
		constrain(Rv, -100, 100);

		motor.control(Lv, Rv);
		//Serial.println(n_Left);

		if ((n_Left < Setpoint + 6) && (n_Left > Setpoint - 6)) {  //stop
			Lv = 0;
		}
		if ((n_Right < Setpoint + 6) && (n_Right > Setpoint - 6)) {  //stop
			Rv = 0;
		}
		if (n_Left < Setpoint - 5) { //fwd
			Lv = spd - (Kp* (n_error));
		}
		if (n_Right < Setpoint - 5) { //fwd
			Rv = spd + (Kp* (n_error));
		}
		if (n_Left > Setpoint + 5) { //bwd
			Lv = -spd + (Kp* (n_error));
		}
		if (n_Right > Setpoint + 5) { //bwd
			Rv = -spd - (Kp* (n_error));
		}
		int nlp = 0;
		int nrp = 0;
		String qs = "L " + String(n_Left-nlp) + " R " + String(n_Right-nrp) + "  E " + String(n_error);

		String hs = "L " + String(n_Left) + " R " + String(n_Right)+"  E "+String(n_error);
		Serial.println(hs);
		if (nlp != n_Left || nrp != n_Right) {
			nlp = n_Left;
			nrp = n_Right;
		}
		if ((Lv == 0) && (Rv == 0)) {
			t--;
		}
	}
	motor.control(0, 0);
}
void Forward(int Setpoint) {
	Left_Motor.write(0);
	Right_Motor.write(0);
	int t = 3;
	int n_errorP = 0;
	double I = 0;
	double Kp = 2.2; //2.2
	double Ki = 0.0;
	double Kd = 1;

	while (t > 0) {
		int spd = 80;
		
		volatile int n_Left = -Left_Motor.read();
		volatile int n_Right = Right_Motor.read();
		double n_error = n_Left - n_Right;
		I += (n_error);
		int Lv;
		int Rv;
		//Lvc = spd - (Kp* (n_error)); // error = L - R
		//Rvc = spd + (Kp * (n_error)); // +(Ki*I) +(Kd*(n_error-n_errorP))
		constrain(Lv, -100, 100);
		constrain(Rv, -100, 100);

		motor.control(Lv, Rv);
		//Serial.println(n_Left);

		if ((n_Left < Setpoint + 6) && (n_Left > Setpoint - 6)) {  //stop
			Lv = 0;
		}
		if ((n_Right < Setpoint + 6) && (n_Right > Setpoint - 6)) {  //stop
			Rv = 0;
		}
		if (n_Left < Setpoint - 5) { //fwd
			Lv = spd - (Kp* (n_error)+(Ki*I) + (Kd*(n_error - n_errorP)));
		}
		if (n_Right < Setpoint - 5) { //fwd
			Rv = spd + (Kp* (n_error)+(Ki*I) + (Kd*(n_error - n_errorP)));
		}
		if (n_Left > Setpoint + 5) { //bwd
			Lv = -spd + (Kp* (n_error)+(Ki*I) + (Kd*(n_error - n_errorP)));
		}
		if (n_Right > Setpoint + 5) { //bwd
			Rv = -spd - (Kp* (n_error)+(Ki*I) + (Kd*(n_error - n_errorP)));
		}
		n_errorP = n_error;

		//int nlp = 0;
		//int nrp = 0;
		//String qs = "L " + String(n_Left - nlp) + " R " + String(n_Right - nrp) + "  E " + String(n_error);

		String hs = "L " + String(n_Left) + " R " + String(n_Right) + "  E " + String(n_error);
		Serial.println(hs);
		/*if (nlp != n_Left || nrp != n_Right) {
			nlp = n_Left;
			nrp = n_Right;
		}*/
		if ((Lv == 0) && (Rv == 0)) {
			t--;
		}

		motor.control(0, 0);
	}
}
	void TurnCWnokd(int Setpoint) {
		Left_Motor.write(0);
		Right_Motor.write(0);
		int t = 2;



		while (t > 0) {
			int spd = 80;
			float Kp = 2.2;
			volatile int n_Left = -Left_Motor.read();
			volatile int n_Right = -Right_Motor.read();
			int n_error = n_Left - n_Right;
			int n_errorP = 0;
			int Lv;
			int Rv;
			//Lvc = spd - (Kp* (n_error)); // error = L - R
			//Rvc = spd + (Kp * (n_error));
			constrain(Lv, -100, 100);
			constrain(Rv, -100, 100);

			motor.control(Lv, -Rv);
			//Serial.println(n_Left);

			if ((n_Left < Setpoint + 3) && (n_Left > Setpoint - 3)) {  //stop
				Lv = 0;
			}
			if ((n_Right < Setpoint + 2) && (n_Right > Setpoint - 2)) {  //stop
				Rv = 0;
			}
			if (n_Left < Setpoint - 2) { //fwd
				Lv = spd - (Kp* (n_error));
			}
			if (n_Right < Setpoint - 2) { //fwd
				Rv = spd + (Kp* (n_error));
			}
			if (n_Left > Setpoint + 2) { //bwd
				Lv = -spd + (Kp* (n_error));
			}
			if (n_Right > Setpoint + 2) { //bwd
				Rv = -spd - (Kp* (n_error));
			}
			int nlp = 0;
			int nrp = 0;
			String qs = "L " + String(n_Left - nlp) + " R " + String(n_Right - nrp) + "  E " + String(n_error);

			String hs = "L " + String(n_Left) + " R " + String(n_Right) + "  E " + String(n_error);
			Serial.println(hs);
			if (nlp != n_Left || nrp != n_Right) {
				nlp = n_Left;
				nrp = n_Right;
			}
			if ((Lv == 0) && (Rv == 0)) {
				t--;
			}
		}
		motor.control(0, 0);
	}

	void TurnCW(int Setpoint) {
		Left_Motor.write(0);
		Right_Motor.write(0);
		int t = 2;
		int n_errorP = 0;


		while (t > 0) {
			int spd = 60;
			float Kp = 0.7;
			float Kd = 0.4;
			volatile int n_Left = -Left_Motor.read();
			volatile int n_Right = -Right_Motor.read();
			int n_error = n_Left - n_Right;

			int Lv;
			int Rv;
			//Lvc = spd - (Kp* (n_error)); // error = L - R
			//Rvc = spd + (Kp * (n_error));
			constrain(Lv, -100, 100);
			constrain(Rv, -100, 100);
			//+(Kd*(n_error-n_errorP))
			motor.control(Lv, -Rv);
			//String rf = String(n_errorP)+" " + String(n_error);
			//Serial.println(rf);

			if ((n_Left < Setpoint + 3) && (n_Left > Setpoint - 3)) {  //stop
				Lv = 0;
			}
			if ((n_Right < Setpoint + 3) && (n_Right > Setpoint - 3)) {  //stop
				Rv = 0;
			}
			if (n_Left < Setpoint - 2) { //fwd
				Lv = spd - (Kp* (n_error)+(Kd*(n_error - n_errorP)));
			}
			if (n_Right < Setpoint - 2) { //fwd
				Rv = spd + (Kp* (n_error)+(Kd*(n_error - n_errorP)));
			}
			if (n_Left > Setpoint + 2) { //bwd
				Lv = -spd + (Kp* (n_error)+(Kd*(n_error - n_errorP)));
			}
			if (n_Right > Setpoint + 2) { //bwd
				Rv = -spd - (Kp* (n_error)+(Kd*(n_error - n_errorP)));
			}
			if (n_errorP != n_error) {
				n_errorP = n_error;
			}
			//int nlp = 0;
			//int nrp = 0;
			//String qs = "L " + String(n_Left - nlp) + " R " + String(n_Right - nrp) + "  E " + String(n_error);

			String hs = "L " + String(n_Left) + " R " + String(n_Right) + "  E " + String(n_error);
			Serial.println(hs);
			/*if (nlp != n_Left || nrp != n_Right) {
				nlp = n_Left;
				nrp = n_Right;
			}*/
			if ((Lv == 0) && (Rv == 0)) {
				t--;
			}
		}
		motor.control(0, 0);
	}


void loop() {
	//motor.control(100,0);
	Forward(1000);
	/*Forward(400);
	TurnCW(205);
	Forward(400);
	TurnCW(205);
	Forward(400);
	TurnCW(205);
	Forward(400);
	TurnCW(205);*/

	
	
	//print(180);
	//analogWrite(pin_EN2,250);
   //digitalWrite(pin_Dir2,LOW);
   //motor.control(100,0);
   //temp();
	while (1);
	
	


}