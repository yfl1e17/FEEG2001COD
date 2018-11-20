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

void Forward(int Setpoint) { //dr

	Left_Motor.write(0);
	Right_Motor.write(0);
	int t = 500;
	int n_errorP = 0;
	double eLp = 0;
	double eRp = 0;
	// L-R
	double Kp = 0.5;//0.4,1.0.0.65 ,0.4
	double Ki = 0.000000;
	double Kd = 0.3;//,0.9,0.4
  // encoder - Setpoint
  double kp = 0.5; //0.5
  double kint = 0.000000;//0.0001
  double kd = 0.1;//0.1

	while (t>0) {
		int spd = 80;

		volatile int n_Left = -Left_Motor.read();
		volatile int n_Right = Right_Motor.read();
		double n_error = n_Left - n_Right;
		double eL = n_Left - Setpoint;
		double eR = n_Right - Setpoint;
		double ieL;
		double ieR;
		double Ie;
		int ieLw, ieRw;
		ieL += eL;
		ieR += eR;
		//ieL = constrain(ieLw, -200, 200);
		//ieR = constrain(ieRw, -200, 200);
		Ie += n_error;
		//if ((abs(eL) < 50) || (abs(eR) < 50)) n_error = 0;
		//if (ieL > 1500) ieL = 1500;
		//else if (ieL < -1000) ieL = -1000;
		//if (ieR > 1500) ieR = 1500;
		//else if (ieR< -1000) ieR = -1000;
		//if (eL < -100) ieL = 0;
		//if (eR < -100) ieL = 0;
		int Lv;
		int Rv;
		int Lspd;
		int Rspd;
		int tol = 3;//tolerance
		
		Lv = constrain(Lv, -100, 100);
		Rv = constrain(Rv, -100, 100);
  //Serial.println(n_error);
   String rv = "L "+String(Lv)+"  R "+String(Rv) +"  I " + String(eL)+"  "+String(eR);
		//String rv = "n_error" + String(n_error)+ "Lspd" + String(Lspd) + "Rspd" + String(Rspd) + "  I " + String(ieL) + " " + String(ieR);
    Serial.println(t);
		motor.control(Lv, Rv);
		//Serial.println(n_Left); //Lv = -(Kp* (eL)+(Ki*IeL) + (Kd*(eL - eLp)));
		Lspd = -(kp* (eL)+(kint*ieL) + (kd*(eL - eLp)));
		Rspd = -(kp* (eR)+(kint*ieR) + (kd*(eR - eRp)));
		Lspd = constrain(Lspd,-80,80);
		Rspd = constrain(Rspd,-80,80);
		Lv = Lspd - (Kp* (n_error)+(Ki*Ie) + (Kd*(n_error - n_errorP)));
		Rv = Rspd + (Kp* (n_error)+(Ki*Ie) + (Kd*(n_error - n_errorP)));
		
		n_errorP = n_error;
		eLp = eL;
		eRp = eR;
		

		//int nlp = 0;
		//int nrp = 0;
		//String qs = "L " + String(n_Left - nlp) + " R " + String(n_Right - nrp) + "  E " + String(n_error);

		String hs = "L " + String(n_Left) + " R " + String(n_Right) + "  E " + String(n_error)+"  "+String(Lv);
		//Serial.println(hs);
		/*if (nlp != n_Left || nrp != n_Right) {
		  nlp = n_Left;
		  nrp = n_Right;
		}*/
  
		if ((n_error-n_errorP)==0) {
			t--;
		}

		
	}
	motor.control(0, 0);
	delay(800);
}
void TurnCW(int Setpoint) { //154 = 90degree

	Left_Motor.write(0);
	Right_Motor.write(0);
	int t = 100;
	int n_errorP = 0;
	double eLp = 0;
	double eRp = 0;
	// L-R
	double Kp = 0.4;//0.4,1.0.0.65
	double Ki = 0.000000;
	double Kd = 0.3;//,0.9,0.4
  // encoder - Setpoint
	double kp = 0.5; //0.5
	double kint = 0.00;//0.0001
	double kd = 0.1;//0.1

	while (t>0) {
		int spd = 80;

		volatile int n_Left = -Left_Motor.read();
		volatile int n_Right = -Right_Motor.read();
		double n_error = n_Left - n_Right;
		double eL = n_Left - Setpoint;
		double eR = n_Right - Setpoint;
		double ieL;
		double ieR;
		double Ie;
		int ieLw, ieRw;
		ieL += eL;
		ieR += eR;
		//ieL = constrain(ieLw, -200, 200);
		//ieR = constrain(ieRw, -200, 200);
		Ie += n_error;
		//if ((abs(eL) < 50) || (abs(eR) < 50)) n_error = 0;
		//if (ieL > 1500) ieL = 1500;
		//else if (ieL < -1000) ieL = -1000;
		//if (ieR > 1500) ieR = 1500;
		//else if (ieR< -1000) ieR = -1000;
		//if (eL < -100) ieL = 0;
		//if (eR < -100) ieL = 0;
		int Lv;
		int Rv;
		int Lspd;
		int Rspd;
		int tol = 3;//tolerance

		Lv = constrain(Lv, -100, 100);
		Rv = constrain(Rv, -100, 100);
		//Serial.println(n_error);
		String rv = "L " + String(Lv) + "  R " + String(Rv) + "  I " + String(eL) + "  " + String(eR);
		//String rv = "n_error" + String(n_error)+ "Lspd" + String(Lspd) + "Rspd" + String(Rspd) + "  I " + String(ieL) + " " + String(ieR);
		Serial.println(rv);
		motor.control(Lv, -Rv);
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


		//int nlp = 0;
		//int nrp = 0;
		//String qs = "L " + String(n_Left - nlp) + " R " + String(n_Right - nrp) + "  E " + String(n_error);

		String hs = "L " + String(n_Left) + " R " + String(n_Right) + "  E " + String(n_error) + "  " + String(Lv);
		//Serial.println(hs);
		/*if (nlp != n_Left || nrp != n_Right) {
		  nlp = n_Left;
		  nrp = n_Right;
		}*/

		if ((n_error - n_errorP) == 0) {
			t--;
		}


	}
	motor.control(0, 0);
	delay(800);
}
void TurnCCW(int Setpoint) { //154 = 90degree

	Left_Motor.write(0);
	Right_Motor.write(0);
	int t = 100;
	int n_errorP = 0;
	double eLp = 0;
	double eRp = 0;
	// L-R
	double Kp = 0.4;//0.4,1.0.0.65
	double Ki = 0.000000;
	double Kd = 0.3;//,0.9,0.4
  // encoder - Setpoint
	double kp = 0.5; //0.5
	double kint = 0.00;//0.0001
	double kd = 0.1;//0.1

	while (t>0) {
		int spd = 80;

		volatile int n_Left = Left_Motor.read();
		volatile int n_Right = Right_Motor.read();
		double n_error = n_Left - n_Right;
		double eL = n_Left - Setpoint;
		double eR = n_Right - Setpoint;
		double ieL;
		double ieR;
		double Ie;
		int ieLw, ieRw;
		ieL += eL;
		ieR += eR;
		//ieL = constrain(ieLw, -200, 200);
		//ieR = constrain(ieRw, -200, 200);
		Ie += n_error;
		//if ((abs(eL) < 50) || (abs(eR) < 50)) n_error = 0;
		//if (ieL > 1500) ieL = 1500;
		//else if (ieL < -1000) ieL = -1000;
		//if (ieR > 1500) ieR = 1500;
		//else if (ieR< -1000) ieR = -1000;
		//if (eL < -100) ieL = 0;
		//if (eR < -100) ieL = 0;
		int Lv;
		int Rv;
		int Lspd;
		int Rspd;
		int tol = 3;//tolerance

		Lv = constrain(Lv, -100, 100);
		Rv = constrain(Rv, -100, 100);
		//Serial.println(n_error);
		String rv = "L " + String(Lv) + "  R " + String(Rv) + "  I " + String(eL) + "  " + String(eR);
		//String rv = "n_error" + String(n_error)+ "Lspd" + String(Lspd) + "Rspd" + String(Rspd) + "  I " + String(ieL) + " " + String(ieR);
		Serial.println(rv);
		motor.control(-Lv, Rv);
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


		//int nlp = 0;
		//int nrp = 0;
		//String qs = "L " + String(n_Left - nlp) + " R " + String(n_Right - nrp) + "  E " + String(n_error);

		String hs = "L " + String(n_Left) + " R " + String(n_Right) + "  E " + String(n_error) + "  " + String(Lv);
		//Serial.println(hs);
		/*if (nlp != n_Left || nrp != n_Right) {
		  nlp = n_Left;
		  nrp = n_Right;
		}*/

		if ((n_error - n_errorP) == 0) {
			t--;
		}


	}
	motor.control(0, 0);
	delay(800);
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
    //Serial.println(hs);
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
void Forwardnopid(int Setpoint) {
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

  void TurnCWnopid(int Setpoint) {
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
  

void Arc(int r, int theta){  //based on Forwardnoki

  Left_Motor.write(0);
  Right_Motor.write(0);
  int t = 3;
  double I = 0;
  double Kp = 0; //2.2
  double Ki = 0.0;
  double Kd = 0.0;
  
  float Inner_Radius = r - 92.5;
  float Outer_Radius = r + 92.5;
  float Rv_Lv = Inner_Radius / Outer_Radius;
  float Cir_Wheel = 267;

  int Setpoint_Rv = Inner_Radius * 6.284 * theta / 360; // 137.5 R<L
  int Setpoint_Lv = Outer_Radius * 6.284 * theta / 360;//  428 r3.10
  

  while (1) {
    int spd = 90;
    float Kp = 2.2;//2.2
    volatile int n_Left = -Left_Motor.read();
    volatile int n_Right = Right_Motor.read();
    int n_error = n_Left - (Rv_Lv)*n_Right; //1.0 : 3.1
    int n_errorP = 0;
    
	int Lv;
	int Rv;
    Serial.println(Rv);
    //int Lv;
    //int Rv;
    //Lvc = spd - (Kp* (n_error)); // error = L - R
    //Rvc = spd + (Kp * (n_error));
    constrain(Lv, -100, 100);
    constrain(Rv, -100, 100);

    motor.control(Lv, Rv);
    //Serial.println(n_Left);

    if ((n_Left < Setpoint_Lv + 6) && (n_Left > Setpoint_Lv - 6)) {  //stop
      Lv = 0;
    }
    if ((n_Right < Setpoint_Rv + 6) && (n_Right > Setpoint_Rv - 6)) {  //stop
      Rv = 0;
    }
    if (n_Left < Setpoint_Lv - 5) { //fwd
      Lv = spd - (Kp* (n_error)+(Ki*I) + (Kd*(n_error - n_errorP)));
    }
    if (n_Right < Setpoint_Rv - 5) { //fwd
      Rv = (Rv_Lv)*(spd + (Kp* (n_error)+(Ki*I) + (Kd*(n_error - n_errorP))));
    }
    if (n_Left > Setpoint_Lv + 5) { //bwd
      Lv = -spd + (Kp* (n_error)+(Ki*I) + (Kd*(n_error - n_errorP)));
    }
    if (n_Right > Setpoint_Rv + 5) { //bwd
      Rv = -(Rv_Lv)*(spd + (Kp* (n_error)+(Ki*I) + (Kd*(n_error - n_errorP))));
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
void power() {
	for (int p = 40; p < 70;) {
		Serial.println(p);
		motor.control(p, p);
		delay(500);
		motor.control(0,0);
		delay(1000);
		p = (p)+5;

	}
}
void timer(int p) {
	Serial.println("Start");
	Right_Motor.write(0);
	int n_Right = Right_Motor.read();
	int r = 1;
	while (r > 0) {
		volatile int n_Right = Right_Motor.read();
		Serial.println(n_Right);
		if (n_Right >= 263) {
			r--;
		}
		motor.control(p, p);
	}
	Serial.println("End");
	motor.control(0, 0);

}
void length(int p) {
	Serial.println("Start");
	int h = 1;
	Left_Motor.write(0);
	int n_Left = Left_Motor.read();
		while (h > 0) {
			int time = millis();
			motor.control(p, p);
			if (time >= 894) h--;
	}
	
	
	Serial.println(n_Left);
	
	

}
void r180dr() {
	motor.control(97, 45);
	delay(980);
	motor.control(0, 0);
	delay(200);
	motor.control(0, -50);
	delay(100);
	motor.control(0, 0);
	delay(400);
}


void loop() {
  
	
	
	Forward(389);//340mm
	TurnCW(154);
	Forward(297);//750 //260mm
	TurnCCW(154);
	Forward(572); //500mm
	TurnCCW(154);
 r180dr();
 TurnCW(154);
 
 Forward(840);//
  
  
  while (1);
  
  


}
