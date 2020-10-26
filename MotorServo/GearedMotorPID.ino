//***************************** PID ********************************/
#include <PID_v1.h>

double Pk1 = 1;                                                     //speed it gets there
double Ik1 = 0;
double Dk1 = 0;

double Setpoint1, Input1, Output1, Output1a;                        // PID variables

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup


unsigned long currentMillis;

long previousMillis = 0;    // set up timers
long interval = 20;        // time constant for timers

//******************************************************************/

int angle;
int red;

void setup() {

  pinMode(A0, INPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  
  Serial.begin(115200);

  PID1.SetMode(AUTOMATIC);              // PID Setup - trousers SERVO
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);

}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  //start timed event
      previousMillis = currentMillis;

      if(Serial.available() > 0){
        angle = Serial.parseInt();

        if(Serial.read() == '\n'){
          angle = constrain(angle, -255, 255);
        }


      }

      MotorAngle(3 , angle);


}
}

//*********************** ARM MOTOR MOVİNG ************************/

void MotorAngle(int leg, int angle){                                     


  if(leg == 3){   
    Setpoint1 = angle;
    red = angle;
    int rot = analogRead(A0);
    Input1 = map(rot, 0, 1023, -255, 255);
    
    PID1.Compute();
    Serial.print(red);
    Serial.print("    ");
    Serial.print(Input1);
    Serial.print("    ");
    Serial.println(abs(Output1));

    if (Output1 > 0) {
        Output1 = filter(Output1);
        analogWrite(5, Output1);
        analogWrite(6, 0);
        
        
      }
      else if (Output1 < 0) {
        Output1a = abs(Output1);
        Output1a = filter(Output1a);
        analogWrite(5, 0);
        analogWrite(6, Output1a);
        
      }
  }

  
}
//*****************************************************************/
float filter(float number){         // That geared motor can't move at values below 45 pwm signal because of gears in geared box.
  if(number < 45 && number > 10){
    number = 45;
    return number;
  }
  else{
    return number;
  }
}

