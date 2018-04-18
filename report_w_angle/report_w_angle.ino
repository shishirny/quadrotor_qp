#include <Wire.h>
#include <Servo.h>


Servo right_prop;
Servo left_prop;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
char desired_w[2];



float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=1.1;//3.55//4.2
double ki=0.005;//0.003//0.005
double kd=0;//2.05//0.9
///////////////////////////////////////////////

double throttle=1300; //initial value of throttle to the motors
float desired_angle = 2; //This is the angle in which we whant the
                         //balance to stay steady


void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  right_prop.attach(3); //attatch the right motor to pin 3
  left_prop.attach(5);  //attatch the left motor to pin 5

  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  delay(2000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 

  //Serial.println("Ready...");

}//end of setup void

char rcv[7];

void loop() {

//while(1)  
if(Serial.available()>=5)
{
  delay(20);
  if (Serial.read()=='a')
  {
  char sign= Serial.read();
  if(sign=='-') 
  {
    desired_angle= (int)(Serial.read() - '0') * 1000 ;
    desired_angle+= (int)(Serial.read() - '0') * 100 ;
    desired_angle+= (int)(Serial.read() - '0') * 10 ;
    desired_angle+=(int)(Serial.read()-'0');    
    desired_angle = -desired_angle;
  }
  else 
  {
    desired_angle= (int)(sign - '0') * 1000 ;
    desired_angle+= (int)(Serial.read() - '0') * 100 ;
    desired_angle+= (int)(Serial.read() - '0') * 10  ;
    desired_angle+=(int)(Serial.read()-'0');
  }  
  
  
  //Serial.println(desired_angle);
  while(Serial.available())Serial.read();
  
/*  if (desired_angle==-49) 
  {
    //if(_running)
    {
      left_prop.writeMicroseconds(1000);
      right_prop.writeMicroseconds(1000);
      //_running=0;
      
      //Serial.println("Stopped...");
    }
  
   while (1)
    {
      delay(100);
      //_running=1;
      if(Serial.available())
      {
        while(Serial.available())Serial.read();
        desired_angle=0;
        throttle=1300;
        PID=0;
        pid_p=0;
        pid_i=0;
        pid_d=0;
        Serial.println("Started...");
        break;
      }
    }
    
}*/
    Serial.flush();

  }
}


/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 

     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   

     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
    /*///This is the part where you need to calculate the angles using Euler equations///*/

     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 

    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
 

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;
   
   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
//Serial.println(Total_angle[1]);

   
  
/*///////////////////////////P I D///////////////////////////////////*/

Serial.print(Total_angle[1]);
Serial.print('\n');
/*error = Total_angle[1] - desired_angle;
    


pid_p = kp*error;

if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}


pid_d = kd*((error - previous_error)/elapsedTime);

PID = pid_p + pid_i + pid_d;

*/
//Serial.readBytesUntil('\n', desired_w, 2);
PID = desired_angle-3.5*Gyro_angle[1];
//Serial.print(desired_angle);
//Serial.print(desired_w[1]);
//Serial.print('\n');
if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}

pwmLeft = throttle - PID;
pwmRight = throttle + PID;



if(pwmRight < 1000)
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}
//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}

left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = error; //Remember to store the previous error.*/
delay(1);
}//end of loop void
