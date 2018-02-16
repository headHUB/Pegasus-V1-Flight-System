#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <BME280I2C.h>
#include <MPU6050.h>

long loop_timer;
unsigned long timer = 0;
float timeStep = 0.01;

float gpitch = 0, groll = 0, gyaw = 0;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;

double R = 8.3144598;
double g = 9.80665;
double M = 0.0289644;
double Pb = 101325;
double hb = 0;
double num=0,dnum=0,h=0;

unsigned long int a,b,c;
int x[15],ch1[15],ch[7],i; //specifing arrays and variables to store values

// Motor connection pins
int M1 = 3;
int M2 = 4;
int M3 = 5;
int M4 = 6;
int M5 = 7;
int M6 = 8;

int X, Y, Z;

//Define Variables we'll be connecting to
double Setpoint1,Setpoint2,Setpoint3,Setpoint4;
double Input1,Input2,Input3,Input4;
double Output1,Output2,Output3,Output4;
//Specify the links and initial tuning parameters
double Kp = 4; double Ki = 1; double Kd = 0.5; 
double Kp1 = 4; double Ki1 = 1; double Kd1 = 0.5; 
double Kp2 = 4; double Ki2 = 1; double Kd2 = 0.5; 
double Kp3 = 4; double Ki3 = 1; double Kd3 = 0.5; 

Servo Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;
PID ThrottPID(&Input1, &Output1, &Setpoint1, Kp,Ki,Kd, DIRECT);
PID RollPID(&Input2, &Output2, &Setpoint2, Kp1,Ki1,Kd1, DIRECT);
PID YawPID(&Input3, &Output3, &Setpoint3, Kp2,Ki2,Kd2, DIRECT);
PID myPID4(&Input4, &Output4, &Setpoint4, Kp3,Ki3,Kd3, DIRECT);
MPU6050 mpu;
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

void setup() 
{
  Serial.begin(115200);
  Serial.println("Initialize BME280");
  Serial.println("Initialize MPU6050 Gyro");
  Serial.println("Initialize MPU6050 Gyro Accel");
  
  Wire.begin();                // join i2c bus with address #1
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), read_me, FALLING); // enabling interrupt at pin 2

  while(!Serial) {} // Wait
  
  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
  
  // Attach motors to servo instances
  Motor1.attach(M1);
  Motor2.attach(M2);
  Motor3.attach(M3);
  Motor4.attach(M4);
  Motor5.attach(M5);
  Motor6.attach(M6);

  Motor1.writeMicroseconds(1000);
  Motor2.writeMicroseconds(1000);
  Motor3.writeMicroseconds(1000);
  Motor4.writeMicroseconds(1000);
  Motor5.writeMicroseconds(1000);
  Motor6.writeMicroseconds(1000);
  delay(500);
  
  ThrottPID.SetOutputLimits(1100, 2000);
  RollPID.SetOutputLimits(1100, 2000);
  YawPID.SetOutputLimits(1100, 2000);
  myPID4.SetOutputLimits(1100, 2000);
  
  ThrottPID.SetMode(AUTOMATIC); 
  RollPID.SetMode(AUTOMATIC); 
  YawPID.SetMode(AUTOMATIC); 
  myPID4.SetMode(AUTOMATIC); 
}

void loop()
{ 
  read_rc();
  printBME280Data(&Serial);
  ThrottPID.Compute();
  RollPID.Compute();
  YawPID.Compute();
  myPID4.Compute();
  
  Serial.print(ch[1]);Serial.print("\t");
  Serial.print(ch[2]);Serial.print("\t");
  Serial.print(ch[3]);Serial.print("\t");
  Serial.print(ch[4]);Serial.print("\t");
  Serial.print(ch[5]);Serial.print("\t");
  Serial.print(ch[6]);Serial.print("\n");
  
}

void altcontr(int al,double x)
{
  double b = x;
    if( al > 1750)
    {
      b = b + 1.5;
      Setpoint1 = b;
      Throttle(Output1);
    }
    else if( al > 1650 &&  al < 1750)
    {
      b = b + 1;
      Setpoint1 = b;
      Throttle(Output1);
    }
    else if( al > 1550 &&  al < 1650)
    {
      b = b + 0.5;
      Setpoint1 = b;
      Throttle(Output1);
    }
    else if( al < 1450 && al > 1350)
    {
      b = b - 0.5;
      Setpoint1 = b;
      Throttle(Output1);
    }
    else if( al < 1350 && al > 1250)
    {
      b = b - 1;
      Setpoint1 = b;
      Throttle(Output1);
    }
    else if( al < 1250)
    {
      b = b - 1.5;
      Setpoint1 = b;
      Throttle(Output1);
    }
    else if( al > 1450 && al < 1550)
    {
      Setpoint1 = b;
      Throttle(Output1);
    }
}

void Throttle(double a)
{
  Motor1.writeMicroseconds(a);
  Motor2.writeMicroseconds(a);
  Motor3.writeMicroseconds(a);
  Motor4.writeMicroseconds(a);
}

void printBME280Data(Stream* client)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);
   
   double T = 2 + 273;
   
   num = log(pres/Pb) * T * R;
   dnum = g * M * -1;
   h = (num/dnum)+ hb;
   
   //client->print(h);
   //client->println(" m");

   Input1 = h;
   altcontr(ch[1],h);
   
   timer = millis();
  
   Vector norm = mpu.readNormalizeGyro();
   Vector normAccel = mpu.readNormalizeAccel();

   gpitch = gpitch + norm.YAxis * timeStep;
   groll = groll + norm.XAxis * timeStep;
   gyaw = gyaw + norm.ZAxis * timeStep;
  
   int Pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
   int Roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  
   if(set_gyro_angles)
   {                                                 //If the IMU is already started
     gpitch = gpitch * 0.9996 + Pitch * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
     groll = groll * 0.9996 + Roll * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
   }
   else
   {                                                                //At first start
     gpitch = Pitch;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
     groll = Roll;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
     set_gyro_angles = true;                                            //Set the IMU started flag
   }
  
   //To dampen the pitch and roll angles a complementary filter is used
   angle_pitch_output = angle_pitch_output * 0.9 +  gpitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
   angle_roll_output = angle_roll_output * 0.9 +  groll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

   int x,y, z;
   x = gpitch;
   y = groll;
   z = gyaw;
  /*
   client->print("GRPitch = ");
   client->print(x);
   client->print(" GRRoll = ");
   client->print(y); 
   client->print(" GRYaw = ");
   client->println(z);
  */
   delay((timeStep*650) - (millis() - timer)); 
}

void read_me() 
{
  //this code reads value from RC reciever from PPM pin (Pin 2 or 3)
  //this code gives channel values from 0-1000 values 
  //    -: ABHILASH :-    //
  a=micros(); //store time value a when pin value falling
  c=a-b;      //calculating time inbetween two peaks
  b=a;        // 
  x[i]=c;     //storing 15 value in array
  i=i+1;       

  if(i==15)
  {
    for(int j=0;j<15;j++) 
    {
      ch1[j]=x[j];
    }
    i=0;
  }
}//copy store all values from temporary array another array after 15 reading 
 
void read_rc()
{
  int i,j,k=0;
  for(k=14;k>-1;k--)
  {
    if(ch1[k]>10000)
    {
      j=k;
    }
  } //detecting separation space 10000us in that another array                     
  for(i=1;i<=6;i++)
  {
    ch[i]=(ch1[i+j]);
  }
}     //assign 6 channel values after separation space
