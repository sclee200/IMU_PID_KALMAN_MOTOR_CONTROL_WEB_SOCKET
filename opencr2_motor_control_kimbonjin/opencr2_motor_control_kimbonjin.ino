#include <DynamixelWorkbench.h>
#include <IMU.h>


#define baudrate 115200
#define baudrateToDxl 1000000

cIMU    IMU;
DynamixelWorkbench dwb;
int a, b, c, d, e, f;
char x;
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;

const double RADIAN_TO_DEGREE = 180/3.141592;
const double DEG_PER_SEC = 16.4;    // 1초에 회전하는 각도
const double ALPHA = 0.96;

double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;

unsigned long now = 0;   // 현재 시간 저장용 변수
unsigned long past = 0;  // 이전 시간 저장용 변수
double dt = 0;           // 한 사이클 동안 걸린 시간 변수 


void multiPrint(int  s1, int  s2 , int  s3 , int  s4, int  s5 , int  s6)  //note default parameters
{
  Serial.print(s1);
  Serial.print(s2);
  Serial.print(s3);
  Serial.print(s4);
  Serial.print(s5);
  Serial.print(s6);
  Serial.print("\n");
}







void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  const char *log;
  Serial.begin(baudrate);
  IMU.begin();
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(36, OUTPUT);
  dwb.begin("",baudrateToDxl);
  dwb.ping(1);
  dwb.ping(2);
  //dwb.jointMode(1);
  //dwb.jointMode(2);
  //result = dxl.wheelMode(1, 0, &log);
  //if (result == false)
  //{
  //  Serial.println(log);
  //  Serial.println("Failed to change wheel mode");
  //}
  dwb.wheelMode(1, 0, &log);
  dwb.wheelMode(2, 0, &log);
  
  calSensor(); //get initial average value
  past = millis();
//  int num = 123456;
//    Serial << "My variable's number is: " << num << '\n';
//  multiPrint(a,b,c,d,e,f);
}

int count=0;
// the loop function runs over and over again forever
void loop() {

  static uint32_t tTime[3];
  static uint32_t imu_time = 0;
 
//  multiPrint(a,b,c,d,e,f);
  if( (millis()-tTime[0]) >= 100 )
  {
    multiPrint(a,b,c,d,e,f);
    tTime[0] = millis();
    {
    //imu calulation 
      IMU.update();
      now = millis();
      dt = (now - past)/1000.;
      past = now;
      AcX = IMU.accRaw[0];    // ACC X
      AcY = IMU.accRaw[1];    // ACC Y
      AcZ = IMU.accRaw[2];    // ACC Z
      GyX = IMU.gyroRaw[0];    // GYRO X
      GyY = IMU.gyroRaw[1];    // GYRO Y
      GyZ = IMU.gyroRaw[2];    // GYRO Z
      angleAcX = atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2)));
      angleAcX *= RADIAN_TO_DEGREE;
      angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
      angleAcY *= RADIAN_TO_DEGREE;
        
      angleGyX += ((GyX - averGyX) / DEG_PER_SEC) * dt;  //각속도로 변환
      angleGyY += ((GyY - averGyY) / DEG_PER_SEC) * dt;
      angleGyZ += ((GyZ - averGyZ) / DEG_PER_SEC) * dt;
      
      // 상보필터 처리를 위한 임시각도 저장
      double angleTmpX = angleFiX + angleGyX * dt;
      double angleTmpY = angleFiY + angleGyY * dt;
      double angleTmpZ = angleFiZ + angleGyZ * dt;
  
      angleFiX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
      angleFiY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
      angleFiZ = angleGyZ;    // Z축은 자이로 센서만을 이용하열 구함.
      
      Serial.print("AngleAcX:");
      Serial.print(angleAcX);
      Serial.print("\t FilteredX:");
      Serial.print(angleFiX);
      Serial.print("\t AngleAcY:");
      Serial.print(angleAcY);
      
      Serial.print("\t FilteredY:");
      Serial.print(angleFiY);
      Serial.print("\t AngleGyZ:");
      Serial.print(angleGyZ);
      Serial.print("\t FilteredZ:");
      Serial.print(angleFiZ);
      Serial.print("\n");
    }
    count ++;
    if(count ==1)
        digitalWrite(22, a);    
    else if(count ==2)
        digitalWrite(23, b);    
    else if(count ==3)
        digitalWrite(24, c);    
    else if(count ==4)
        digitalWrite(25, d);    
    else if(count ==5)
        digitalWrite(36, e);    
    else if(count ==6)
        digitalWrite(22, f);    
    else if(count ==7)
        digitalWrite(23, f);    
    else if(count ==8)
        digitalWrite(24, f);    
    else if(count ==9)
        digitalWrite(25, f);    
    else if(count ==10)
        digitalWrite(36, f);
    else
        count = 0;
  }
}

void serialEvent(){
  x = Serial.read(); //String().toInt();
  //Serial.println(x);
  if (x == '1'){ //first blink
    a = 1;
    b = 0;
    c = 0;
    d = 0;
    e = 0;
    f = 0;
  }
  else if (x == '2'){ //second blink
    a = 0;
    b = 1;
    c = 0;
    d = 0;
    e = 0;
    f = 0;
  }
  else if (x == '3'){ //third blink
    a = 0;
    b = 0;
    c = 1;
    d = 0;
    e = 0;
    f = 0;
  }
  else if (x == '4'){ //serial blink
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    e = 0;
    f = 1;     
  }
  else if (x == '5'){//all on
    a = 1;
    b = 1;
    c = 1;
    d = 1;
    e = 1;
    f = 1;     
  }
  else if (x == '6'){ //all out
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    e = 0;
    f = 0;     
  }
  else if (x == 's'){
    //dwb.goalPosition(1,0);
    //dwb.goalPosition(2,0);
    dwb.goalVelocity(1, (int32_t)0);
    dwb.goalVelocity(2, (int32_t)0);
  }
  else if (x == 'f'){    
    //dwb.goalPosition(1,2000);
    //dwb.goalPosition(2,2000); 
    dwb.goalVelocity(1, (int32_t)100);
    dwb.goalVelocity(2, (int32_t)100);
    
  }
  else if (x == 'l'){
    dwb.goalVelocity(1, (int32_t)10);
    dwb.goalVelocity(2, (int32_t)100);
  }
  else if (x == 'r'){
    dwb.goalVelocity(1, (int32_t)100);
    dwb.goalVelocity(2, (int32_t)10);
  }    
  else if (x == 'b'){
    dwb.goalVelocity(1, (int32_t)-100);
    dwb.goalVelocity(2, (int32_t)-100);
  }    

  else if (x == 'i'){
    Serial.println(IMU.accRaw[0]);    // ACC X
    Serial.println(IMU.accRaw[1]);    // ACC Y
    Serial.println(IMU.accRaw[2]);    // ACC Z
    Serial.println(IMU.gyroRaw[0]);    // GYRO X
    Serial.println(IMU.gyroRaw[1]);    // GYRO Y
    Serial.println(IMU.gyroRaw[2]);    // GYRO Z
    Serial.println(IMU.rpy[0]);
    Serial.println(IMU.rpy[1]);
    Serial.println(IMU.rpy[2]);
  } 
  else if( x == 'c' )
  {
  //  Serial.println("ACC Cali Start");

    IMU.SEN.acc_cali_start();
    while( IMU.SEN.acc_cali_get_done() == false )
    {
      IMU.update();
    }

  //  Serial.print("ACC Cali End ");
  }
  
  else {
    
  }
}

void calSensor(){
  double sumAcX = 0 , sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0 , sumGyY = 0, sumGyZ = 0;
  IMU.update();
  for (int i=0;i<10;i++) {
    IMU.update();
    sumAcX += IMU.accRaw[0];    // ACC X
    sumAcY += IMU.accRaw[1];    // ACC Y
    sumAcZ += IMU.accRaw[2];    // ACC Z
    sumGyX += IMU.gyroRaw[0];    // GYRO X
    sumGyY += IMU.gyroRaw[1];    // GYRO Y
    sumGyZ += IMU.gyroRaw[2];    // GYRO Z
    sumAcX+=AcX;  sumAcY+=AcY;  sumAcZ+=AcZ;
    sumGyX+=GyX;  sumGyY+=GyY;  sumGyZ+=GyZ;
    delay(50);
  }
  averAcX=sumAcX/10;  averAcY=sumAcY/10;  averAcZ=sumAcY/10;
  averGyX=sumGyX/10;  averGyY=sumGyY/10;  averGyZ=sumGyZ/10;

}
  
