#include <Wire.h>
#define mpu_add 0x68  //mpu6050 address

class kalman {
  public :
    double getkalman(double acc, double gyro, double dt) {
      //project the state ahead
      angle += dt * (gyro - bias) ;

      //Project the error covariance ahead
      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle) ;
      P[0][1] -= dt * P[1][1] ;
      P[1][0] -= dt * P[1][1] ;
      P[1][1] += Q_gyro * dt ;

      //Compute the Kalman gain
      double S = P[0][0] + R_measure ;
      K[0] = P[0][0] / S ;
      K[1] = P[1][0] / S ;

      //Update estimate with measurement z
      double y = acc - angle ;
      angle += K[0] * y ;
      bias += K[1] * y ;

      //Update the error covariance
      double P_temp[2] = {P[0][0], P[0][1]} ;
      P[0][0] -= K[0] * P_temp[0] ;
      P[0][1] -= K[0] * P_temp[1] ;
      P[1][0] -= K[1] * P_temp[0] ;
      P[1][1] -= K[1] * P_temp[1] ;

      return angle ;
    } ;

    void init(double angle, double gyro, double measure) {
      Q_angle = angle ;
      Q_gyro = gyro ;
      R_measure = measure ;

      angle = 0 ;
      bias = 0 ;

      P[0][0] = 0 ;
      P[0][1] = 0 ;
      P[1][0] = 0 ;
      P[1][1] = 0 ;
    } ;

    double getvar(int num) {
      switch (num) {
        case 0 :
          return Q_angle ;
          break ;

        case 1 :
          return Q_gyro ;
          break ;

        case 2 :
          return R_measure ;
          break ;
      }
    } ;
  private :
    double Q_angle, Q_gyro, R_measure ;
    double angle, bias ;
    double P[2][2], K[2] ;

} ;

kalman kal ;

long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z ;

double deg, dgy_y ;
double dt ;
uint32_t pasttime ;



#include <IMU.h>
cIMU    IMU;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200) ;

  IMU.begin();
#if 0
  Wire.begin() ;
  Wire.beginTransmission(mpu_add) ;
  Wire.write(0x6B) ;
  Wire.write(0) ;
  Wire.endTransmission(true) ;
#endif
  kal.init(0.001, 0.003, 0.03) ;  //init kalman filter
  Serial.println() ;
  Serial.print("parameter") ;
  Serial.print("\t") ;
  Serial.print(kal.getvar(0), 4) ;
  Serial.print("\t") ;
  Serial.print(kal.getvar(1), 4) ;
  Serial.print("\t") ;
  Serial.println(kal.getvar(2), 4) ;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  IMU.update();
#if 0  
  Wire.beginTransmission(mpu_add) ; //get acc data
  Wire.write(0x3B) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read() ;
  ac_z = Wire.read() << 8 | Wire.read() ;

  Wire.beginTransmission(mpu_add) ; //get gyro data
  Wire.write(0x43) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  gy_x = Wire.read() << 8 | Wire.read() ;
  gy_y = Wire.read() << 8 | Wire.read() ;
  gy_z = Wire.read() << 8 | Wire.read() ;
#endif

  deg = atan2(IMU.accRaw[0]/*ac_x*/, IMU.accRaw[2]/*ac_z*/) * 180 / PI ;  //acc data to degree data
  dgy_y = IMU.gyroRaw[1]/*gy_y*/ / 131. ;  //gyro output to

  dt = (double)(micros() - pasttime) / 1000000;
  pasttime = micros();  //convert output to understandable data

  double val = kal.getkalman(deg, dgy_y, dt) ;  //get kalman data

  Serial.print("kalman degree") ;
  Serial.print("\t") ;
  Serial.println(val) ;
}
