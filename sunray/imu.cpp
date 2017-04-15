#include "imu.h"
#include "helper_3dmath.h"
#include "i2c.h"
#include "helper.h"
#include "config.h"
#include "buzzer.h"
#include "flashmem.h"
#include <math.h>


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define ADDR 40
#define MAGIC 4

#define GYRO_DPS 2000       // max gyro dps
#define GYRO_CAL_INTERVAL  1000L * 180L // gyro is precise (<2 degree) for 3 minutes
#define GYRO_CAL_FIRST_INTERVAL 10000   // first gyro calibration
#define GYRO_CAL_TIME  1000   // wait for one second for measurement
#define GYRO_CAL_BIAS_DPS_MAX 0.01    // maximum allowed bias after calibration (degree per sec)

#define DEFAULT_MPU_HZ  (100)  // sensor sampling rate
#define DMP_FIFO_RATE 5       // DMP FIFO rate

// -------------I2C addresses ------------------------
#define ADXL345B (0x53)          // ADXL345B acceleration sensor (GY-80 PCB)
#define HMC5883L (0x1E)          // HMC5883L compass sensor (GY-80 PCB)
#define CMPS11   (0x60)          // CMPS11 compass sensor


IMUClass IMU;
//Adafruit_BNO055 bno = Adafruit_BNO055(55);



/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};



/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}


static void tap_cb(unsigned char direction, unsigned char count)
{
    /*char data[2];
    data[0] = (char)direction;
    data[1] = (char)count;
    send_packet(PACKET_TYPE_TAP, data);*/
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
    //hal.new_gyro = 1;
}


//  This function must be called with the device either face-up or face-down
//  (z-axis is parallel to gravity).
void IMUClass::runSelfTest(bool accelSelfTest) 
{
    int result;
    char test_packet[4] = {0};
    long gyro[3], accel[3];
    unsigned char i = 0;

  while(true){

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        DEBUG(F("IMU runSelfTest - accelSelfTest="));                    
        DEBUGLN(accelSelfTest);                            

        for(i = 0; i<3; i++) {
          gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
          accel[i] *= 2048.f; //convert to +-16G
          accel[i] = accel[i] >> 16;          
          gyro[i] = (long)(gyro[i] >> 16);
        }        
        gyroBias[0]=gyro[0];
        gyroBias[1]=gyro[1];
        gyroBias[2]=gyro[2];
        DEBUG("mpu_set_gyro_bias_reg (delta)=");        
        DEBUG(gyroBias[0]);
        DEBUG(F(","));
        DEBUG(gyroBias[1]);
        DEBUG(F(","));
        DEBUGLN(gyroBias[2]);                      
        mpu_set_gyro_bias_reg(gyro);        

        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        //dmp_set_gyro_bias(gyro);
                
        if (accelSelfTest) {          
          long accel_st[3]; 
          mpu_read_6050_accel_bias(accel_st);                    
          accelCal[0] = accel_st[0] - accel[0];
          accelCal[1] = accel_st[1] - accel[1];
          accelCal[2] = accel_st[2] - accel[2];                  
          mpu_set_accel_bias_6050_reg(accel);   
          unsigned short accel_sens;
          mpu_get_accel_sens(&accel_sens);
          accel[0] *= accel_sens;
          accel[1] *= accel_sens;
          accel[2] *= accel_sens;
          //dmp_set_accel_bias(accel);                            
          saveCalib();                  
        }
        return;
    } 
    DEBUG(F("ERROR IMU self test: "));    
    DEBUGLN(result);        
  }
}


void IMUClass::initCompassHMC5833L(){
  // HMC5883L compass sensor driver
  uint8_t data = 0;
  while(true){
    I2C_readFrom(HMC5883L, 10, 1, &data, 1);
    DEBUG(F("HMC5883L ID="));
    DEBUGLN(data);    
    if (data == 72) break;
    delay(1000);
  }    
  I2C_writeToValue(HMC5883L, 0x00, 0b01111000);  // config A:  8 samples averaged, 75Hz frequency, no artificial bias.         
  I2C_writeToValue(HMC5883L, 0x01, 0b00000000);  // config B: gain
  I2C_writeToValue(HMC5883L, 0x02, 0b00000000);  // mode: continuous
}


void IMUClass::initAccelerationADXL345B(){
  // ADXL345B acceleration sensor driver  
  uint8_t data = 0;
  while(true){
    I2C_readFrom(ADXL345B, 0x00, 1, &data, 1);
    DEBUG(F("ADXL345B ID="));
    DEBUGLN(data);    
    if (data == 0xE5) break;    
    delay(1000);
  }
  I2C_writeToValue(ADXL345B, 0x2D, 0);
  I2C_writeToValue(ADXL345B, 0x2D, 16);
  I2C_writeToValue(ADXL345B, 0x2D, 8);            
}


void IMUClass::initSensors(){
  initCompassHMC5833L();
  // initAccelerationADXL345B();
  
  // MPU9150/6050
  uint8_t data = 0;
  while (true){     
     I2C_readFrom(0x68, 0x75, 1, &data, 1); // mpu6050/9150     
     DEBUG(F("MPU6050 ID="));
     DEBUGLN(data);
     if (data == 104) break;
     // (0b110100, 0x34)
     delay(1000);
  }

  /*// CMPS11
  data = 0;  
  while (true){
    byte res = I2C_readFrom(CMPS11, 0x00, 1, &data, 1);   // CMPS11
    DEBUG(F("CMPS11 ID="));  
    DEBUGLN(data);
    if (data == 3) break;
    delay(1000);
  }*/

  // BNO55
  /*while (true){
    if (bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))  {
      //if (bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS)) {              
      break;
    }
    DEBUGLN(F("IMU BNO055?"));
    delay(1000);
  }
  bno.setExtCrystalUse(true);  
  if (calibFound) {
    DEBUGLN("bno.setSensorOffsets");
    bno.setSensorOffsets(calibData);
  }*/
} 


void IMUClass::begin()
{    
  gyroBiasDpsMax = GYRO_CAL_BIAS_DPS_MAX;
  statsGyroCalibrationTimeMax = 0;
  statsYawMin = 10;
  statsYawMax = -10;   
  useGyro = true;  
  int result;
  //struct int_param_s int_param;
  unsigned char accel_fsr;
  unsigned short gyro_rate = 0;
  unsigned short gyro_fsr;
  accelCal[0]=424;  
  accelCal[1]=62143;
  accelCal[2]=1335; 
  comAccMin.x=comAccMin.y=comAccMin.z = 0;
  comAccMax.x=comAccMax.y=comAccMax.z = 0;  
  comAccOfs.x= 4.41;     comAccOfs.y=1.4;    comAccOfs.z = -12.58;
  comAccScale.x= 0.01;   comAccScale.y= 0.01;  comAccScale.z = 0.01;    
  useComAccCalibration = true;
  calibComAccAxisCounter = 0;
  comCalA_1[0]=  1.45;  comCalA_1[1]= 0.0;    comCalA_1[2]= -0.02;
  comCalA_1[3]=  0.00;  comCalA_1[4]= 1.51;   comCalA_1[5]=  0.01;
  comCalA_1[6]= -0.02;  comCalA_1[7]= 0.01;   comCalA_1[8]=  1.62;
  comCalB[0]= -185.70;
  comCalB[1]= 35.18;
  comCalB[2]= -342.41;  
  isRotating = false;
  calibrationTime = GYRO_CAL_TIME;
  yawComOfs = 0;
  ypr.yaw =0;
  ypr.pitch =0;
  ypr.roll =0;
  mode=IMU_MODE_COM_TILT;
  verboseOutput = false;
  useComCalibration = true;    
  gyroZlowpass = 0;
  nextBeepTime=0;
  nextComTime=0;
  nextInfoTime =0;  
    
  loadCalib();
  initSensors();   

  while(true){
    result = mpu_init(NULL);
    if (result == 0) break;
    DEBUG(F("ERROR: mpu_init failed "));
    DEBUGLN(result);    
    delay(1000);
  } 

  // Get/set hardware configuration. Start gyro.
  // Wake up all sensors.
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  //mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // Push both gyro and accel data into the FIFO.
  mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
  mpu_set_gyro_fsr( GYRO_DPS );      
  while(true){
    mpu_set_sample_rate(DEFAULT_MPU_HZ); 
    // Read back configuration in case it was set improperly. 
    mpu_get_sample_rate(&gyro_rate);
    if (gyro_rate == DEFAULT_MPU_HZ) break;
    DEBUGLN(F("ERROR: mpu_set_sample_rate failed"));
    delay(1000);    
  }
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  DEBUG(F("gyro_rate="));
  DEBUGLN(gyro_rate);
  DEBUG(F("  gyro_fsr="));
  DEBUGLN(gyro_fsr);
  DEBUG(F("dps  accel_fsr="));  
  DEBUG(accel_fsr);
  DEBUGLN(F("g"));
 
  runSelfTest(false); // required to get precise acceleration values

  // set accel sensor bias
  long accel_st[3]; 
  long accel[3]; 
  mpu_read_6050_accel_bias(accel_st);          
  accel[0] = -(accelCal[0] - accel_st[0]);
  accel[1] = -(accelCal[1] - accel_st[1]);
  accel[2] = -(accelCal[2] - accel_st[2]);  
  mpu_set_accel_bias_6050_reg(accel);                                             
  unsigned short accel_sens;
  mpu_get_accel_sens(&accel_sens);
  accel[0] *= accel_sens;
  accel[1] *= accel_sens;
  accel[2] *= accel_sens;
  //dmp_set_accel_bias(accel);       
  mpu_read_6050_accel_bias(accel);          
  DEBUG(F("accelreg="));                            
  DEBUG(accel[0]);                 
  DEBUG(F(","));                       
  DEBUG(accel[1]);                            
  DEBUG(F(","));                               
  DEBUGLN(accel[2]);                           

  // init DMP
  while(true){
    result = dmp_load_motion_driver_firmware();
    if (!result) break;
    DEBUG(F("ERROR: dmp_load_motion_driver_firmware failed "));
    DEBUGLN(result);    
    delay(1000);
  }

  // warning: do not use DMP_FEATURE_GYRO_CAL because then gyro will not count slow rotations!!
  dmp_set_orientation( inv_orientation_matrix_to_scalar(gyro_orientation) );
  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO; // | DMP_FEATURE_GYRO_CAL;
  //unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP; // | DMP_FEATURE_GYRO_CAL;
  //dmp_register_tap_cb(tap_cb);
  //dmp_register_android_orient_cb(android_orient_cb);
  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DMP_FIFO_RATE);
  mpu_set_dmp_state(1);
  
  // mpu_set_compass_sample_rate(100);
  nextGyroCalTime = millis() + GYRO_CAL_FIRST_INTERVAL;
}


float IMUClass::getYaw(){
  return ypr.yaw;
}

uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}

uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}


uint8_t dmpGetLinearAccel(point_float_t *v, point_float_t *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component 
  	v->x = vRaw -> x + gravity -> x;
    v->y = vRaw -> y + gravity -> y;
    v->z = vRaw -> z - gravity -> z;
	return 0;
}

void IMUClass::printSensors(){
  if (!verboseOutput) return;
  // raw compass values
  DEBUG(F("!04,"));
  DEBUG(comR.x);
  DEBUG(F(","));
  DEBUG(comR.y);
  DEBUG(F(","));
  DEBUG(comR.z);
  DEBUG(F(","));
  DEBUG(ypr.yaw);
  DEBUG(F(","));
  DEBUG(ypr.pitch);
  DEBUG(F(","));
  DEBUG(ypr.roll);
  DEBUG(F(","));
  DEBUG(q.w);
  DEBUG(F(","));
  DEBUG(q.x);
  DEBUG(F(","));
  DEBUG(q.y);
  DEBUG(F(","));
  DEBUG(q.z);
  DEBUG(F(","));
  DEBUG(comPitch);
  DEBUG(F(","));
  DEBUG(comRoll);  
	DEBUG(F(","));
	DEBUG(acc.x); 
	DEBUG(F(","));
	DEBUG(acc.y); 
	DEBUG(F(","));
	DEBUG(acc.z); 
	DEBUG(F(","));
	DEBUG(accXmin); 	
	DEBUG(F(","));
	DEBUG(accXmax); 	
  DEBUGLN(); 
}


void IMUClass::readCompassMPU9150(){
  short compass[3];
  unsigned long sensor_timestamp;  
  if (mpu_get_compass_reg(compass, &sensor_timestamp)) return;  
  comR.x = compass[0];
  comR.y = compass[1];
  comR.z = compass[2];                                          
  printSensors();      
  if (useComCalibration){        
    // calA = A_1 * (rawA - B)        
    comR.x -= comCalB[0];
    comR.y -= comCalB[1];
    comR.z -= comCalB[2];
    comR.x = comCalA_1[0] * comR.x + comCalA_1[1] * comR.y + comCalA_1[2] * comR.z;
    comR.y = comCalA_1[3] * comR.x + comCalA_1[4] * comR.y + comCalA_1[5] * comR.z;
    comR.z = comCalA_1[6] * comR.x + comCalA_1[7] * comR.y + comCalA_1[8] * comR.z;        
  }
  // low-pass                                
  //float diff = max( abs(com.x-comR.x), abs(com.y-comR.y) );
  //DEBUGLN(diff);                
  float w = 0.99;                
  com.x = w * com.x + (1.0-w) * comR.x;
  com.y = w * com.y + (1.0-w) * comR.y;
  com.z = w * com.z + (1.0-w) * comR.z;                                          
  // compute tilt compensated compass yaw 
  // local system: X axis pointing forward, Y axis pointing left, Z axis pointing upwards
  // NOTE: MPU9150 compass X = gyro Y,   compass Y = gyro X,   compass Z = gyro -Z
  float pitch = ypr.pitch; // pitch need to increase when X axis going down  
  float roll = -ypr.roll;  // roll  need to increase when Y axis going up    
  float cos_roll = cos(roll);
  float sin_roll = sin(roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = sin(pitch);
  float MAG_X = com.y * cos_pitch - com.z * sin_pitch;
  float MAG_Y = com.y * sin_roll * sin_pitch + com.x * cos_roll + com.z * sin_roll * cos_pitch;               
  comYaw = scalePI( atan2(-MAG_Y,MAG_X) );                         
}


// read acceleration sensor (for compass-tilt compensation)
void IMUClass::readAccelerationADXL345B(){
  uint8_t buf[6];  
  if (I2C_readFrom(ADXL345B, 0x32, 6, (uint8_t*)buf, 1) != 6) return;
  float x=(int16_t) (((uint16_t)buf[1]) << 8 | buf[0]); 
  float y=(int16_t) (((uint16_t)buf[3]) << 8 | buf[2]); 
  float z=(int16_t) (((uint16_t)buf[5]) << 8 | buf[4]);   
  if (useComAccCalibration){
    x = (x - comAccOfs.x) * comAccScale.x;
    y = (y - comAccOfs.y) * comAccScale.y;
    z = (z - comAccOfs.z) * comAccScale.z;    
  } 
  float w= 0.99;
  comAcc.x = w * comAcc.x + (1.0-w) * x;
  comAcc.y = w * comAcc.y + (1.0-w) * y;
  comAcc.z = w * comAcc.z + (1.0-w) * z;  
  comPitch   = atan2(-comAcc.x , sqrt(sq(comAcc.y) + sq(comAcc.z)));         
  comRoll    = atan2(comAcc.y , comAcc.z);              
}


// read compass sensor
void IMUClass::readCompassHMC5883(){    
  uint8_t buf[6];  
  if (I2C_readFrom(HMC5883L, 0x03, 6, (uint8_t*)buf, 1) != 6) return;  
  // scale +1.3Gauss..-1.3Gauss  (*0.00092)  
  comR.x = (int16_t) (((uint16_t)buf[0]) << 8 | buf[1]);
  comR.y = (int16_t) (((uint16_t)buf[4]) << 8 | buf[5]);
  comR.z = (int16_t) (((uint16_t)buf[2]) << 8 | buf[3]);  
  printSensors();        
  if (useComCalibration){
    // calA = A_1 * (rawA - B)        
    comR.x -= comCalB[0];
    comR.y -= comCalB[1];
    comR.z -= comCalB[2];
    comR.x = comCalA_1[0] * comR.x + comCalA_1[1] * comR.y + comCalA_1[2] * comR.z;
    comR.y = comCalA_1[3] * comR.x + comCalA_1[4] * comR.y + comCalA_1[5] * comR.z;
    comR.z = comCalA_1[6] * comR.x + comCalA_1[7] * comR.y + comCalA_1[8] * comR.z;                   
  }
  //float w = 0.99;        
  float w = 0.0;        
  com.x = w * com.x + (1.0-w) * comR.x;
  com.y = w * com.y + (1.0-w) * comR.y;
  com.z = w * com.z + (1.0-w) * comR.z;                                        
  if (mode == IMU_MODE_COM_TILT){
    // compute tilt compensated compass yaw 
    float pitch = ypr.pitch; // pitch need to increase when X axis going down  
    float roll = -ypr.roll;  // roll  need to increase when Y axis going up      
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    float MAG_X = com.x * cos_pitch + com.z * sin_pitch;
    float MAG_Y = com.x * sin_roll * sin_pitch + com.y * cos_roll - com.z * sin_roll * cos_pitch;               
    comYaw = fusionPI(0.9, comYaw, scalePI( atan2(-MAG_Y,MAG_X) ));    
  } else if (mode == IMU_MODE_COM_FLAT){
    // no tilt compensation     
    comYaw = scalePI( atan2(-com.y,com.x) );                         
  }  
}

// BNO055
void IMUClass::readCompassBNO055(){ 
  /*if (bno.isFullyCalibrated()){
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);    
      comYaw = scalePI( 2*PI -  (((float)euler.x())/180.0*PI)  );            
  } else {                  
      if (millis() >= nextInfoTime){
        nextInfoTime = millis() + 1000;      
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);    
        // The data should be ignored until the system calibration is > 0                     
        DEBUG(F("Sys:"));
        DEBUG(system, DEC);
        DEBUG(F(" G:"));
        DEBUG(gyro, DEC);
        DEBUG(F(" A:"));
        DEBUG(accel, DEC);
        DEBUG(F(" M:"));
        DEBUGLN(mag, DEC);      
      }  
  } */
}


// read compass sensor
void IMUClass::readCompassCMPS11(){    
  uint8_t buf[2];  
  if (I2C_readFrom(CMPS11, 0x02, 2, (uint8_t*)buf, 1) != 2) return;  
  int bearing = (int16_t) (((uint16_t)buf[0]) << 8 | buf[1]);
  comYaw = scalePI( PI -  (((float)bearing)/1800.0*PI)  );            
}


void IMUClass::startCompassCalibration(){
  DEBUGLN(F("startCompassCalibration"));  
  /*I2C_writeToValue(CMPS11, 0x00, 0xF0);
  delay(50);
  I2C_writeToValue(CMPS11, 0x00, 0xF5);
  delay(50);
  I2C_writeToValue(CMPS11, 0x00, 0xF6);
  delay(50);*/  
  calibFound = false;
  Buzzer.sound(SND_PROGRESS, false);      
}

void IMUClass::stopCompassCalibration(){
  DEBUGLN(F("stopCompassCalibration"));
  //I2C_writeToValue(CMPS11, 0x00, 0xF8);
  //bno.getSensorOffsets(calibData);
  saveCalib();
  delay(50);  
  Buzzer.sound(SND_READY, false);    
}



void IMUClass::run(){
  unsigned char more;
  long quat[4];       
  short gyroD[3], accel[3], sensors;
  unsigned long sensor_timestamp;  
  bool foundQuat = false;  
  while (true){
    more = 0;
    sensors = 0;
    dmp_read_fifo(gyroD, accel, quat, &sensor_timestamp, &sensors, &more);
    if (sensors & INV_XYZ_ACCEL){
      //DEBUGLN("INV_XYZ_ACCEL");
      acc.x = accel[0]/ACCEL_SENS;
      acc.y = accel[1]/ACCEL_SENS;
      acc.z = accel[2]/ACCEL_SENS;	  
  	  // compute linar acceleration
  	  dmpGetLinearAccel(&acc, &acc, &gravity);
      accXmin = min(accXmin, acc.x);
      accXmax = max(accXmax, acc.x);
      float diff = accXmax - accXmin;
      //isMoving = (diff > 0.05);
      accXmax = 0.8 * accXmax;
      accXmin = 0.8 * accXmin;
    }
    if (sensors &  INV_XYZ_GYRO) {
      //DEBUGLN("INV_XYZ_GYRO");
      gyro.x = gyroD[0]/GYRO_SENS;
      gyro.y = gyroD[1]/GYRO_SENS;
      gyro.z = gyroD[2]/GYRO_SENS;
      gyroSum.x += gyroD[0];
      gyroSum.y += gyroD[1];
      gyroSum.z += gyroD[2];
      gyroSumCount++;
      //DEBUGLN(gyro.z);
      gyroZlowpass = gyroZlowpass * 0.9 + fabs(gyro.z) * 0.1;
      isRotating = (gyroZlowpass > 2);
    }
    if (sensors & INV_WXYZ_QUAT ){
      //DEBUGLN("INV_WXYZ_QUAT");
      //long *ldata = (long*)data;
      q.w = quat[0]/QUAT_SENS;
      q.x = quat[1]/QUAT_SENS;
      q.y = quat[2]/QUAT_SENS;
      q.z = quat[3]/QUAT_SENS;
      foundQuat = true;
      /*DEBUG(q.w);
      DEBUG(F(","));
      DEBUG(q.x);
      DEBUG(F(","));
      DEBUG(q.y);
      DEBUG(F(","));
      DEBUGLN(q.z);    */
      dmpGetGravity(&gravity, &q); // get gravity vector
      dmpGetYawPitchRoll((float*)&ypr, &q, &gravity); // compute ypr
      if (useGyro){
        // flip positive direction to counter-clockwise
        ypr.yaw = scalePI( 2*PI - ypr.yaw + yawComOfs) ;    
        //ypr.roll = -ypr.roll;    
        //ypr.pitch = -ypr.pitch;          
      }      
    }
    if (more == 0) break;
  }

  //readCompassBNO055();
  //readCompassCMPS11();  
  readCompassHMC5883();
  if (!useGyro){
    ypr.yaw = scalePI(comYaw); 
  }
  
  if ((state == IMU_CAL_GYRO) || (state == IMU_CAL_COM) || (verboseOutput)){
    //readCompassMPU9150();    
    //readAccelerationADXL345B();
    //readCompassHMC5883();
  }
  
  //if needGyroCal() startGyroCalibration();    
  if (state == IMU_CAL_GYRO) calibrateGyro();	
    //else if (state == IMU_CAL_COM) calibrateCompass();   

  // stats
  if (millis() > 20000){
    statsYawMax = max(statsYawMax, ypr.yaw);
    statsYawMin = min(statsYawMin, ypr.yaw);  
  }
}


bool IMUClass::needGyroCal(){
  if (!useGyro) return false;
  return ((state == IMU_RUN) && (millis() >= nextGyroCalTime));
}

bool IMUClass::needCompassCal(){
  return false;
  //return (!comCalibrated);
}

void IMUClass::startGyroCalibration(){
  if (state == IMU_RUN){    
    DEBUGLN(F("startGyroCalibration"));
    //runSelfTest(false);
    //dmp_enable_gyro_cal(true);
    // start new calibration            
    state = IMU_CAL_GYRO;
    //yawComOfs = 0;            
    gyroSum.x = gyroSum.y = gyroSum.z = gyroSumCount = 0;
    yawAtGyroCalibrationTime = ypr.yaw;
    timeAtGyroCalibrationTime = millis();
    gyroCalStartTime = millis() + 5000;
    gyroCalStopTime = millis() + calibrationTime;              
  }
}

bool IMUClass::calibrateGyro(){   
  if (millis() < gyroCalStartTime) return false;
  //float yawDiff = abs(distancePI(yprLast.yaw, ypr.yaw)/PI*180.0);
  //DEBUGLN(yawDiff);     

  if (yawAtGyroCalibrationTime > 10) {    
    timeAtGyroCalibrationTime = millis();
    yawAtGyroCalibrationTime = ypr.yaw;         
  }

  if (millis() >= gyroCalStopTime) {
    unsigned long timeDiff = millis() - timeAtGyroCalibrationTime;
    if (timeDiff == 0) timeDiff = 1;
    float yawDiff = abs(distancePI(yawAtGyroCalibrationTime, ypr.yaw)/PI*180.0); // w-x      
    float timeDiffSec = ((float)timeDiff) / 1000.0;
    float biasDps = yawDiff / timeDiffSec;
    long biasX = (gyroSum.x + 0.5) / ((float)gyroSumCount);
    long biasY = (gyroSum.y + 0.5) / ((float)gyroSumCount);
    long biasZ = (gyroSum.z + 0.5) / ((float)gyroSumCount);
    DEBUG(F("gyro count "));
    DEBUG(gyroSumCount);
    DEBUG(F(" bias "));    
    DEBUG(biasX);
    DEBUG(F(","));      
    DEBUG(biasY);
    DEBUG(F(","));      
    DEBUG(biasZ);
    DEBUG(F("  dps "));      
    DEBUG(biasDps);       
    DEBUG(F("  max "));      
    DEBUGLN(gyroBiasDpsMax);      
    //DEBUG(F(" @time "));      
    //DEBUGLN(timeDiffSec, 2);
    //if (yawDiff > gyroErrorMax * ((float)calibrationTime) / 1000.0) {    
    if (biasDps > gyroBiasDpsMax) {          
      //runSelfTest(false);    
      //if (gyroBias[2] != 0){   // a zero bias delta measurement means gyro is stable 
      //if ((abs(gyro.x) > 0.19) || (abs(gyro.y) > 0.19) || (abs(gyro.z) > 0.19)  ){    
      // do not allow to move during calibration
      //DEBUG(F("gyro drift "));
      //DEBUGLN(error);    
      //DEBUG(F("gyro "));
      //DEBUG(gyro.x);
      //DEBUG(F(","));
      //DEBUG(gyro.y);
      //DEBUG(F(","));
      //DEBUGLN(gyro.z);                  
      //long gyro_reg_bias[3] = {0, 0, 0};      
      //mpu_read_6500_gyro_bias(gyro_reg_bias);
      //gyro_reg_bias[0] = -biasX;
      //gyro_reg_bias[1] = -biasY;
      //gyro_reg_bias[2] = -biasZ;      
      //mpu_set_gyro_bias_reg(gyro_reg_bias);

      long gyro_reg_bias[3] = {0, 0, 0};      
      float sens;
      mpu_get_gyro_sens(&sens);
      gyro_reg_bias[0] = -biasX;
      gyro_reg_bias[1] = -biasY;
      gyro_reg_bias[2] = biasZ;
      //mpu_set_gyro_bias_reg(gyro_reg_bias);
      gyro_reg_bias[0] = (long)(-biasX * sens);
      gyro_reg_bias[1] = (long)(-biasY * sens);
      gyro_reg_bias[2] = (long)(biasZ * sens);      
      //dmp_set_gyro_bias(gyro_reg_bias);
        
      runSelfTest(false);    
      gyroSum.x = gyroSum.y = gyroSum.z = gyroSumCount = 0;
      gyroCalStopTime = millis() + calibrationTime;                
      yawAtGyroCalibrationTime = 100; 
      timeAtGyroCalibrationTime = millis();    
    }
  } 

  // wait for pending calibration to finish
  if (millis() >= gyroCalStopTime) {
    //runSelfTest(false);
    // after 8 seconds of silence, gyro bias is auto-calibrated by MPU, now determine compass yaw for current gyro yaw (yawComOfs)                
    if (useComCalibration){
      //DEBUG(F("yawDiff "));
      //DEBUG(yawDiff);    
      DEBUG(F("DONE! ypr.pitch="));
      DEBUGLN(ypr.pitch/PI*180.0);      
      DEBUG(F("ypr.roll="));
      DEBUGLN(ypr.roll/PI*180.0);      
      DEBUG(F("comYaw="));
      DEBUGLN(comYaw/PI*180.0);      
      yawComOfs = distancePI( scalePI(ypr.yaw-yawComOfs), comYaw); // w-x
    }
    statsGyroCalibrationTimeMax = max(statsGyroCalibrationTimeMax, (millis()-gyroCalStartTime) / 1000);
    nextGyroCalTime = millis() + GYRO_CAL_INTERVAL;
  	DEBUG(F("nextGyroCalTime="));    
	  DEBUGLN(nextGyroCalTime);
    //dmp_enable_gyro_cal(false);
    // calibrationTime = 8000; // 9000;
    state = IMU_RUN;    
    return true;
  }

  if (millis() >= nextBeepTime){
    Buzzer.sound(SND_PROGRESS, false);
    nextBeepTime = millis() + 1000;
  }
  return false;
}


// calibrate next acceleration sensor axis 
void IMUClass::calibrateAcceleration(){    
  useComAccCalibration = false;  
  if (calibComAccAxisCounter >= 6) calibComAccAxisCounter = 0;
  if (calibComAccAxisCounter == 0){    
    DEBUGLN(F("acc calib restart..."));
    comAccMin.x = comAccMin.y = comAccMin.z = 99999;
    comAccMax.x = comAccMax.y = comAccMax.z = -99999;    
  }  
  for (int i=0; i < 100; i++){        
    readAccelerationADXL345B();                
    delay(1);
  }
  comAccMin.x = min(comAccMin.x, comAcc.x);
  comAccMax.x = max(comAccMax.x, comAcc.x);         
  comAccMin.y = min(comAccMin.y, comAcc.y);
  comAccMax.y = max(comAccMax.y, comAcc.y);         
  comAccMin.z = min(comAccMin.z, comAcc.z);
  comAccMax.z = max(comAccMax.z, comAcc.z);           
  calibComAccAxisCounter++;        
  useComAccCalibration = true;  
  DEBUG(F("side "));
  DEBUG(calibComAccAxisCounter);
  DEBUGLN(F(" of 6 completed"));    
  if (calibComAccAxisCounter == 6){    
    // all axis complete     
    comAccScale.x = 1.0 / ((comAccMax.x-comAccMin.x)/2.0);
    comAccScale.y = 1.0 / ((comAccMax.y-comAccMin.y)/2.0);
    comAccScale.z = 1.0 / ((comAccMax.z-comAccMin.z)/2.0);
    comAccOfs.x = (comAccMax.x+comAccMin.x)/2.0;
    comAccOfs.y = (comAccMax.y+comAccMin.y)/2.0;
    comAccOfs.z = (comAccMax.z+comAccMin.z)/2.0;        
    saveCalib();            
    DEBUGLN(F("acc calibration completed"));    
    Buzzer.sound(SND_READY, false);    
  } else Buzzer.sound(SND_PROGRESS, true);  
}      


void IMUClass::loadSaveCalib(boolean readflag){
  int addr = ADDR;
  short magic = MAGIC;
  DEBUG(F("IMU EEPROM addr="));
  DEBUGLN(ADDR);
  eereadwrite(readflag, addr, magic); // magic  
  //eereadwrite(readflag, addr, calibData);
  eereadwrite(readflag, addr, comCalA_1);  
  eereadwrite(readflag, addr, comCalB);  
  eereadwrite(readflag, addr, accelCal);  
  /*eereadwrite(readflag, addr, comAccOfs);
  eereadwrite(readflag, addr, comAccScale);       */
  //  eereadwrite(readflag, addr, calibData);
  DEBUG(F(" endaddr="));
  DEBUGLN(addr);
}

void IMUClass::printCalib(){
  /*  DEBUG(F("Accelerometer: "));
  DEBUG(calibData.accel_offset_x); DEBUG(" ");
  DEBUG(calibData.accel_offset_y); DEBUG(" ");
  DEBUG(calibData.accel_offset_z); DEBUG(" ");

  DEBUG(F("\nGyro: "));
  DEBUG(calibData.gyro_offset_x); DEBUG(" ");
  DEBUG(calibData.gyro_offset_y); DEBUG(" ");
  DEBUG(calibData.gyro_offset_z); DEBUG(" ");

  DEBUG(F("\nMag: "));
  DEBUG(calibData.mag_offset_x); DEBUG(" ");
  DEBUG(calibData.mag_offset_y); DEBUG(" ");
  DEBUG(calibData.mag_offset_z); DEBUG(" ");

  DEBUG(F("\nAccel Radius: "));
  DEBUG(calibData.accel_radius);

  DEBUG(F("\nMag Radius: "));
  DEBUGLN(calibData.mag_radius);*/
  
  DEBUGLN(F("A_1="));
  DEBUG(comCalA_1[0]);
  DEBUG(F("  "));
  DEBUG(comCalA_1[1]);
  DEBUG(F("  "));
  DEBUG(comCalA_1[2]);
  DEBUGLN();
  DEBUG(comCalA_1[3]);
  DEBUG(F("  "));
  DEBUG(comCalA_1[4]);
  DEBUG(F("  "));
  DEBUG(comCalA_1[5]);
  DEBUGLN();
  DEBUG(comCalA_1[6]);
  DEBUG(F("  "));
  DEBUG(comCalA_1[7]);
  DEBUG(F("  "));
  DEBUG(comCalA_1[8]);
  DEBUGLN();  
  DEBUGLN(F("B="));
  DEBUGLN(comCalB[0]);  
  DEBUGLN(comCalB[1]);  
  DEBUGLN(comCalB[2]);    
  
  DEBUG(F("accelCal=")); 
  DEBUG(accelCal[0]);
  DEBUG(F(","));
  DEBUG(accelCal[1]);
  DEBUG(F(","));
  DEBUGLN(accelCal[2]);
  
  /*DEBUG(F("comAccOfs="));
  DEBUG(comAccOfs.x);  
  DEBUG(F(","));
  DEBUG(comAccOfs.y);  
  DEBUG(F(","));
  DEBUGLN(comAccOfs.z);  
  DEBUG(F("comAccScale="));
  DEBUG(comAccScale.x);  
  DEBUG(F(","));
  DEBUG(comAccScale.y);  
  DEBUG(F(","));
  DEBUGLN(comAccScale.z);    */
}

boolean IMUClass::loadCalib(){
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC) {
    DEBUGLN(F("IMU error: no calib data"));
    calibFound = false;
    printCalib();
    return false;   
  } 
  calibFound = true; 
  DEBUGLN(F("IMU: found calib data"));
  loadSaveCalib(true);
  printCalib();  
  return true;
}

void IMUClass::saveCalib(){  
  loadSaveCalib(false);
  calibFound = true;
  printCalib();
  Buzzer.sound(SND_READY, false);    
}

void IMUClass::setNextMode(){
  mode = ((IMUMode) (((byte)mode) + 1));
  if (mode == 2) mode=IMU_MODE_COM_TILT;
  DEBUG(F("IMU mode: "));
  DEBUGLN(mode);  
}


