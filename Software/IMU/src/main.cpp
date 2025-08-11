#include <Wire.h>
#include <MPU9250.h>
#undef INFO                   // avoid conflict with rosserial Log.h
#include <MadgwickAHRS.h>
#include <ros.h>
#include <std_msgs/Float32.h>

// I²C pins for ESP32-S3
static constexpr int I2C_SDA = 15;
static constexpr int I2C_SCL =  4;
// dummy interrupt pin for library
static constexpr uint8_t INT_PIN = 0;

// sensor + filter
MPU9250 mpu(INT_PIN);
Madgwick filter;

// raw buffers + calibration arrays
int16_t accelCount[3], gyroCount[3], magCount[3];
float magAdjustment[3], magBias[3];

// forward declaration
void calibrateMagBias();

// rosserial node, msgs & pubs
ros::NodeHandle nh;
std_msgs::Float32 yaw_msg, pitch_msg, roll_msg, ax_msg, ay_msg, az_msg, gx_msg, gy_msg, gz_msg;
ros::Publisher yaw_pub  ("imu/yaw",   &yaw_msg);
ros::Publisher pitch_pub("imu/pitch", &pitch_msg);
ros::Publisher roll_pub ("imu/roll",  &roll_msg);
ros::Publisher ax_pub ("imu/accel_x", &ax_msg);
ros::Publisher ay_pub ("imu/accel_y", &ay_msg);
ros::Publisher az_pub ("imu/accel_z", &az_msg);
ros::Publisher gx_pub ("imu/gyro_x", &gx_msg);
ros::Publisher gy_pub ("imu/gyro_y", &gy_msg);
ros::Publisher gz_pub ("imu/gyro_z", &gz_msg);

void setup() {
  // bring up rosserial at 115200 baud
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(yaw_pub);
  nh.advertise(pitch_pub);
  nh.advertise(roll_pub);
  nh.advertise(ax_pub);
  nh.advertise(ay_pub);
  nh.advertise(az_pub);
  nh.advertise(gx_pub);
  nh.advertise(gy_pub);
  nh.advertise(gz_pub);

  // I²C @400 kHz
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // init MPU9250
  mpu.initMPU9250(AFS_2G, GFS_250DPS, 200);
  mpu.initAK8963(MFS_16BITS, 0x02, magAdjustment);
  //calibrateMagBias();

  // start Madgwick filter @200 Hz
  filter.begin(200);
}

void loop() {
  nh.spinOnce();

  // check for new sample
  if (mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    mpu.readAccelData(accelCount);
    mpu.readGyroData (gyroCount);
    mpu.readMagData  (magCount);

    // conversion
    float aRes = mpu.getAres(AFS_2G);
    float gRes = mpu.getGres(GFS_250DPS) * DEG_TO_RAD;
    float mRes = mpu.getMres(MFS_16BITS) * 1e-3f;

    float ax = accelCount[0]*aRes,
          ay = accelCount[1]*aRes,
          az = accelCount[2]*aRes;
    float gx = gyroCount [0]*gRes,
          gy = gyroCount [1]*gRes,
          gz = gyroCount [2]*gRes;
    float mx = magCount  [0]*mRes*magAdjustment[0] - magBias[0],
          my = magCount  [1]*mRes*magAdjustment[1] - magBias[1],
          mz = magCount  [2]*mRes*magAdjustment[2] - magBias[2];

    // AHRS update
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    // publish yaw, pitch, roll
    yaw_msg.data   = filter.getYaw();
    pitch_msg.data = filter.getPitch();
    roll_msg.data  = filter.getRoll();
    ax_msg.data = ax;
    ay_msg.data = ay;
    az_msg.data = az;
    gx_msg.data = gx;
    gy_msg.data = gy;
    gz_msg.data = gz;

    yaw_pub.publish(&yaw_msg);
    pitch_pub.publish(&pitch_msg);
    roll_pub.publish(&roll_msg);
    ax_pub.publish(&ax_msg);
    ay_pub.publish(&ay_msg);
    az_pub.publish(&az_msg);
    gx_pub.publish(&gx_msg);
    gy_pub.publish(&gy_msg);
    gy_pub.publish(&gz_msg);
  }

  // ~200 Hz
  delay(5);
}

void calibrateMagBias() {
  const int N = 300;
  float magMin[3] = {1e3,1e3,1e3}, magMax[3] = {-1e3,-1e3,-1e3};

  for (int i = 0; i < N; i++) {
    while (!(mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)) {}
    mpu.readMagData(magCount);
    float mRes = mpu.getMres(MFS_16BITS) * 1e-3f;
    for (int j = 0; j < 3; j++) {
      float v = magCount[j] * mRes * magAdjustment[j];
      magMin[j] = min(magMin[j], v);
      magMax[j] = max(magMax[j], v);
    }
    delay(20);
  }

  for (int j = 0; j < 3; j++) {
    magBias[j] = (magMax[j] + magMin[j]) * 0.5f;
  }
}

