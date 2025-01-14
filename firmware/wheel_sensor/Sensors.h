#ifndef SENSORS_H_
#define SENSORS_H_

#include <Wire.h>

#include <ASM330LHHSensor.h>
#include <vl53l4cd_class.h>
#include "FIFO.h"
#include "Board.h"

// default IMU configuration
#define DEFAULT_GYRO_FS       2000    // deg/s
#define DEFAULT_ACCEL_FS      4       // g
#define DEFAULT_IMU_ODR       208.0   // Hz
#define DEFAULT_LIDAR_PERIOD  50      // ms

#define LIDAR_COUNT           3

SPIClass spi(VSPI);
ASM330LHHSensor imu(&spi, SPI_CS);
VL53L4CD lidar1(&Wire, TLXS1);
VL53L4CD lidar2(&Wire, TLXS2);
VL53L4CD lidar3(&Wire, TLXS3);
VL53L4CD* lidars[3];


int lidarInterruptPins[3] = {TLGP1, TLGP2, TLGP3};

struct Sample
{
  int64_t tim;
  float temperature;
  float gyro[3];
  float accel[3];
  float distance[3];
  float distanceErr[3];
  // TODO
};

struct SensorSettings
{
  int32_t gyroFS;
  int32_t accelFS;
  float imuODR;
  uint32_t lidarPeriod;
};

SensorSettings sensorSettings = {DEFAULT_GYRO_FS, DEFAULT_ACCEL_FS, DEFAULT_IMU_ODR, DEFAULT_LIDAR_PERIOD};
volatile bool sensorSettingsChanged = true; // sets config for the first time

Sample sample;
FIFO<Sample, 8> fifo;

int64_t getTimeMicros()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

void initSensors()
{
  spi.begin(SPI_CLK, SPI_SDO, SPI_SDI, SPI_CS);
  pinMode(SPI_CS, OUTPUT);

  pinMode(IMU_INT1, INPUT);
  pinMode(IMU_INT2, INPUT);
  pinMode(TLGP1, INPUT_PULLUP);
  pinMode(TLGP2, INPUT_PULLUP);
  pinMode(TLGP3, INPUT_PULLUP);

  if(imu.begin() != ASM330LHH_OK)
    Serial.println("failed to init IMU.");

  imu.Set_G_FS(sensorSettings.gyroFS);
  imu.Set_X_FS(sensorSettings.accelFS);
  imu.Set_G_ODR(sensorSettings.imuODR);
  imu.Set_X_ODR(sensorSettings.imuODR);

  if(imu.Enable_X() != ASM330LHH_OK)
    Serial.println("failed to enable accel.");

  if(imu.Enable_G() != ASM330LHH_OK)
    Serial.println("failed to enable gyro.");

  if(imu.Enable_INT1(ASM330LHH_INT_DRDY_G) != ASM330LHH_OK)
    Serial.println("failed to setup INT1 pin.");

  if(imu.Enable_INT2(ASM330LHH_INT_DRDY_XL) != ASM330LHH_OK)
    Serial.println("failed to setup INT2 pin.");

  imu.Get_G_FS(&sensorSettings.gyroFS);
  imu.Get_X_FS(&sensorSettings.accelFS);
  imu.Get_X_ODR(&sensorSettings.imuODR);

  Wire.begin();

  // begin() will power-off lidar
  lidar1.begin();
  lidar2.begin();
  lidar3.begin();

  lidars[0] = &lidar1;
  lidars[1] = &lidar2;
  lidars[2] = &lidar3;

  for(int i=0; i < LIDAR_COUNT; i++)
  {
    VL53L4CD* lid = lidars[i];
    lid->InitSensor(0x55 + i);
    lid->VL53L4CD_SetRangeTiming(sensorSettings.lidarPeriod, 0); // 200 Hz
    lid->VL53L4CD_StartRanging();
  }
}

// task for reading sensors
void runSensors(void* args)
{
  while(true)
  {
    if(sensorSettingsChanged)
    {
      initSensors();
      sensorSettingsChanged = false;
    }

    for(int i=0; i < LIDAR_COUNT; i++)
    {
      if(!digitalRead(lidarInterruptPins[i]))
      {
        VL53L4CD_Result_t res;
        VL53L4CD* lid = lidars[i];
        lid->VL53L4CD_ClearInterrupt();
        if(lid->VL53L4CD_GetResult(&res) == VL53L4CD_ERROR_NONE)
        {
          if(res.range_status == 0)
          {
            // data valid
            sample.distance[i] = (float)res.distance_mm;
            sample.distanceErr[i] = (float)res.sigma_mm;
          }
          else
          {
            // data invalid
            sample.distance[i] = 0;
            sample.distanceErr[i] = 0;
          }
        }
      }
    }

    if(digitalRead(IMU_INT2))
    {
      // accelerometer ready
      imu.Get_X_Axes(sample.accel);
    }

    if(digitalRead(IMU_INT1))
    {
      // gyroscope ready, finish sample
      sample.tim = getTimeMicros();
      imu.Get_G_Axes(sample.gyro);
      imu.Get_T(&sample.temperature);

      if(sample.tim > 1000000000000000UL) // about year 2001, wait for NTP sync
        fifo.push(sample);
    }

    taskYIELD();
  }
}

#endif