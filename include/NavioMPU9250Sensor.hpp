#pragma once 

#include "NineAxisSensor.hpp"
#include "Common/MPU9250.h"
#include "NavioMPU9250Acc.hpp"
#include "NavioMPU9250Gyro.hpp"
#include "NavioMPU9250Mag.hpp"
#include "Timer.hpp"

//TODO: this assumes modifactions to the navio_library, don't use with the original one
class NAVIOMPU9250_sensor : public NineAxisSensor
{
public:

	NAVIOMPU9250_sensor();

	Acc* getAcc();
	Gyro* getGyro();
	Mag* getMag();
	void setSettings(sens_type, setting_type, int);
	void updateReadings();
	Vector3D<int> getAccelReadings();
	Vector3D<int> getGyroReadings();
	Vector3D<int> getMagReadings();


private:

	//int m_samplerate = 1000;
	int _dt = 1000;
	Vector3D<int> m_acc, m_gyro, m_mag;
	Timer _timer;

	MPU9250 _imu;
	ThreeAxisSensor* _acc = new NAVIOMPU9250_acc(this);
	ThreeAxisSensor* _gyro = new NAVIOMPU9250_gyro(this);
	ThreeAxisSensor* _mag = new NAVIOMPU9250_mag(this);
};