#include "AccGyroAttitudeObserver.hpp"

AccGyroAttitudeObserver::AccGyroAttitudeObserver(BodyAccProvider* t_acc, BodyRateProvider* t_rate
                                                , block_frequency t_bf) : TimedBlock(t_bf)
{
    m_acc = t_acc;
    m_rate = t_rate;
    m_bf = t_bf;
}

void AccGyroAttitudeObserver::setFilterType(DataFilter* t_pitch_filter, DataFilter* t_roll_filter)
{
    m_pitch_filter = t_pitch_filter;
    m_roll_filter = t_roll_filter;
}

void AccGyroAttitudeObserver::updateSettings(FilterSettings* t_settings, float t_val_threshold)
{
    m_val_threshold = t_val_threshold;
    m_pitch_filter->setFilterSettings(t_settings);
    m_roll_filter->setFilterSettings(t_settings);
}
void AccGyroAttitudeObserver::loopInternal()
{
    AttitudeMsg acc_attitude;
    acc_data = m_acc->getBodyAcceleration();
    gyro_data = m_rate->getBodyRate() * (M_PI/180);
    acc_attitude = getAccAttitude();

    if(fabs(sqrt((acc_data.x*acc_data.x) + (acc_data.y*acc_data.y) + (acc_data.z*acc_data.z)) - grav) > m_val_threshold)
    {
        m_filtered_attitude_temp.pitch = m_pitch_filter->getFilteredData(gyro_data.x);
        m_filtered_attitude_temp.roll = m_roll_filter->getFilteredData(gyro_data.y);
        //std::cout << " 1 " << std::endl;
    }
    else
    {
        m_filtered_attitude_temp.pitch = m_pitch_filter->getFilteredData(acc_attitude.pitch, gyro_data.x);
        m_filtered_attitude_temp.roll = m_roll_filter->getFilteredData(acc_attitude.roll, gyro_data.y);
        //std::cout << " 2 " << std::endl;
    }

    // std::cout << "Acc x: " << acc_data.x << std::endl;
    // std::cout << "Acc y: " << acc_data.y << std::endl;
    // std::cout << "Gyro x: " << gyro_data.x << std::endl;
    // std::cout << "Gyro y: " << gyro_data.y << std::endl;

    m_filtered_attitude = m_filtered_attitude_temp;
}

Vector3D<float> AccGyroAttitudeObserver::getBodyRate()
{
    return m_rate->getBodyRate();
}

AttitudeMsg AccGyroAttitudeObserver::getAttitude()
{
    return m_filtered_attitude;
}

AttitudeMsg AccGyroAttitudeObserver::getAccAttitude()
{
    AttitudeMsg acc_att;
    acc_att.pitch =  atan2( acc_data.y , sqrt((acc_data.x*acc_data.x) + (acc_data.z*acc_data.z)));
    acc_att.roll = atan2( acc_data.x , sqrt((acc_data.y*acc_data.y) + (acc_data.z*acc_data.z)));
    return acc_att;
}