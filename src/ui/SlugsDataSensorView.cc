#include "SlugsDataSensorView.h"
#include "ui_SlugsDataSensorView.h"

#include <UASManager.h>
#include "SlugsMAV.h"

#include <QDebug>

SlugsDataSensorView::SlugsDataSensorView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SlugsDataSensorView)
{
    ui->setupUi(this);

    activeUAS = NULL;

    this->setVisible(false);

    timer =new QTimer(this);
    timer->setInterval(200);
    connect(timer, SIGNAL(timeout()), this, SLOT(refresh()));
    timer->start();
}

SlugsDataSensorView::~SlugsDataSensorView()
{
    delete ui;
}

void SlugsDataSensorView::addUAS(UASInterface* uas)
{
    SlugsMAV* slugsMav = qobject_cast<SlugsMAV*>(uas);

  if (slugsMav != NULL) {

    connect(slugsMav, SIGNAL(slugsRawImu(int, const mavlink_raw_imu_t&)), this, SLOT(slugRawDataChanged(int, const mavlink_raw_imu_t&)));

    #ifdef MAVLINK_ENABLED_SLUGS

    connect(slugsMav, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(slugLocalPositionChanged(UASInterface*,double,double,double,quint64)));
    connect(slugsMav, SIGNAL(speedChanged(UASInterface*,double,double,double,quint64)), this, SLOT(slugSpeedLocalPositionChanged(UASInterface*,double,double,double,quint64)));
    connect(slugsMav, SIGNAL(attitudeChanged(UASInterface*,double,double,double,quint64)), this, SLOT(slugAttitudeChanged(UASInterface*,double,double,double,quint64)));
    connect(slugsMav, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(slugsGlobalPositionChanged(UASInterface*,double,double,double,quint64)));
    connect(slugsMav,SIGNAL(slugsGPSCogSog(int,double,double)),this,SLOT(slugsGPSCogSog(int,double,double)));

    connect(slugsMav, SIGNAL(slugsSensorBias(int,const mavlink_sensor_bias_t&)), this, SLOT(slugsSensorBiasChanged(int,const mavlink_sensor_bias_t&)));
    connect(slugsMav, SIGNAL(slugsDiagnostic(int,const mavlink_diagnostic_t&)), this, SLOT(slugsDiagnosticMessageChanged(int,const mavlink_diagnostic_t&)));
    connect(slugsMav, SIGNAL(slugsCPULoad(int,const mavlink_cpu_load_t&)), this, SLOT(slugsCpuLoadChanged(int,const mavlink_cpu_load_t&)));

    connect(slugsMav, SIGNAL(slugsNavegation(int,const mavlink_slugs_navigation_t&)),this,SLOT(slugsNavegationChanged(int,const mavlink_slugs_navigation_t&)));
    connect(slugsMav, SIGNAL(slugsDataLog(int,const mavlink_data_log_t&)), this, SLOT(slugsDataLogChanged(int,const mavlink_data_log_t&)));
    connect(slugsMav, SIGNAL(slugsGPSDateTime(int,const mavlink_gps_date_time_t&)),this,SLOT(slugsGPSDateTimeChanged(int,const mavlink_gps_date_time_t&)));
    connect(slugsMav,SIGNAL(slugsAirData(int, const mavlink_air_data_t&)),this,SLOT(slugsAirDataChanged(int, const mavlink_air_data_t&)));

    connect(slugsMav, SIGNAL(slugsChannels(int, const mavlink_rc_channels_raw_t&)), this, SLOT(slugsRCRawChannels(int, const mavlink_rc_channels_raw_t&)));
    connect(slugsMav, SIGNAL(slugsServo(int, const mavlink_servo_output_raw_t&)), this, SLOT(slugsRCServo(int,const mavlink_servo_output_raw_t&)));
    connect(slugsMav, SIGNAL(slugsScaled(int, const mavlink_scaled_imu_t&)), this, SLOT(slugsFilteredDataChanged(int, const mavlink_scaled_imu_t&)));


    //connect(slugsMav, SIGNAL(valueChanged(int,QString,QString,double,quint64)), this, SLOT(valueChangedT(int,QString,QString,double,quint64)));

    #endif // MAVLINK_ENABLED_SLUGS
        // Set this UAS as active if it is the first one
    if(activeUAS == 0) {
            activeUAS = uas;
        }

    }
}

void SlugsDataSensorView::refresh()
{
    if(this->isVisible())
    {
        QString str;

        ui->m_Axr->setText(str.sprintf("% 11.4f", (double)rawImu.xacc));
        ui->m_Ayr->setText(str.sprintf("% 11.4f", (double)rawImu.yacc));
        ui->m_Azr->setText(str.sprintf("% 11.4f", (double)rawImu.zacc));
        ui->m_Mxr->setText(str.sprintf("% 11.4f", (double)rawImu.xmag));
        ui->m_Myr->setText(str.sprintf("% 11.4f", (double)rawImu.ymag));
        ui->m_Mzr->setText(str.sprintf("% 11.4f", (double)rawImu.zmag));
        ui->m_Gxr->setText(str.sprintf("% 11.4f", (double)rawImu.xgyro));
        ui->m_Gyr->setText(str.sprintf("% 11.4f", (double)rawImu.ygyro));
        ui->m_Gzr->setText(str.sprintf("% 11.4f", (double)rawImu.zgyro));

        ui->m_Axf->setText(str.sprintf("% 11.4f", (double)scaledImu.xacc/1000.0f));
        ui->m_Ayf->setText(str.sprintf("% 11.4f", (double)scaledImu.yacc/1000.0f));
        ui->m_Azf->setText(str.sprintf("% 11.4f", (double)scaledImu.zacc/1000.0f));
        ui->m_Gxf->setText(str.sprintf("% 11.4f", (double)scaledImu.xgyro/1000.0f));
        ui->m_Gyf->setText(str.sprintf("% 11.4f", (double)scaledImu.ygyro/1000.0f));
        ui->m_Gzf->setText(str.sprintf("% 11.4f", (double)scaledImu.zgyro/1000.0f));
        ui->m_Mxf->setText(str.sprintf("% 11.4f", (double)scaledImu.xmag/1000.0f));
        ui->m_Myf->setText(str.sprintf("% 11.4f", (double)scaledImu.ymag/1000.0f));
        ui->m_Mzf->setText(str.sprintf("% 11.4f", (double)scaledImu.zmag/1000.0f));

        ui->tbRCThrottle->setText(str.sprintf("% 11.4f", (double)channelsRaw.chan1_raw));
        ui->tbRCAileron->setText(str.sprintf("% 11.4f", (double)channelsRaw.chan2_raw));
        ui->tbRCRudder->setText(str.sprintf("% 11.4f", (double)channelsRaw.chan3_raw));
        ui->tbRCElevator->setText(str.sprintf("% 11.4f", (double)channelsRaw.chan4_raw));

        ui->m_pwmThro->setText(str.sprintf("% 11.4f", (double)servoOutputRaw.servo1_raw));
        ui->m_pwmAile->setText(str.sprintf("% 11.4f", (double)servoOutputRaw.servo2_raw));
        ui->m_pwmRudd->setText(str.sprintf("% 11.4f", (double)servoOutputRaw.servo3_raw));
        ui->m_pwmElev->setText(str.sprintf("% 11.4f", (double)servoOutputRaw.servo4_raw));

        ui->m_pwmThroTrim->setText(str.sprintf("% 11.4f", (double)servoOutputRaw.servo5_raw));
        ui->m_pwmAileTrim->setText(str.sprintf("% 11.4f", (double)servoOutputRaw.servo6_raw));
        ui->m_pwmRuddTrim->setText(str.sprintf("% 11.4f", (double)servoOutputRaw.servo7_raw));
        ui->m_pwmElevTrim->setText(str.sprintf("% 11.4f", (double)servoOutputRaw.servo8_raw));

        ui->m_Um->setText(str.sprintf("% 11.4f", (double)slugsNavigation.u_m));
        ui->m_PhiC->setText(str.sprintf("% 11.4f", (double)slugsNavigation.phi_c));
        ui->m_PitchC->setText(str.sprintf("% 11.4f", (double)slugsNavigation.theta_c));
        ui->m_PsidC->setText(str.sprintf("% 11.4f", (double)slugsNavigation.psiDot_c));
        ui->m_AyBody->setText(str.sprintf("% 11.4f", (double)slugsNavigation.ay_body));
        ui->m_TotRun->setText(str.sprintf("% 11.4f", (double)slugsNavigation.totalDist));
        ui->m_DistToGo->setText(str.sprintf("% 11.4f", (double)slugsNavigation.dist2Go));
        ui->m_FromWP->setText(str.sprintf("% 11.4f", (double)slugsNavigation.fromWP));
        ui->m_ToWP->setText(str.sprintf("% 11.4f", (double)slugsNavigation.toWP));

        ui->m_AxBiases->setText(str.sprintf("% 11.4f", (double)sensorBias.axBias));
        ui->m_AyBiases->setText(str.sprintf("% 11.4f", (double)sensorBias.ayBias));
        ui->m_AzBiases->setText(str.sprintf("% 11.4f", (double)sensorBias.azBias));
        ui->m_GxBiases->setText(str.sprintf("% 11.4f", (double)sensorBias.gxBias));
        ui->m_GyBiases->setText(str.sprintf("% 11.4f", (double)sensorBias.gyBias));
        ui->m_GzBiases->setText(str.sprintf("% 11.4f", (double)sensorBias.gzBias));

        ui->m_Fl1->setText(str.sprintf("% 11.4f", (double)diagnostic.diagFl1));
        ui->m_Fl2->setText(str.sprintf("% 11.4f", (double)diagnostic.diagFl2));
        ui->m_Fl3->setText(str.sprintf("% 11.4f", (double)diagnostic.diagFl2));

        ui->m_Sh1->setText(str.sprintf("% 11.4f", (double)diagnostic.diagSh1));
        ui->m_Sh2->setText(str.sprintf("% 11.4f", (double)diagnostic.diagSh2));
        ui->m_Sh3->setText(str.sprintf("% 11.4f", (double)diagnostic.diagSh3));

        ui->m_logFl1->setText(str.sprintf("% 11.4f", (double)dataLog.fl_1));
        ui->m_logFl2->setText(str.sprintf("% 11.4f", (double)dataLog.fl_2));
        ui->m_logFl3->setText(str.sprintf("% 11.4f", (double)dataLog.fl_3));
        ui->m_logFl4->setText(str.sprintf("% 11.4f", (double)dataLog.fl_4));
        ui->m_logFl5->setText(str.sprintf("% 11.4f", (double)dataLog.fl_5));
        ui->m_logFl6->setText(str.sprintf("% 11.4f", (double)dataLog.fl_6));

        ui->m_GpsLatitude->setText(str.sprintf("% 11.4f", latitude));
        ui->m_GpsLongitude->setText(str.sprintf("% 11.4f", longitude));
        ui->m_GpsHeight->setText(str.sprintf("% 11.4f", altitude));

        ui->ed_x->setText(str.sprintf("% 11.4f", ed_x));
        ui->ed_y->setText(str.sprintf("% 11.4f", ed_y));
        ui->ed_z->setText(str.sprintf("% 11.4f", ed_z));

        ui->ed_vx->setText(str.sprintf("% 11.4f", ed_vx));
        ui->ed_vy->setText(str.sprintf("% 11.4f", ed_vy));
        ui->ed_vz->setText(str.sprintf("% 11.4f", ed_vz));

        ui->m_Roll->setText(str.sprintf("% 11.4f", roll));
        ui->m_Pitch->setText(str.sprintf("% 11.4f", pitch));
        ui->m_Yaw->setText(str.sprintf("% 11.4f", yaw));

        updateGpsDateTime();
    }
}

void SlugsDataSensorView::slugRawDataChanged(int uasId, const mavlink_raw_imu_t &rawData)
{
    Q_UNUSED(uasId);

    this->rawImu = rawData;
}

void SlugsDataSensorView::slugsRCRawChannels(int systemId, const mavlink_rc_channels_raw_t &gpsDateTime)
{
    Q_UNUSED(systemId);

    this->channelsRaw = gpsDateTime;
}

void SlugsDataSensorView::slugsRCServo(int systemId, const mavlink_servo_output_raw_t &gpsDateTime)
{
    Q_UNUSED(systemId);

    this->servoOutputRaw = gpsDateTime;
}

void SlugsDataSensorView::setActiveUAS(UASInterface* uas)
{
    activeUAS = uas;
    addUAS(activeUAS);
}

#ifdef MAVLINK_ENABLED_SLUGS

void SlugsDataSensorView::slugsGlobalPositionChanged(UASInterface *uas,
                                                     double lat,
                                                     double lon,
                                                     double alt,
                                                     quint64 time) {
 Q_UNUSED(uas);
 Q_UNUSED(time);

    this->latitude = lat;
    this->longitude = lon;
    this->altitude = alt;

}


void SlugsDataSensorView::slugLocalPositionChanged(UASInterface* uas,
                                                   double x,
                                                   double y,
                                                   double z,
                                                   quint64 time) {
  Q_UNUSED(uas);
  Q_UNUSED(time);

    this->ed_x = x;
    this->ed_y = y;
    this->ed_z = z;



}

void SlugsDataSensorView::slugSpeedLocalPositionChanged(UASInterface* uas,
                                                        double vx,
                                                        double vy,
                                                        double vz,
                                                        quint64 time) {
    Q_UNUSED( uas);
  Q_UNUSED(time);

    this->ed_vx = vx;
    this->ed_vy = vy;
    this->ed_vz = vz;


  //qDebug()<<"Speed Local Position = "<<vx<<" - "<<vy<<" - "<<vz;


}

void SlugsDataSensorView::slugAttitudeChanged(UASInterface* uas,
                                              double slugroll,
                                              double slugpitch,
                                              double slugyaw,
                                              quint64 time)
{
    Q_UNUSED( uas);
    Q_UNUSED(time);

    this->roll = slugroll;
    this->pitch = slugpitch;
    this->yaw = slugyaw;


    // qDebug()<<"Attitude change = "<<slugroll<<" - "<<slugpitch<<" - "<<slugyaw;

}


void SlugsDataSensorView::slugsSensorBiasChanged(int systemId, const mavlink_sensor_bias_t& sensorBias)
{
    Q_UNUSED( systemId);

    this->sensorBias = sensorBias;
}

void SlugsDataSensorView::slugsDiagnosticMessageChanged(int systemId, const mavlink_diagnostic_t& diagnostic)
{
    Q_UNUSED(systemId);

    this->diagnostic = diagnostic;
}


void SlugsDataSensorView::slugsCpuLoadChanged(int systemId,
                                              const mavlink_cpu_load_t& cpuLoad){
     Q_UNUSED(systemId);
  ui->ed_sens->setText(QString::number(cpuLoad.sensLoad));
  ui->ed_control->setText(QString::number(cpuLoad.ctrlLoad));
  ui->ed_batvolt->setText(QString::number(cpuLoad.batVolt));
}

void SlugsDataSensorView::slugsNavegationChanged(int systemId,
                                                 const mavlink_slugs_navigation_t& slugsNavigation){
     Q_UNUSED(systemId);

    this->slugsNavigation = slugsNavigation;

}



void SlugsDataSensorView::slugsDataLogChanged(int systemId, const mavlink_data_log_t& dataLog)
{
    Q_UNUSED(systemId);

    this->dataLog = dataLog;

}

void SlugsDataSensorView::slugsFilteredDataChanged(int systemId, const mavlink_scaled_imu_t& filteredData){
    Q_UNUSED(systemId);        

    scaledImu = filteredData;
}

void SlugsDataSensorView::slugsGPSDateTimeChanged(int systemId, const mavlink_gps_date_time_t& gpsDateTime)
{
    Q_UNUSED(systemId);

    this->gpsDateTime = gpsDateTime;
}

void SlugsDataSensorView::updateGpsDateTime()
{
    QString month, day;

    month = QString::number(gpsDateTime.month);
    day = QString::number(gpsDateTime.day);

    if(gpsDateTime.month < 10) month = "0" + QString::number(gpsDateTime.month);
    if(gpsDateTime.day < 10) day = "0" + QString::number(gpsDateTime.day);


    ui->m_GpsDate->setText(day + "/" +
                         month + "/" +
                         QString::number(gpsDateTime.year));

    QString hour, min, sec;

    hour = QString::number(gpsDateTime.hour);
    min =  QString::number(gpsDateTime.min);
    sec =  QString::number(gpsDateTime.sec);

    if(gpsDateTime.hour < 10) hour = "0" + QString::number(gpsDateTime.hour);
    if(gpsDateTime.min < 10) min = "0" + QString::number(gpsDateTime.min);
    if(gpsDateTime.sec < 10) sec = "0" + QString::number(gpsDateTime.sec);

    ui->m_GpsTime->setText(hour + ":" +
                           min + ":" +
                           sec);

  ui->m_GpsSat->setText(QString::number(gpsDateTime.visSat));
}

/**
     * @brief Updates the air data widget - 171
*/
void SlugsDataSensorView::slugsAirDataChanged(int systemId, const mavlink_air_data_t &airData)
{
     Q_UNUSED(systemId);
     ui->ed_dynamic->setText(QString::number(airData.dynamicPressure));
     ui->ed_static->setText(QString::number(airData.staticPressure));
     ui->ed_temp->setText(QString::number(airData.temperature));
}

/**
     * @brief set COG and SOG values
     *
     * COG and SOG GPS display on the Widgets
*/
void SlugsDataSensorView::slugsGPSCogSog(int systemId, double cog, double sog)
{
     Q_UNUSED(systemId);

     ui->m_GpsCog->setText(QString::number(cog));
     ui->m_GpsSog->setText(QString::number(sog));
}

#endif // MAVLINK_ENABLED_SLUGS
