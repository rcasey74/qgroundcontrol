#include "SlugsPadCameraControl.h"
#include "ui_SlugsPadCameraControl.h"

SlugsPadCameraControl::SlugsPadCameraControl(QWidget *parent) :
   QWidget(parent),
    ui(new Ui::SlugsPadCameraControl),
     dragging(0)
{
    ui->setupUi(this);
    x1= 0;
    y1 = 0;
    motion = NONE;

    connect(ui->btZoomIn, SIGNAL(clicked()), this, SLOT(zoomIn()));
    connect(ui->btZoomOut, SIGNAL(clicked()), this, SLOT(zoomOut()));
    connect(ui->btHome, SIGNAL(clicked()), this, SLOT(zoomOut()));
}

SlugsPadCameraControl::~SlugsPadCameraControl()
{
    delete ui;
}

void SlugsPadCameraControl::activeUasSet(UASInterface *uas)
{
    if(uas)
    {
         this->activeUAS= uas;
    }
}

void SlugsPadCameraControl::mouseMoveEvent(QMouseEvent *event)
{
    Q_UNUSED(event);

    if(dragging)
    {
        getDeltaPositionPad(event->x(), event->y());
    }
}

void SlugsPadCameraControl::mousePressEvent(QMouseEvent *event)
{
    if(!dragging)
    {
        dragging = true;
        x1 = event->x();
        y1 = event->y();
    }
}

void SlugsPadCameraControl::mouseReleaseEvent(QMouseEvent *event)
{
    if(dragging)
    {
        dragging = false;
        getDeltaPositionPad(event->x(), event->y());

        xFin = event->x();
        yFin = event->y();
    }
}

void SlugsPadCameraControl::getDeltaPositionPad(int x2, int y2)
{
    QPointF localMeasures = getDistancePixel(y1,x1,y2,x2);

    if(localMeasures.y()>10)
    {
        QString dir = "nd";

        double bearing = localMeasures.x();

        bearing = bearing +90;

        if(bearing>= 360)
        {
            bearing = bearing - 360;
        }

        if(bearing >337.5 || bearing <=22.5)
        {
            motion= UP;
            movePad = QPoint(0, 1);
            dir = "UP";
        }
        else if(bearing >22.5 && bearing <=67.5)
        {
            motion= RIGHT_UP;
            movePad = QPoint(1, 1);
            dir = "RIGHT UP";
        }
        else if(bearing >67.5 && bearing <=112.5)
        {
            motion= RIGHT;
            movePad = QPoint(1, 0);
            dir = "RIGHT";
        }
        else if(bearing >112.5 && bearing <= 157.5)
        {
            motion= RIGHT_DOWN;
            movePad = QPoint(1, -1);
            dir = "RIGHT DOWN";
        }
        else if(bearing >157.5 && bearing <=202.5)
        {
            motion= DOWN;
            movePad = QPoint(0, -1);
            dir = "DOWN";
        }
        else if(bearing >202.5 && bearing <=247.5)
        {
            motion= LEFT_DOWN;
            movePad = QPoint(-1, -1);
            dir = "LEFT DOWN";
        }
        else if(bearing >247.5 && bearing <=292.5)
        {
            motion= LEFT;
            movePad = QPoint(-1, 0);
            dir = "LEFT";
        }
        else if(bearing >292.5 && bearing <=337.5)
        {
            motion= LEFT_UP;
            movePad = QPoint(-1, 1);
            dir = "LEFT UP";
        }

        sendMessageCamera(0, movePad.x(), movePad.y(), 0);

        ui->lbPixel->setText(QString::number(localMeasures.y()));
        ui->lbDirection->setText(dir);

        update();
    }
}

void SlugsPadCameraControl::sendMessageCamera(uint8_t moveHome, uint8_t pan, uint8_t tilt, uint8_t zoom)
{
    if(activeUAS)
    {
        mavlink_message_t msg;
        mavlink_slugs_camera_order_t camera;

        camera.moveHome = moveHome;
        camera.pan = pan;
        camera.tilt = tilt;
        camera.zoom = zoom;
        camera.target = activeUAS->getUASID();

        mavlink_msg_slugs_camera_order_encode(MG::SYSTEM::ID, MG::SYSTEM::COMPID, &msg, &camera);
        UAS* myUas= static_cast<UAS*>(this->activeUAS);
        myUas->sendMessage(msg);
        qDebug()<<"target: "<<camera.target <<" Til: "<<camera.tilt<<" Pan: "<<camera.pan<<" zoom: "<<camera.zoom<<" moveHome"<<camera.moveHome;
    }
}


void SlugsPadCameraControl::zoomIn()
{
    sendMessageCamera(0, 0, 0, 1);
}

void SlugsPadCameraControl::zoomOut()
{
    sendMessageCamera(0, 0, 0, -1);
}

void SlugsPadCameraControl::moveHome()
{
    sendMessageCamera(1, 0, 0, 0);
}

QPointF SlugsPadCameraControl::getDistancePixel(double lon1, double lat1, double lon2, double lat2)
{
    double cateto_opuesto,cateto_adyacente, hipotenusa;//, distancia;
    double marcacion = 0.0;

    //latitude and longitude first point

    if(lat1<0) lat1= lat1*(-1);
    if(lat2<0) lat2= lat2*(-1);
    if(lon1<0) lon1= lon1*(-1);
    if(lon2<0) lon1= lon1*(-1);

    cateto_opuesto = abs((lat1-lat2));
    cateto_adyacente = abs((lon1-lon2));

    hipotenusa = sqrt(pow(cateto_opuesto,2) + pow(cateto_adyacente,2));
    //distancia = hipotenusa*60.0;


    if ((lat1 < lat2) && (lon1 > lon2)) //primer cuadrante
        marcacion = 360 -((asin(cateto_adyacente/hipotenusa))/ 0.017453292);
    else if ((lat1 < lat2) && (lon1 < lon2)) //segundo cuadrante
        marcacion = (asin(cateto_adyacente/hipotenusa))/ 0.017453292;
    else if((lat1 > lat2) && (lon1 < lon2)) //tercer cuadrante
        marcacion = 180 -((asin(cateto_adyacente/hipotenusa))/ 0.017453292);
    else if((lat1 > lat2) && (lon1 > lon2)) //cuarto cuadrante
        marcacion = 180 +((asin(cateto_adyacente/hipotenusa))/ 0.017453292);
    else if((lat1 < lat2) && (lon1 == lon2)) //360
        marcacion = 360;
    else if((lat1 == lat2) && (lon1 > lon2)) //270
        marcacion = 270;
    else if((lat1 > lat2) && (lon1 == lon2)) //180
        marcacion = 180;
    else if((lat1 == lat2) && (lon1 < lon2)) //90
        marcacion =90;
    else if((lat1 == lat2) && (lon1 == lon2)) //0
        marcacion = 0.0;

    return QPointF(marcacion,hipotenusa);// distancia);
}
