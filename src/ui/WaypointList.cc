/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Waypoint list widget
 *
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *   @author Benjamin Knecht <mavteam@student.ethz.ch>
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *
 */

#include "WaypointList.h"
#include "ui_WaypointList.h"
#include <UASInterface.h>
#include <UASManager.h>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QMouseEvent>

WaypointList::WaypointList(QWidget *parent, UASInterface* uas) :
        QWidget(parent),
        uas(NULL),
        mavX(0.0),
        mavY(0.0),
        mavZ(0.0),
        mavYaw(0.0),
        m_ui(new Ui::WaypointList)
{
    m_ui->setupUi(this);

    listLayout = new QVBoxLayout(m_ui->listWidget);
    listLayout->setSpacing(6);
    listLayout->setMargin(0);
    listLayout->setAlignment(Qt::AlignTop);
    m_ui->listWidget->setLayout(listLayout);

    this->uas = NULL;

    // ADD WAYPOINT
    // Connect add action, set right button icon and connect action to this class
    connect(m_ui->addButton, SIGNAL(clicked()), m_ui->actionAddWaypoint, SIGNAL(triggered()));
    connect(m_ui->actionAddWaypoint, SIGNAL(triggered()), this, SLOT(add()));

    // ADD WAYPOINT AT CURRENT POSITION
    connect(m_ui->positionAddButton, SIGNAL(clicked()), this, SLOT(addCurrentPositonWaypoint()));

    // SEND WAYPOINTS
    connect(m_ui->transmitButton, SIGNAL(clicked()), this, SLOT(transmit()));

    // REQUEST WAYPOINTS
    connect(m_ui->readButton, SIGNAL(clicked()), this, SLOT(read()));

    // SAVE/LOAD WAYPOINTS
    connect(m_ui->saveButton, SIGNAL(clicked()), this, SLOT(saveWaypoints()));
    connect(m_ui->loadButton, SIGNAL(clicked()), this, SLOT(loadWaypoints()));

    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setUAS(UASInterface*)));



    // SET UAS AFTER ALL SIGNALS/SLOTS ARE CONNECTED
    setUAS(uas);

    // STATUS LABEL
    updateStatusLabel("");

    this->setVisible(false);
    loadFileGlobalWP = false;
    readGlobalWP = false;
    centerMapCoordinate.setX(0.0);
    centerMapCoordinate.setY(0.0);

}

WaypointList::~WaypointList()
{
    delete m_ui;
}

void WaypointList::updatePosition(UASInterface* uas, double x, double y, double z, quint64 usec)
{
    Q_UNUSED(uas);
    Q_UNUSED(usec);
    mavX = x;
    mavY = y;
    mavZ = z;
}

void WaypointList::updateAttitude(UASInterface* uas, double roll, double pitch, double yaw, quint64 usec)
{
    Q_UNUSED(uas);
    Q_UNUSED(usec);
    Q_UNUSED(roll);
    Q_UNUSED(pitch);
    mavYaw = yaw;
}

void WaypointList::setUAS(UASInterface* uas)
{
    if (this->uas == NULL && uas != NULL)
    {
        this->uas = uas;

        connect(uas, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)),  this, SLOT(updatePosition(UASInterface*,double,double,double,quint64)));
        connect(uas, SIGNAL(attitudeChanged(UASInterface*,double,double,double,quint64)),       this, SLOT(updateAttitude(UASInterface*,double,double,double,quint64)));

        connect(&uas->getWaypointManager(), SIGNAL(updateStatusString(const QString &)),        this, SLOT(updateStatusLabel(const QString &)));
        connect(&uas->getWaypointManager(), SIGNAL(waypointListChanged(void)),                  this, SLOT(waypointListChanged(void)));
        connect(&uas->getWaypointManager(), SIGNAL(currentWaypointChanged(quint16)),            this, SLOT(currentWaypointChanged(quint16)));
        connect(&uas->getWaypointManager(),SIGNAL(loadWPFile()),this,SLOT(setIsLoadFileWP()));
        connect(&uas->getWaypointManager(),SIGNAL(readGlobalWPFromUAS(bool)),this,SLOT(setIsReadGlobalWP(bool)));

    }
}

void WaypointList::saveWaypoints()
{
    if (uas)
    {
        QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "./waypoints.txt", tr("Waypoint File (*.txt)"));
        uas->getWaypointManager().saveWaypoints(fileName);
    }
}

void WaypointList::loadWaypoints()
{
    if (uas)
    {


        QString fileName = QFileDialog::getOpenFileName(this, tr("Load File"), ".", tr("Waypoint File (*.txt)"));
        uas->getWaypointManager().loadWaypoints(fileName);
     }
}

void WaypointList::transmit()
{
    if (uas)
    {
        uas->getWaypointManager().writeWaypoints();
    }
}

void WaypointList::read()
{
    if (uas)
    {
        uas->getWaypointManager().readWaypoints();
    }
}

void WaypointList::add()
{
    if (uas)
    {
//        if(isGlobalWP)
//        {
//            const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();
//            if (waypoints.size() > 0)
//            {
//                Waypoint *last = waypoints.at(waypoints.size()-1);
//                Waypoint *wp = new Waypoint(0, centerMapCoordinate.x(), centerMapCoordinate.y(), last->getZ(), last->getYaw(), last->getAutoContinue(), false, last->getOrbit(), last->getHoldTime());
//                uas->getWaypointManager().addWaypoint(wp);
//            }
//            else
//            {
//                Waypoint *wp = new Waypoint(0, centerMapCoordinate.x(), centerMapCoordinate.y(), -0.8, 0.0, true, true, 0.15, 2000);
//                uas->getWaypointManager().addWaypoint(wp);
//            }
//
//            emit createWaypointAtMap(centerMapCoordinate);
//        }
//        else
        {

            const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();
            Waypoint *wp;


            if (waypoints.size() > 0)
            {
                // Create waypoint with last frame
                Waypoint *last = waypoints.at(waypoints.size()-1);
                wp = new Waypoint(0, last->getX(), last->getY(), last->getZ(), last->getYaw(), last->getAutoContinue(), false, last->getOrbit(),
                                            last->getHoldTime(), last->getFrame(), last->getAction());
                uas->getWaypointManager().addWaypoint(wp);
            }
            else
            {
                    // Create global frame waypoint per default
                    wp = new Waypoint(0, uas->getLongitude(), uas->getLatitude(), uas->getAltitude(), 0.0, true, true, 0.15, 0, MAV_FRAME_GLOBAL, MAV_ACTION_NAVIGATE);
                    uas->getWaypointManager().addWaypoint(wp);
            }
            if (wp->getFrame() == MAV_FRAME_GLOBAL)
            {
                emit createWaypointAtMap(QPointF(wp->getX(), wp->getY()));
            }
        }
    }
}

void WaypointList::addCurrentPositonWaypoint()
{
    /* TODO: implement with new waypoint structure
    if (uas)
    {
        // For Global Waypoints
        //if(isGlobalWP)
        //{
            //isLocalWP = false;
        //}
        //else
        {
            const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();
            if (waypoints.size() > 0)
            {
                Waypoint *last = waypoints.at(waypoints.size()-1);
                Waypoint *wp = new Waypoint(0, (qRound(mavX*100))/100., (qRound(mavY*100))/100., (qRound(mavZ*100))/100., (qRound(mavYaw*100))/100., last->getAutoContinue(), false, last->getOrbit(), last->getHoldTime());
                uas->getWaypointManager().addWaypoint(wp);
            }
            else
            {
                Waypoint *wp = new Waypoint(0, (qRound(mavX*100))/100., (qRound(mavY*100))/100., (qRound(mavZ*100))/100., (qRound(mavYaw*100))/100., true, true, 0.15, 2000);
                uas->getWaypointManager().addWaypoint(wp);
            }

             //isLocalWP = true;
        }
    }
    */
}

void WaypointList::updateStatusLabel(const QString &string)
{
    m_ui->statusLabel->setText(string);


}

void WaypointList::changeCurrentWaypoint(quint16 seq)
{
    if (uas)
    {
        uas->getWaypointManager().setCurrentWaypoint(seq);
    }
}

void WaypointList::currentWaypointChanged(quint16 seq)
{
    if (uas)
    {
        const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();

        if (seq < waypoints.size())
        {
            for(int i = 0; i < waypoints.size(); i++)
            {
                WaypointView* widget = wpViews.find(waypoints[i]).value();

                if (waypoints[i]->getId() == seq)
                {
                    widget->setCurrent(true);
                }
                else
                {
                    widget->setCurrent(false);
                }
            }
        }
    }
}

void WaypointList::waypointListChanged()
{
    if (uas)
    {
        const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();

            // For Global Waypoints
            //if(isGlobalWP)
            //{
                //isLocalWP = false;
                //// first remove all views of non existing waypoints
                //if (!wpGlobalViews.empty())
                //{
                    //QMapIterator<Waypoint*,WaypointGlobalView*> viewIt(wpGlobalViews);
                    //viewIt.toFront();
                    //while(viewIt.hasNext())
                    //{
                        //viewIt.next();
                        //Waypoint *cur = viewIt.key();
                        //int i;
                        //for (i = 0; i < waypoints.size(); i++)
                        //{
                            //if (waypoints[i] == cur)
                            //{
                                //break;
                            //}
                        //}
                        //if (i == waypoints.size())
                        //{
                            //WaypointGlobalView* widget = wpGlobalViews.find(cur).value();
                            //widget->hide();
                            //listLayout->removeWidget(widget);
                            //wpGlobalViews.remove(cur);
                        //}
                    //}
                //}

                //// then add/update the views for each waypoint in the list
                //for(int i = 0; i < waypoints.size(); i++)
                //{
                    //Waypoint *wp = waypoints[i];
                    //if (!wpGlobalViews.contains(wp))
                    //{
                        //WaypointGlobalView* wpview = new WaypointGlobalView(wp, this);
                        //wpGlobalViews.insert(wp, wpview);
                        //connect(wpview, SIGNAL(removeWaypoint(Waypoint*)),      this, SLOT(removeWaypoint(Waypoint*)));
                        //connect(wpview, SIGNAL(changePositionWP(Waypoint*)), this, SLOT(changeWPPositionBySpinBox(Waypoint*)));
////                        connect(wpview, SIGNAL(moveDownWaypoint(Waypoint*)),    this, SLOT(moveDown(Waypoint*)));
////                        connect(wpview, SIGNAL(moveUpWaypoint(Waypoint*)),      this, SLOT(moveUp(Waypoint*)));
                       //// connect(wpview, SIGNAL(changePositionWP(Waypoint*)), this, SLOT(waypointGlobalPositionChanged(Waypoint*)));
////                        connect(wpview, SIGNAL(currentWaypointChanged(quint16)), this, SLOT(currentWaypointChanged(quint16)));
////                        connect(wpview, SIGNAL(changeCurrentWaypoint(quint16)), this, SLOT(changeCurrentWaypoint(quint16)));
                    //}
                //WaypointGlobalView *wpgv = wpGlobalViews.value(wp);
                //wpgv->updateValues();
                //listLayout->addWidget(wpgv);
            //}

            //}
            //else
            {
                // for local Waypoints
                // first remove all views of non existing waypoints
                if (!wpViews.empty())
                {
                    QMapIterator<Waypoint*,WaypointView*> viewIt(wpViews);
                    viewIt.toFront();
                    while(viewIt.hasNext())
                    {
                        viewIt.next();
                        Waypoint *cur = viewIt.key();
                        int i;
                        for (i = 0; i < waypoints.size(); i++)
                        {
                            if (waypoints[i] == cur)
                            {
                                break;
                            }
                        }
                        if (i == waypoints.size())
                        {
                            WaypointView* widget = wpViews.find(cur).value();
                            widget->hide();
                            listLayout->removeWidget(widget);
                            wpViews.remove(cur);
                        }
                    }
                }

                // then add/update the views for each waypoint in the list
                for(int i = 0; i < waypoints.size(); i++)
                {
                    Waypoint *wp = waypoints[i];
                    if (!wpViews.contains(wp))
                    {
                        WaypointView* wpview = new WaypointView(wp, this);
                        wpViews.insert(wp, wpview);
                        connect(wpview, SIGNAL(moveDownWaypoint(Waypoint*)),    this, SLOT(moveDown(Waypoint*)));
                        connect(wpview, SIGNAL(moveUpWaypoint(Waypoint*)),      this, SLOT(moveUp(Waypoint*)));
                        connect(wpview, SIGNAL(removeWaypoint(Waypoint*)),      this, SLOT(removeWaypoint(Waypoint*)));
                        connect(wpview, SIGNAL(currentWaypointChanged(quint16)), this, SLOT(currentWaypointChanged(quint16)));
                        connect(wpview, SIGNAL(changeCurrentWaypoint(quint16)), this, SLOT(changeCurrentWaypoint(quint16)));
                    }
                WaypointView *wpv = wpViews.value(wp);
                wpv->updateValues();    // update the values of the ui elements in the view
                listLayout->addWidget(wpv);

            }
            }


            loadFileGlobalWP = false;
        }


}

void WaypointList::moveUp(Waypoint* wp)
{
    if (uas)
    {
        const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();

        //get the current position of wp in the local storage
        int i;
        for (i = 0; i < waypoints.size(); i++)
        {
            if (waypoints[i] == wp)
                break;
        }

        // if wp was found and its not the first entry, move it
        if (i < waypoints.size() && i > 0)
        {
            uas->getWaypointManager().moveWaypoint(i, i-1);
        }
    }
}

void WaypointList::moveDown(Waypoint* wp)
{
    if (uas)
    {
        const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();

        //get the current position of wp in the local storage
        int i;
        for (i = 0; i < waypoints.size(); i++)
        {
            if (waypoints[i] == wp)
                break;
        }

        // if wp was found and its not the last entry, move it
        if (i < waypoints.size()-1)
        {
            uas->getWaypointManager().moveWaypoint(i, i+1);
        }
    }
}

void WaypointList::removeWaypoint(Waypoint* wp)
{
    if (uas)
    {
        uas->getWaypointManager().removeWaypoint(wp->getId());
    }
}

void WaypointList::changeEvent(QEvent *e)
{
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}



void WaypointList::on_clearWPListButton_clicked()
{


    if (uas)
    {
           emit clearPathclicked();
          const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();
            while(!waypoints.isEmpty())//for(int i = 0; i <= waypoints.size(); i++)
            {
                WaypointView* widget = wpViews.find(waypoints[0]).value();
                widget->remove();
        }
    }
    else
    {
//        if(isGlobalWP)
//        {
//           emit clearPathclicked();
//        }
    }
}

/** @brief Add a waypoint by mouse click over the map */
void WaypointList::addWaypointMouse(QPointF coordinate)
{
    if (uas)
    {
        const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();
        if (waypoints.size() > 0)
        {
            Waypoint *last = waypoints.at(waypoints.size()-1);
            Waypoint *wp = new Waypoint(0, coordinate.x(), coordinate.y(), last->getZ(), last->getYaw(), last->getAutoContinue(), false, last->getOrbit(), last->getHoldTime());
            uas->getWaypointManager().addWaypoint(wp);
        }
        else
        {
            Waypoint *wp = new Waypoint(0, coordinate.x(), coordinate.y(), -0.8, 0.0, true, true, 0.15, 2000);
            uas->getWaypointManager().addWaypoint(wp);
        }


    }

}

/** @brief The MapWidget informs that a waypoint global was changed on the map */

void WaypointList::waypointGlobalChanged(QPointF coordinate, int indexWP)
{
    if (uas)
    {
        const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();
        if (waypoints.size() > 0)
        {
            Waypoint *temp = waypoints.at(indexWP);

            temp->setX(coordinate.x());
            temp->setY(coordinate.y());

            //WaypointGlobalView* widget = wpGlobalViews.find(waypoints[indexWP]).value();
            //widget->updateValues();
        }
    }


}

///** @brief The MapWidget informs that a waypoint global was changed on the map */

//void WaypointList::waypointGlobalPositionChanged(Waypoint* wp)
//{
//    QPointF coordinate;
//    coordinate.setX(wp->getX());
//    coordinate.setY(wp->getY());

//   emit ChangeWaypointGlobalPosition(wp->getId(), coordinate);


//}

void WaypointList::clearWPWidget()
{
  if (uas)
  {
    const QVector<Waypoint *> &waypoints = uas->getWaypointManager().getWaypointList();
      while(!waypoints.isEmpty())//for(int i = 0; i <= waypoints.size(); i++)
      {
          WaypointView* widget = wpViews.find(waypoints[0]).value();
          widget->remove();
      }
  }
    //emit changePositionWPBySpinBox(wp->getId(), wp->getY(), wp->getX());
}

void WaypointList::setIsLoadFileWP()
{
    loadFileGlobalWP = true;
}

void WaypointList::setIsReadGlobalWP(bool value)
{
    // FIXME James Check this
    Q_UNUSED(value);
    // readGlobalWP = value;
}
