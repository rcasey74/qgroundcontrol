#ifndef QGCMAVLINKLOGPLAYER_H
#define QGCMAVLINKLOGPLAYER_H

#include <QWidget>
#include <QFile>

#include "MAVLinkProtocol.h"
#include "MAVLinkSimulationLink.h"

namespace Ui {
    class QGCMAVLinkLogPlayer;
}

/**
 * @brief Replays MAVLink log files
 *
 * This class allows to replay MAVLink logs at varying speeds.
 * captured flights can be replayed, shown to others and analyzed
 * in-depth later on.
 */
class QGCMAVLinkLogPlayer : public QWidget
{
    Q_OBJECT

public:
    explicit QGCMAVLinkLogPlayer(MAVLinkProtocol* mavlink, QWidget *parent = 0);
    ~QGCMAVLinkLogPlayer();

public slots:
    /** @brief Replay the logfile */
    void play();
    /** @brief Pause the logfile */
    void pause();
    /** @brief Reset the logfile */
    void reset();
    /** @brief Select logfile */
    void selectLogFile();
    /** @brief Load log file */
    void loadLogFile(const QString& file);
    /** @brief The logging mainloop */
    void logLoop();
    /** @brief Set acceleration factor in percent */
    void setAccelerationFactorInt(int factor);

protected:
    int lineCounter;
    int totalLines;
    quint64 startTime;
    quint64 endTime;
    quint64 currentStartTime;
    float accelerationFactor;
    MAVLinkProtocol* mavlink;
    MAVLinkSimulationLink* logLink;
    QFile logFile;
    QTimer loopTimer;
    int loopCounter;
    void changeEvent(QEvent *e);

private:
    Ui::QGCMAVLinkLogPlayer *ui;
};

#endif // QGCMAVLINKLOGPLAYER_H
