#include "QGCUASParamManager.h"
#include "UASInterface.h"

QGCUASParamManager::QGCUASParamManager(UASInterface* uas, QWidget *parent) :
    QWidget(parent),
    mav(uas),
    transmissionListMode(false),
    transmissionActive(false),
    transmissionTimeout(0),
    retransmissionTimeout(350),
    rewriteTimeout(500),
#ifdef MAVLINK_ENABLED_SLUGS
    retransmissionBurstRequestSize(1)
#else
    retransmissionBurstRequestSize(2)
#endif
{
    uas->setParamManager(this);
}


/**
 * The .. signal is emitted
 */
void QGCUASParamManager::requestParameterListUpdate(int component)
{

}


