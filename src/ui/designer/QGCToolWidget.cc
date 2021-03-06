#include "QGCToolWidget.h"
#include "ui_QGCToolWidget.h"

#include <QMenu>
#include <QList>
#include <QInputDialog>
#include <QDockWidget>
#include <QContextMenuEvent>
#include <QSettings>

#include "QGCParamSlider.h"
#include "QGCActionButton.h"
#include "UASManager.h"

QGCToolWidget::QGCToolWidget(const QString& title, QWidget *parent) :
        QWidget(parent),
        mav(NULL),
        mainMenuAction(NULL),
        ui(new Ui::QGCToolWidget)
{
    ui->setupUi(this);
    setObjectName(title);
    createActions();
    toolLayout = ui->toolLayout;
    toolLayout->setAlignment(Qt::AlignTop);

    QDockWidget* dock = dynamic_cast<QDockWidget*>(this->parentWidget());
    if (dock)
    {
        dock->setWindowTitle(title);
        dock->setObjectName(title+"DOCK");
    }

    // Try with parent
    dock = dynamic_cast<QDockWidget*>(this->parentWidget());
    if (dock)
    {
        dock->setWindowTitle(title);
        dock->setObjectName(title+"DOCK");
    }

    this->setWindowTitle(title);

    QList<UASInterface*> systems = UASManager::instance()->getUASList();
    foreach (UASInterface* uas, systems)
    {
        UAS* newMav = dynamic_cast<UAS*>(uas);
        if (newMav)
        {
            addUAS(uas);
        }
    }
    connect(UASManager::instance(), SIGNAL(UASCreated(UASInterface*)), this, SLOT(addUAS(UASInterface*)));
    if (!instances()->contains(title)) instances()->insert(title, this);
}

QGCToolWidget::~QGCToolWidget()
{
    delete ui;
}

/**
 * @param parent Object later holding these widgets, usually the main window
 * @return List of all widgets
 */
QList<QGCToolWidget*> QGCToolWidget::createWidgetsFromSettings(QWidget* parent)
{
    // Store list of widgets
    QSettings settings;
    QList<QGCToolWidget*> newWidgets;
    int size = settings.beginReadArray("QGC_TOOL_WIDGET_NAMES");
    for (int i = 0; i < size; i++)
    {
        settings.setArrayIndex(i);
        QString name = settings.value("TITLE", tr("UNKNOWN WIDGET %1").arg(i)).toString();

        if (!instances()->contains(name))
        {
            QGCToolWidget* tool = new QGCToolWidget(name, parent);
            instances()->insert(name, tool);
            newWidgets.append(tool);
        }
    }
    settings.endArray();

    qDebug() << "NEW WIDGETS: " << newWidgets.size();

    // Load individual widget items
    for (int i = 0; i < newWidgets.size(); i++)
    {
        QString widgetName = newWidgets.at(i)->getTitle();
        qDebug() << "READING: " << widgetName;
        settings.beginGroup(widgetName);
        int size = settings.beginReadArray("QGC_TOOL_WIDGET_ITEMS");
        qDebug() << "CHILDREN SIZE:" << size;
        for (int j = 0; j < size; j++)
        {
            settings.setArrayIndex(j);
            QString type = settings.value("TYPE", "UNKNOWN").toString();
            if (type != "UNKNOWN")
            {
                QGCToolWidgetItem* item = NULL;
                if (type == "BUTTON")
                {
                    item = new QGCActionButton(newWidgets.at(i));
                    qDebug() << "CREATED BUTTON";
                }

                if (item)
                {
                    // Configure and add to layout
                    newWidgets.at(i)->addToolWidget(item);
                    item->readSettings(settings);

                    qDebug() << "Created tool widget";
                }
            }
            else
            {
                qDebug() << "UNKNOWN TOOL WIDGET TYPE";
            }
        }
        settings.endArray();
        settings.endGroup();
    }

    return instances()->values();
}

void QGCToolWidget::storeWidgetsToSettings()
{
    // Store list of widgets
    QSettings settings;
    settings.beginWriteArray("QGC_TOOL_WIDGET_NAMES");
    for (int i = 0; i < instances()->size(); ++i)
    {
        settings.setArrayIndex(i);
        settings.setValue("TITLE", instances()->values().at(i)->getTitle());
    }
    settings.endArray();

    // Store individual widget items
    for (int i = 0; i < instances()->size(); ++i)
    {
        QString widgetName = instances()->values().at(i)->getTitle();
        settings.beginGroup(widgetName);
        settings.beginWriteArray("QGC_TOOL_WIDGET_ITEMS");
        int k = 0; // QGCToolItem counter
        for (int j = 0; j  < instances()->values().at(i)->children().size(); ++j)
        {
            // Store only QGCToolWidgetItems
            QGCToolWidgetItem* item = dynamic_cast<QGCToolWidgetItem*>(instances()->values().at(i)->children().at(j));
            if (item)
            {
                settings.setArrayIndex(k++);
                // Store the ToolWidgetItem
                item->writeSettings(settings);
            }
        }
        settings.endArray();
        settings.endGroup();
    }
}

void QGCToolWidget::addUAS(UASInterface* uas)
{
    UAS* newMav = dynamic_cast<UAS*>(uas);
    if (newMav)
    {
        // FIXME Convert to list
        if (mav == NULL) mav = newMav;
    }
}

void QGCToolWidget::contextMenuEvent (QContextMenuEvent* event)
{
    QMenu menu(this);
    //menu.addAction(addParamAction);
    menu.addAction(addButtonAction);
    menu.addAction(setTitleAction);
    menu.addAction(deleteAction);
    menu.exec(event->globalPos());
}

void QGCToolWidget::createActions()
{
    addParamAction = new QAction(tr("New &Parameter Slider"), this);
    addParamAction->setStatusTip(tr("Add a parameter setting slider widget to the tool"));
    connect(addParamAction, SIGNAL(triggered()), this, SLOT(addParam()));

    addButtonAction = new QAction(tr("New MAV &Action Button"), this);
    addButtonAction->setStatusTip(tr("Add a new action button to the tool"));
    connect(addButtonAction, SIGNAL(triggered()), this, SLOT(addAction()));

    setTitleAction = new QAction(tr("Set Widget Title"), this);
    setTitleAction->setStatusTip(tr("Set the title caption of this tool widget"));
    connect(setTitleAction, SIGNAL(triggered()), this, SLOT(setTitle()));

    deleteAction = new QAction(tr("Delete this widget"), this);
    deleteAction->setStatusTip(tr("Delete this widget permanently"));
    connect(deleteAction, SIGNAL(triggered()), this, SLOT(deleteWidget()));
}

QMap<QString, QGCToolWidget*>* QGCToolWidget::instances()
{
    static QMap<QString, QGCToolWidget*>* instances;
    if (!instances) instances = new QMap<QString, QGCToolWidget*>();
    return instances;
}

QList<QGCToolWidgetItem*>* QGCToolWidget::itemList()
{
    static QList<QGCToolWidgetItem*>* instances;
    if (!instances) instances = new QList<QGCToolWidgetItem*>();
    return instances;
}

void QGCToolWidget::addParam()
{
    QGCParamSlider* slider = new QGCParamSlider(this);
    if (ui->hintLabel)
    {
        ui->hintLabel->deleteLater();
    }
    toolLayout->addWidget(slider);
    slider->startEditMode();
}

void QGCToolWidget::addAction()
{
    QGCActionButton* button = new QGCActionButton(this);
    if (ui->hintLabel)
    {
        ui->hintLabel->deleteLater();
    }
    toolLayout->addWidget(button);
    button->startEditMode();
}

void QGCToolWidget::addToolWidget(QGCToolWidgetItem* widget)
{
    if (ui->hintLabel)
    {
        ui->hintLabel->deleteLater();
    }
    toolLayout->addWidget(widget);
}

const QString QGCToolWidget::getTitle()
{
    QDockWidget* parent = dynamic_cast<QDockWidget*>(this->parentWidget());
    if (parent)
    {
        return parent->windowTitle();
    }
    else
    {
        return this->windowTitle();
    }
}


void QGCToolWidget::setTitle()
{
    QDockWidget* parent = dynamic_cast<QDockWidget*>(this->parentWidget());
    if (parent)
    {
        bool ok;
        QString text = QInputDialog::getText(this, tr("QInputDialog::getText()"),
                                             tr("Widget title:"), QLineEdit::Normal,
                                             parent->windowTitle(), &ok);
        if (ok && !text.isEmpty())
        {
            QSettings settings;
            settings.beginGroup(parent->windowTitle());
            settings.remove("");
            settings.endGroup();
            parent->setWindowTitle(text);

            storeWidgetsToSettings();
            emit titleChanged(text);
            if (mainMenuAction) mainMenuAction->setText(text);
        }
    }
}

void QGCToolWidget::setMainMenuAction(QAction* action)
{
    this->mainMenuAction = action;
}

void QGCToolWidget::deleteWidget()
{
    // Remove from settings

    // Hide
    this->hide();
    instances()->remove(getTitle());
    QSettings settings;
    settings.beginGroup(getTitle());
    settings.remove("");
    settings.endGroup();
    storeWidgetsToSettings();

    // Delete
    mainMenuAction->deleteLater();
    this->deleteLater();
}
