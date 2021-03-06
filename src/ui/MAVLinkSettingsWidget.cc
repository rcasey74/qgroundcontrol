/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009, 2010 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

This file is part of the QGROUNDCONTROL project

    QGROUNDCONTROL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QGROUNDCONTROL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Implementation of MAVLinkSettingsWidget
 *   @author Lorenz Meier <mail@qgroundcontrol.org>
 */

#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QDesktopServices>

#include "MAVLinkSettingsWidget.h"
#include "ui_MAVLinkSettingsWidget.h"

MAVLinkSettingsWidget::MAVLinkSettingsWidget(MAVLinkProtocol* protocol, QWidget *parent) :
    QWidget(parent),
    protocol(protocol),
    m_ui(new Ui::MAVLinkSettingsWidget)
{
    m_ui->setupUi(this);

    m_ui->gridLayout->setAlignment(Qt::AlignTop);

    // Initialize state
    m_ui->heartbeatCheckBox->setChecked(protocol->heartbeatsEnabled());
    m_ui->loggingCheckBox->setChecked(protocol->loggingEnabled());
    m_ui->versionCheckBox->setChecked(protocol->versionCheckEnabled());

    // Connect actions
    connect(protocol, SIGNAL(heartbeatChanged(bool)), m_ui->heartbeatCheckBox, SLOT(setChecked(bool)));
    connect(m_ui->heartbeatCheckBox, SIGNAL(toggled(bool)), protocol, SLOT(enableHeartbeats(bool)));
    connect(protocol, SIGNAL(loggingChanged(bool)), m_ui->loggingCheckBox, SLOT(setChecked(bool)));
    connect(m_ui->loggingCheckBox, SIGNAL(toggled(bool)), protocol, SLOT(enableLogging(bool)));
    connect(protocol, SIGNAL(versionCheckChanged(bool)), m_ui->versionCheckBox, SLOT(setChecked(bool)));
    connect(m_ui->versionCheckBox, SIGNAL(toggled(bool)), protocol, SLOT(enableVersionCheck(bool)));
    connect(m_ui->logFileButton, SIGNAL(clicked()), this, SLOT(chooseLogfileName()));

    // Update values
    m_ui->versionLabel->setText(tr("MAVLINK_VERSION: %1").arg(protocol->getVersion()));
    updateLogfileName(protocol->getLogfileName());

    // Connect visibility updates
    connect(protocol, SIGNAL(versionCheckChanged(bool)), m_ui->versionLabel, SLOT(setVisible(bool)));
    m_ui->versionLabel->setVisible(protocol->versionCheckEnabled());
    //connect(m_ui->versionCheckBox, SIGNAL(toggled(bool)), m_ui->versionSpacer, SLOT(setVisible(bool)));
    //connect(m_ui->loggingCheckBox, SIGNAL(toggled(bool)), m_ui->logFileSpacer, SLOT(setVisible(bool)));
    connect(protocol, SIGNAL(loggingChanged(bool)), m_ui->logFileLabel, SLOT(setVisible(bool)));
    m_ui->logFileLabel->setVisible(protocol->loggingEnabled());
    connect(protocol, SIGNAL(loggingChanged(bool)), m_ui->logFileButton, SLOT(setVisible(bool)));
    m_ui->logFileButton->setVisible(protocol->loggingEnabled());

    // Update settings
    m_ui->loggingCheckBox->setChecked(protocol->loggingEnabled());
    m_ui->heartbeatCheckBox->setChecked(protocol->heartbeatsEnabled());
    m_ui->versionCheckBox->setChecked(protocol->versionCheckEnabled());
}

void MAVLinkSettingsWidget::updateLogfileName(const QString& fileName)
{
    QFileInfo file(fileName);
    m_ui->logFileLabel->setText(file.fileName());
}

void MAVLinkSettingsWidget::chooseLogfileName()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Specify MAVLink log file name"), QDesktopServices::storageLocation(QDesktopServices::DesktopLocation), tr("MAVLink Logfile (*.mavlink);;"));

    if (!fileName.endsWith(".mavlink"))
    {
        fileName.append(".mavlink");
    }

    QFileInfo file(fileName);
    if (file.exists() && !file.isWritable())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setText(tr("The selected logfile is not writable"));
        msgBox.setInformativeText(tr("Please make sure that the file %1 is writable or select a different file").arg(fileName));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
    }
    else
    {
        updateLogfileName(fileName);
        protocol->setLogfileName(fileName);
    }
}

MAVLinkSettingsWidget::~MAVLinkSettingsWidget()
{
    delete m_ui;
}

void MAVLinkSettingsWidget::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}
