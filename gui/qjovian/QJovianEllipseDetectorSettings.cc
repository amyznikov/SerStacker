/*
 * QJovianEllipseDetectorSettings.cc
 *
 *  Created on: Aug 21, 2022
 *      Author: amyznikov
 */

#include "QJovianEllipseDetectorSettings.h"
#include <gui/widgets/QToolbarSpacer.h>
#include <core/settings.h>

#define ICON_download   "download"
#define ICON_upload     "upload"
#define ICON_copy       "copy"
#define ICON_paste      "paste"


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qjovian/icons/%1").arg(name));
}

QJovianEllipseDetectorSettings::QJovianEllipseDetectorSettings(QWidget * parent) :
    Base("QJovianEllipseDetectorSettings", parent)
{

  Q_INIT_RESOURCE(qjovian_resources);

  QAction * action;

  toolbar_ctl =
      add_widget<QToolBar>();

  toolbar_ctl->setToolButtonStyle(
      Qt::ToolButtonStyle::ToolButtonTextBesideIcon);

  toolbar_ctl->setIconSize(QSize(16, 16));

  //toolbar_ctl->addWidget(new QToolbarSpacer());

  toolbar_ctl->addAction(action = new QAction(getIcon(ICON_copy), "Copy"));
  action->setToolTip("Copy parameters to clipboard");
  connect(action, &QAction::triggered, this, &ThisClass::copyParametersToClipboard);

  toolbar_ctl->addAction(action = new QAction(getIcon(ICON_paste), "Paste"));
  action->setToolTip("Paste parameters from clipboard");
  connect(action, &QAction::triggered, this, &ThisClass::pasteParametersFromClipboard);

  stdev_factor_ctl =
      add_numeric_box<double>("stdev_factor",
          "",
          [this](float v) {
            if ( options_ && options_->stdev_factor != v ) {
              options_->stdev_factor = v;
              Q_EMIT parameterChanged();
            }
          });

  force_reference_ellipse_ctl =
      add_checkbox("force reference ellipse",
          "",
          [this](bool checked) {
            if ( options_ && options_->force_reference_ellipse != checked ) {
              options_->force_reference_ellipse = checked;
              Q_EMIT parameterChanged();
            }
          });
}

void QJovianEllipseDetectorSettings::set_jovian_ellipse_detector_options(c_jovian_ellipse_detector_options * options)
{
  options_ = options;
  updateControls();
}

c_jovian_ellipse_detector_options * QJovianEllipseDetectorSettings::jovian_ellipse_detector_options() const
{
  return options_;
}

void QJovianEllipseDetectorSettings::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    stdev_factor_ctl->setValue(options_->stdev_factor);
    force_reference_ellipse_ctl->setChecked(options_->force_reference_ellipse);

    setEnabled(true);
  }
}

void QJovianEllipseDetectorSettings::copyParametersToClipboard()
{
  if ( options_ ) {

//    c_config cfg;
//
//    c_config_setting root =
//        cfg.root();

    //save_settings(cfg, v);


  }

}

void QJovianEllipseDetectorSettings::pasteParametersFromClipboard()
{
  if ( options_ ) {
  }
}
