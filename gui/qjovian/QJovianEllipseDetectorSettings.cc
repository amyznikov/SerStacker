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


  hlines_ctl =
      add_numeric_box<std::vector<float>>("hlines",
          [this](const std::vector<float> & v) {
            if ( options_ ) {
              options_->hlines = v;
              emit parameterChanged();
            }
          });

  normalization_scale_ctl =
      add_numeric_box<int>("normalization_scale",
          [this](int v) {
            if ( options_ && options_->normalization_scale != v ) {
              options_->normalization_scale = v;
              emit parameterChanged();
            }
          });

  normalization_blur_ctl =
      add_numeric_box<float>("normalization_blur",
          [this](float v) {
            if ( options_ && options_->normalization_blur != v ) {
              options_->normalization_blur = v;
              emit parameterChanged();
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

    hlines_ctl->setValue(options_->hlines);
    normalization_scale_ctl->setValue(options_->normalization_scale);
    normalization_blur_ctl->setValue(options_->normalization_blur);

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
