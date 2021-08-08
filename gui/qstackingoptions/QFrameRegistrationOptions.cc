/*
 * QFrameRegistrationSettings.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QFrameRegistrationOptions.h"

#include <gui/widgets/addctrl.h>
#include <gui/widgets/settings.h>
#include <core/debug.h>

#define ICON_check_all      "check_all"

static const char borderless_style[] = ""
    "QToolButton { border: none; } "
    "QToolButton::menu-indicator { image: none; }"
    "";

static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qstackingoptions/icons/%1").arg(name));
}


QString toString(enum frame_registration_method m)
{
  return toStdString(m).c_str();
}

enum frame_registration_method fromString(const QString  & s,
    enum frame_registration_method defval )
{
  return fromStdString(s.toStdString(), defval);
}

QString toString(enum ECC_MOTION_TYPE v)
{
  return toStdString(v).c_str();
}

enum ECC_MOTION_TYPE fromString(const QString & s, enum ECC_MOTION_TYPE defval)
{
  return fromStdString(s.toStdString(), defval);
}


QString toString(enum color_channel_type v)
{
  return toStdString(v).c_str();
}

enum color_channel_type fromString(const QString  & s, enum color_channel_type defval )
{
  return fromStdString(s.toStdString(), defval);
}


QEccSettings::QEccSettings(QWidget * parent)
  : Base("QEccSettings", parent)
{

  scale_ctl = add_numeric_box(form, "scale",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(scale_ctl->text(), &value) && value != options_->scale ) {
            LOCK();
            options_->scale = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  eps_ctl = add_numeric_box(form, "eps",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(eps_ctl->text(), &value) && value != options_->eps ) {
            LOCK();
            options_->eps = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  min_rho_ctl = add_numeric_box(form, "min_rho",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(min_rho_ctl->text(), &value) && value != options_->min_rho ) {
            LOCK();
            options_->min_rho = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  input_smooth_sigma_ctl = add_numeric_box(form, "input_smooth_sigma",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(input_smooth_sigma_ctl->text(), &value) && value != options_->input_smooth_sigma ) {
            LOCK();
            options_->input_smooth_sigma = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  reference_smooth_sigma_ctl = add_numeric_box(form, "reference_smooth_sigma",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(reference_smooth_sigma_ctl->text(), &value) && value != options_->reference_smooth_sigma ) {
            LOCK();
            options_->reference_smooth_sigma = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  update_step_scale_ctl = add_numeric_box(form, "update_step_scale",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(update_step_scale_ctl->text(), &value) && value != options_->update_step_scale ) {
            LOCK();
            options_->update_step_scale = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  normalization_scale_ctl = add_numeric_box(form, "normalization_scale",
      [this]() {
        if ( options_ && !updatingControls() ) {
          int value;
          if ( fromString(normalization_scale_ctl->text(), &value) && value != options_->normalization_scale ) {
            LOCK();
            options_->normalization_scale = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  normalization_noise_ctl = add_numeric_box(form, "normalization_noise",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(normalization_noise_ctl->text(), &value) && value != options_->normalization_noise ) {
            LOCK();
            options_->normalization_noise = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });


  max_iterations_ctl = add_numeric_box(form, "max_iterations",
      [this]() {
        if ( options_ && !updatingControls() ) {
          int value;
          if ( fromString(max_iterations_ctl->text(), &value) && value != options_->max_iterations ) {
            LOCK();
            options_->max_iterations = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

}

void QEccSettings::set_registration_options(c_ecc_options * options)
{
  options_ = options;
  updateControls();
}

const c_ecc_options * QEccSettings::registration_options() const
{
  return options_;
}

void QEccSettings::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    scale_ctl->setValue(options_->scale);
    eps_ctl->setValue(options_->eps);
    min_rho_ctl->setValue(options_->min_rho);
    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);
    update_step_scale_ctl->setValue(options_->update_step_scale);
    normalization_scale_ctl->setValue(options_->normalization_scale);
    normalization_noise_ctl->setValue(options_->normalization_noise);
    max_iterations_ctl->setValue(options_->max_iterations);
    setEnabled(true);
  }
}



QEccflowSettings::QEccflowSettings(QWidget * parent)
    : Base("QEccflowSettings", parent)
{
  support_scale_ctl = add_numeric_box(form, "support_scale",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(support_scale_ctl->text(), &value) && value != options_->support_scale ) {
            LOCK();
            options_->support_scale = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  update_multiplier_ctl = add_numeric_box(form, "update_multiplier",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(update_multiplier_ctl->text(), &value) && value != options_->update_multiplier ) {
            LOCK();
            options_->update_multiplier = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  max_iterations_ctl = add_numeric_box(form, "max_iterations",
      [this]() {
        if ( options_ && !updatingControls() ) {
          int value;
          if ( fromString(max_iterations_ctl->text(), &value) && value != options_->max_iterations ) {
            LOCK();
            options_->max_iterations = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });



  normalization_scale_ctl = add_numeric_box(form, "normalization_scale",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(normalization_scale_ctl->text(), &value) && value != options_->normalization_scale ) {
            LOCK();
            options_->normalization_scale = value;
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

}

void QEccflowSettings::set_registration_options(c_eccflow_options * options)
{
  options_ = options;
  updateControls();
}

const c_eccflow_options * QEccflowSettings::registration_options() const
{
  return options_;
}

void QEccflowSettings::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    support_scale_ctl->setValue(options_->support_scale);
    update_multiplier_ctl->setValue(options_->update_multiplier);
    max_iterations_ctl->setValue(options_->max_iterations);
    normalization_scale_ctl->setValue(options_->normalization_scale);

    setEnabled(true);
  }
}



QFrameRegistrationBaseSettings::QFrameRegistrationBaseSettings(QWidget * parent)
    : Base("QFrameRegistrationBaseSettings", parent)
{

  reference_channel_ctl = add_enum_combobox<QRegistrationColorChannelCombo>(
      "Registration channel:",
      [this](color_channel_type v) {
        if ( options_ && v != options_->registration_channel ) {
          options_->registration_channel = v;
          emit parameterChanged();
        }
      });

  motion_type_ctl = add_enum_combobox<QEccMotionTypeCombo>(
      "Motion type:",
      [this](ECC_MOTION_TYPE v) {
        if ( options_ && v != options_->motion_type ) {
          options_->motion_type = v;
          emit parameterChanged();
        }
      });

  feature_scale_ctl = add_numeric_box(form, "Feature scale",
      [this]() {
        if ( options_ && !updatingControls() ) {
          double value;
          if ( fromString(feature_scale_ctl->text(), &value) && value != options_->feature_scale ) {
            options_->feature_scale = value;
            emit parameterChanged();
          }
        }
      });

  enable_ecc_ctl = add_checkbox("Enable ECC",
      [this](int state) {
        if ( options_ ) {
          const bool checked = state == Qt::Checked;
          if ( options_->enable_ecc != checked ) {
            ecc_ctl->setVisible(options_->enable_ecc = checked);
            emit parameterChanged();
          }
        }
      });

  form->addRow(ecc_ctl = new QEccSettings(this));


  enable_eccflow_ctl = add_checkbox("Enable ECCFLOW",
      [this](int state ) {
        if ( options_ ) {
          const bool checked = state == Qt::Checked;
          if ( options_->enable_eccflow != checked ) {
            eccflow_ctl->setVisible(options_->enable_eccflow = checked);
            emit parameterChanged();
          }
        }
      });

  form->addRow(eccflow_ctl = new QEccflowSettings(this));

  updateControls();
}

void QFrameRegistrationBaseSettings::set_registration_options(c_frame_registration_base_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_frame_registration_base_options * QFrameRegistrationBaseSettings::registration_options() const
{
  return options_;
}

void QFrameRegistrationBaseSettings::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {

    reference_channel_ctl->setCurrentItem(options_->registration_channel);
    motion_type_ctl->setCurrentItem(options_->motion_type);
    enable_ecc_ctl->setChecked(options_->enable_ecc);
    enable_eccflow_ctl->setChecked(options_->enable_eccflow);
    feature_scale_ctl->setValue(options_->feature_scale);

    ecc_ctl->set_registration_options(&options_->ecc);
    ecc_ctl->setVisible(options_->enable_ecc);

    eccflow_ctl->set_registration_options(&options_->eccflow);
    eccflow_ctl->setVisible(options_->enable_eccflow);

    setEnabled(true);
  }
}

QFeatureBasedRegistrationSettings::QFeatureBasedRegistrationSettings(QWidget * parent)
    : Base("QFeatureBasedRegistrationSettings", parent)
{

  hessianThreshold_ctl =
      add_numeric_box(form, "Hessian threshold",
          [this]() {
            if (options_ && !updatingControls() ) {
              double value;
              if ( fromString(hessianThreshold_ctl->text(), &value) && value != options_->hessianThreshold ) {
                options_->hessianThreshold = value;
                emit parameterChanged();
              }
            }
          });

  nOctaves_ctl = add_numeric_box(form, "Num Octaves",
      [this]() {
        if (options_ && !updatingControls() ) {
          int value;
          if ( fromString(nOctaves_ctl->text(), &value) && value != options_->nOctaves ) {
            options_->nOctaves = value;
            emit parameterChanged();
          }
        }
      });

  nOctaveLayers_ctl = add_numeric_box(form, "Num Octave Layers",
      [this]() {
        if (options_ && !updatingControls() ) {
          int value;
          if ( fromString(nOctaveLayers_ctl->text(), &value) && value != options_->nOctaveLayers ) {
            options_->nOctaveLayers = value;
            emit parameterChanged();
          }
        }
      });

  extended_ctl = add_checkbox("Extended",
      [this](int state) {
        if (options_ ) {
          bool checked = state == Qt::Checked;
          if ( checked != options_->extended ) {
            options_->extended = checked;
            emit parameterChanged();
          }
        }
      });

  upright_ctl = add_checkbox("Upright",
      [this](int state) {
        if (options_ ) {
          bool checked = state == Qt::Checked;
          if ( checked != options_->upright ) {
            options_->upright = checked;
            emit parameterChanged();
          }
        }
      });

}

void QFeatureBasedRegistrationSettings::set_registration_options(c_feature_based_registration_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_feature_based_registration_options * QFeatureBasedRegistrationSettings::registration_options() const
{
  return this->options_;
}

void QFeatureBasedRegistrationSettings::onupdatecontrols()
{
  if ( !options_ )  {
    setEnabled(false);
  }
  else {
    hessianThreshold_ctl->setValue(options_->hessianThreshold);
    nOctaves_ctl->setValue(options_->nOctaves);
    nOctaveLayers_ctl->setValue(options_->nOctaveLayers);
    extended_ctl->setChecked(options_->extended);
    upright_ctl->setChecked(options_->upright);
    setEnabled(true);
  }
}

QPlanetaryDiskRegistrationSettings::QPlanetaryDiskRegistrationSettings(QWidget * parent) :
    Base("QPlanetaryDiskRegistrationSettings", parent)
{
//  crop_size_ctl =
//      add_numeric_box(form, "Crop size WxH:",
//          [this]() {
//            if (options_ && !updatingControls() ) {
//              cv::Size v;
//              if ( fromString(crop_size_ctl->text(), &v) && v != options_->crop_size ) {
//                options_->crop_size = v;
//                emit parameterChanged();
//              }
//            }
//          });

}

void QPlanetaryDiskRegistrationSettings::set_registration_options(c_planetary_disk_registration_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_planetary_disk_registration_options * QPlanetaryDiskRegistrationSettings::registration_options() const
{
  return this->options_;
}

void QPlanetaryDiskRegistrationSettings::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);
  }
  else {
    //crop_size_ctl->setValue(options_->crop_size);
    setEnabled(true);
  }
}

QStarFieldRegistrationSettings::QStarFieldRegistrationSettings(QWidget * parent)
  : Base("QStarFieldRegistrationSettings", parent)
{
}

void QStarFieldRegistrationSettings::set_registration_options(c_star_field_registration_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_star_field_registration_options * QStarFieldRegistrationSettings::registration_options() const
{
  return this->options_;
}

void QStarFieldRegistrationSettings::onupdatecontrols()
{
  Base::onupdatecontrols();
}

QFrameRegistrationOptions::QFrameRegistrationOptions(QWidget * parent)
    : Base("QFrameRegistrationOptions", parent)
{
  Q_INIT_RESOURCE(qstackingoptions_resources);


  frameRegistrationMethod_ctl = add_enum_combobox<QFrameRegistrationMethodCombo>(
      "Registration method:",
      [this](frame_registration_method) {
        updatemethodspecificpage();
      });


  form->addRow(featureBasedRegistrationSettings_ctl = new QFeatureBasedRegistrationSettings(this));
  form->addRow(planetaryDiskRegistrationSettings_ctl = new QPlanetaryDiskRegistrationSettings(this));
  form->addRow(starFieldRegistrationSettings_ctl = new QStarFieldRegistrationSettings(this));
  form->addRow(frameRegistrationBaseSettings_ctl = new QFrameRegistrationBaseSettings(this));


  applyToAll_ctl = new QToolButton(this);
  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  applyToAll_ctl->setIconSize(QSize(16,16));
  applyToAll_ctl->setStyleSheet(borderless_style);
  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
  connect(applyToAll_ctl, &QToolButton::clicked,
      [this]() {
        if ( options_ ) {
          emit applyFrameRegistrationOptionsToAllRequested(*options_);
        }
      });

  form->addRow(applyToAll_ctl);

  updatemethodspecificpage();
  setEnabled(false);
}

void QFrameRegistrationOptions::set_registration_options(c_frame_registration_options * options)
{
  this->options_ = options;
  updateControls();
}

const c_frame_registration_options * QFrameRegistrationOptions::registration_options() const
{
  return this->options_;
}

void QFrameRegistrationOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);

    frameRegistrationBaseSettings_ctl->set_registration_options(nullptr);
    featureBasedRegistrationSettings_ctl->set_registration_options(nullptr);
    planetaryDiskRegistrationSettings_ctl->set_registration_options(nullptr);
    starFieldRegistrationSettings_ctl->set_registration_options(nullptr);

  }
  else {

    frameRegistrationMethod_ctl->setCurrentItem(options_->registration_method);
    frameRegistrationBaseSettings_ctl->set_registration_options(&options_->base_options);
    featureBasedRegistrationSettings_ctl->set_registration_options(&options_->feature_options);
    planetaryDiskRegistrationSettings_ctl->set_registration_options(&options_->planetary_disk_options);
    starFieldRegistrationSettings_ctl->set_registration_options(&options_->star_field_options);

    setEnabled(true);
  }

  updatemethodspecificpage();
}

void QFrameRegistrationOptions::updatemethodspecificpage()
{
  if ( !options_ ) {
    //stackWidget->setVisible(false);
    featureBasedRegistrationSettings_ctl->setVisible(false);
    planetaryDiskRegistrationSettings_ctl->setVisible(false);
    starFieldRegistrationSettings_ctl->setVisible(false);
  }
  else {

//    switch ( options_->registration_method = frameRegistrationMethod_ctl->currentItem() ) {
//    case frame_registration_method_surf :
//      stackWidget->setCurrentWidget(featureBasedRegistrationSettings_ctl);
//      break;
//    case frame_registration_method_small_planetary_disk :
//      stackWidget->setCurrentWidget(planetaryDiskRegistrationSettings_ctl);
//      break;
//    case frame_registration_method_star_field :
//      stackWidget->setCurrentWidget(starFieldRegistrationSettings_ctl);
//      break;
//    }
//
//    if ( !stackWidget->isVisible() ) {
//      stackWidget->setVisible(true);
//    }

    switch ( options_->registration_method = frameRegistrationMethod_ctl->currentItem() ) {
    case frame_registration_method_surf :
      featureBasedRegistrationSettings_ctl->setVisible(true);
      planetaryDiskRegistrationSettings_ctl->setVisible(false);
      starFieldRegistrationSettings_ctl->setVisible(false);
      break;
    case frame_registration_method_planetary_disk :
      featureBasedRegistrationSettings_ctl->setVisible(false);
      planetaryDiskRegistrationSettings_ctl->setVisible(true);
      starFieldRegistrationSettings_ctl->setVisible(false);
      break;
    case frame_registration_method_star_field :
      featureBasedRegistrationSettings_ctl->setVisible(false);
      planetaryDiskRegistrationSettings_ctl->setVisible(false);
      starFieldRegistrationSettings_ctl->setVisible(true);
      break;
    }

  }

}






