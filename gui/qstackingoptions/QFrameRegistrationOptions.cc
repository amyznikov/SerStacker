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

  scale_ctl = add_numeric_box<double>("scale",
      [this](double value) {
        if ( options_ && value != options_->scale ) {
          options_->scale = value;
          emit parameterChanged();
        }
      });

  eps_ctl = add_numeric_box<double>("eps",
      [this](double value) {
        if ( options_ && value != options_->eps ) {
          options_->eps = value;
          emit parameterChanged();
        }
      });

  min_rho_ctl = add_numeric_box<double>("min_rho",
      [this](double value) {
        if ( options_ && value != options_->min_rho ) {
          options_->min_rho = value;
          emit parameterChanged();
        }
      });

  input_smooth_sigma_ctl = add_numeric_box<double>("input_smooth_sigma",
      [this](double value) {
        if ( options_ && value != options_->input_smooth_sigma ) {
          options_->input_smooth_sigma = value;
          emit parameterChanged();
        }
      });

  reference_smooth_sigma_ctl = add_numeric_box<double>("reference_smooth_sigma",
      [this](double value) {
        if ( options_ && value != options_->reference_smooth_sigma ) {
          options_->reference_smooth_sigma = value;
          emit parameterChanged();
        }
      });

  update_step_scale_ctl = add_numeric_box<double>("update_step_scale",
      [this](double value) {
        if ( options_ && value != options_->update_step_scale ) {
            options_->update_step_scale = value;
            emit parameterChanged();
        }
      });

  normalization_scale_ctl = add_numeric_box<int>("normalization_scale",
      [this](int value) {
        if ( options_ && value != options_->normalization_scale ) {
            options_->normalization_scale = value;
            emit parameterChanged();
        }
      });

  normalization_noise_ctl = add_numeric_box<double>("normalization_noise",
      [this](double value) {
        if ( options_ && value != options_->normalization_noise ) {
          options_->normalization_noise = value;
          emit parameterChanged();
        }
      });

  max_iterations_ctl = add_numeric_box<int>("max_iterations",
      [this](int value) {
        if ( options_ && value != options_->max_iterations ) {
          options_->max_iterations = value;
          emit parameterChanged();
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
  support_scale_ctl = add_numeric_box<double>("support_scale",
      [this](double value) {
        if ( options_ && value != options_->support_scale ) {
            options_->support_scale = value;
            emit parameterChanged();
        }
      });

  normalization_scale_ctl = add_numeric_box<double>("normalization_scale",
      [this](double v) {
        if ( options_ && v != options_->normalization_scale ) {
          options_->normalization_scale = v;
          emit parameterChanged();
        }
      });

//  input_smooth_sigma_ctl = add_numeric_box<double>("input smooth sigma",
//      [this](double v) {
//        if ( options_ && v != options_->input_smooth_sigma ) {
//          options_->input_smooth_sigma = v;
//          emit parameterChanged();
//        }
//      });
//
//  reference_smooth_sigma_ctl = add_numeric_box<double>("reference smooth sigma",
//      [this](double v) {
//        if ( options_ && v != options_->input_smooth_sigma ) {
//          options_->reference_smooth_sigma = v;
//          emit parameterChanged();
//        }
//      });

  update_multiplier_ctl = add_numeric_box<double>("update_multiplier",
      [this](double value) {
        if ( options_ && value != options_->update_multiplier ) {
            options_->update_multiplier = value;
            emit parameterChanged();
        }
      });

  max_iterations_ctl = add_numeric_box<int>("max_iterations",
      [this](int value) {
        if ( options_ && value != options_->max_iterations ) {
          options_->max_iterations = value;
          emit parameterChanged();
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
//    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
//    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);

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

  feature_scale_ctl = add_numeric_box<double>(form, "Feature scale",
      [this](double value) {
        if ( options_ && value != options_->feature_scale ) {
          options_->feature_scale = value;
          emit parameterChanged();
        }
      });

  interpolation_ctl =  add_enum_combobox<QEccInterpolatioMethodCombo>(
      "Interpolation method:",
      [this](ECC_INTERPOLATION_METHOD v) {
        if ( options_ && v != options_->interpolation ) {
          options_->interpolation = v;
          emit parameterChanged();
        }
      });

  border_ctl = add_enum_combobox<QEccBorderModeCombo>(
      "Border mode:",
      [this](ECC_BORDER_MODE v) {
        if ( options_ && v != options_->border_mode ) {
          options_->border_mode = v;
          emit parameterChanged();
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
  connect(ecc_ctl, &QEccSettings::parameterChanged,
      this, &ThisClass::parameterChanged);

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
  connect(eccflow_ctl, &QEccflowSettings::parameterChanged,
      this, &ThisClass::parameterChanged);

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
    interpolation_ctl->setCurrentItem(options_->interpolation);
    border_ctl->setCurrentItem(options_->border_mode);
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
      add_numeric_box<double>("Hessian threshold",
          [this](double value) {
            if (options_ && value != options_->hessianThreshold ) {
              options_->hessianThreshold = value;
              emit parameterChanged();
            }
          });

  nOctaves_ctl = add_numeric_box<int>("Num Octaves",
      [this](int value) {
        if (options_ && value != options_->nOctaves ) {
          options_->nOctaves = value;
          emit parameterChanged();
        }
      });

  nOctaveLayers_ctl = add_numeric_box<int>("Num Octave Layers",
      [this](int value) {
        if (options_ && value != options_->nOctaveLayers ) {
          options_->nOctaveLayers = value;
          emit parameterChanged();
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
}

QPlanetaryDiskRegistrationSettings::QPlanetaryDiskRegistrationSettings(const QString & prefix, QWidget * parent)
  : Base(prefix, parent)
{
}

void QPlanetaryDiskRegistrationSettings::set_planetary_disk_options(c_planetary_disk_registration_options * options)
{
  this->planetary_disk_options_ = options;
  updateControls();
}

const c_planetary_disk_registration_options * QPlanetaryDiskRegistrationSettings::planetary_disk_options() const
{
  return this->planetary_disk_options_;
}

void QPlanetaryDiskRegistrationSettings::onupdatecontrols()
{
  if ( !planetary_disk_options_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

QJovianDerotationSettings::QJovianDerotationSettings(QWidget * parent)
  : Base("QJovianDerotationSettings", parent)
{
}

void QJovianDerotationSettings::set_jovian_derotation_options(c_jovian_derotation_options * jovian_derotation_options)
{
  this->jovian_derotation_options_ = jovian_derotation_options;
  updateControls();
}

const c_jovian_derotation_options * QJovianDerotationSettings::jovian_derotation_options() const
{
  return this->jovian_derotation_options_;
}

void QJovianDerotationSettings::onupdatecontrols()
{
  Base::onupdatecontrols();

  if ( !jovian_derotation_options_ || !planetary_disk_options_ ) {
    setEnabled(false);
  }
  else {
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
        emit parameterChanged();
      });


  accumulateAndCompensateTurbulentFlow_ctl =
      add_checkbox("Accumulate and compensate turbulent flow",
          [this](int state) {
            if ( options_ ) {
              bool checked = state == Qt::Checked;
              if ( options_->accumulate_and_compensate_turbulent_flow != checked ) {
                options_->accumulate_and_compensate_turbulent_flow = checked;
                emit parameterChanged();
              }
            }
          });


  alignedFramesProcessor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Postprocess aligned frames:",
          [this](int index) {
            if ( options_ ) {
              options_->aligned_frame_processor =
                  alignedFramesProcessor_ctl->processor(index);
              emit parameterChanged();
            }
          });

//  incremental_mode_ctl = add_checkbox("Incremental mode (STUPID TEST, DON'T USE)",
//      [this](int state) {
//        if ( options_ ) {
//          bool checked = state == Qt::Checked;
//          if ( options_->incremental_mode != checked ) {
//            options_->incremental_mode = checked;
//            emit parameterChanged();
//          }
//        }
//      });

  form->addRow(featureBasedRegistrationSettings = new QFeatureBasedRegistrationSettings(this));
  form->addRow(planetaryDiskRegistrationSettings = new QPlanetaryDiskRegistrationSettings(this));
  form->addRow(jovianDerotationSettings = new QJovianDerotationSettings(this));
  form->addRow(starFieldRegistrationSettings = new QStarFieldRegistrationSettings(this));
  form->addRow(frameRegistrationBaseSettings = new QFrameRegistrationBaseSettings(this));


  connect(featureBasedRegistrationSettings, &QFeatureBasedRegistrationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(planetaryDiskRegistrationSettings, &QPlanetaryDiskRegistrationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(jovianDerotationSettings, &QJovianDerotationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(starFieldRegistrationSettings, &QStarFieldRegistrationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(frameRegistrationBaseSettings, &QFrameRegistrationBaseSettings::parameterChanged,
      this, &ThisClass::parameterChanged);


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

    frameRegistrationBaseSettings->set_registration_options(nullptr);
    featureBasedRegistrationSettings->set_registration_options(nullptr);
    planetaryDiskRegistrationSettings->set_planetary_disk_options(nullptr);
    jovianDerotationSettings->set_planetary_disk_options(nullptr);
    jovianDerotationSettings->set_jovian_derotation_options(nullptr);
    starFieldRegistrationSettings->set_registration_options(nullptr);

  }
  else {

    frameRegistrationMethod_ctl->setCurrentItem(options_->registration_method);
    accumulateAndCompensateTurbulentFlow_ctl->setChecked(options_->accumulate_and_compensate_turbulent_flow);
//    incremental_mode_ctl->setChecked(options_->incremental_mode);
    frameRegistrationBaseSettings->set_registration_options(&options_->base_options);
    featureBasedRegistrationSettings->set_registration_options(&options_->feature_options);
    planetaryDiskRegistrationSettings->set_planetary_disk_options(&options_->planetary_disk_options);
    jovianDerotationSettings->set_planetary_disk_options(&options_->planetary_disk_options);
    jovianDerotationSettings->set_jovian_derotation_options(&options_->jovian_derotation_options);
    starFieldRegistrationSettings->set_registration_options(&options_->star_field_options);

    if ( !alignedFramesProcessor_ctl->setCurrentProcessor(options_->aligned_frame_processor) ) {
      options_->aligned_frame_processor.reset();
    }

    setEnabled(true);
  }

  updatemethodspecificpage();
}

void QFrameRegistrationOptions::updatemethodspecificpage()
{
  if ( !options_ ) {
    //stackWidget->setVisible(false);
    featureBasedRegistrationSettings->setVisible(false);
    planetaryDiskRegistrationSettings->setVisible(false);
    jovianDerotationSettings->setVisible(false);
    starFieldRegistrationSettings->setVisible(false);
  }
  else {

    switch ( options_->registration_method = frameRegistrationMethod_ctl->currentItem() ) {
    case frame_registration_method_surf :
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(true);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(false);
      break;
    case frame_registration_method_planetary_disk :
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(true);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(false);
      break;
    case frame_registration_method_star_field :
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(true);
      break;
    case frame_registration_method_jovian_derotate :
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(true);
      starFieldRegistrationSettings->setVisible(false);
      break;
    case frame_registration_none :
      frameRegistrationBaseSettings->setVisible(false);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(false);
      break;
    }

  }

}






