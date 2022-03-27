/*
 * QFrameRegistrationSettings.cc
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#include "QFrameRegistrationOptions.h"
//#include <gui/widgets/addctrl.h>
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

//
//QString toString(enum frame_registration_method m)
//{
//  return toStdString(m).c_str();
//}
//
//enum frame_registration_method fromString(const QString  & s,
//    enum frame_registration_method defval )
//{
//  return fromStdString(s.toStdString(), defval);
//}
//
//QString toString(enum ECC_MOTION_TYPE v)
//{
//  return toStdString(v).c_str();
//}
//
//enum ECC_MOTION_TYPE fromString(const QString & s, enum ECC_MOTION_TYPE defval)
//{
//  return fromStdString(s.toStdString(), defval);
//}
//
//
//QString toString(enum color_channel_type v)
//{
//  return toStdString(v).c_str();
//}
//
//enum color_channel_type fromString(const QString  & s, enum color_channel_type defval )
//{
//  return fromStdString(s.toStdString(), defval);
//}


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

  input_smooth_sigma_ctl = add_numeric_box<double>("input smooth sigma",
      [this](double v) {
        if ( options_ && v != options_->input_smooth_sigma ) {
          options_->input_smooth_sigma = v;
          emit parameterChanged();
        }
      });

  reference_smooth_sigma_ctl = add_numeric_box<double>("reference smooth sigma",
      [this](double v) {
        if ( options_ && v != options_->input_smooth_sigma ) {
          options_->reference_smooth_sigma = v;
          emit parameterChanged();
        }
      });

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
    input_smooth_sigma_ctl->setValue(options_->input_smooth_sigma);
    reference_smooth_sigma_ctl->setValue(options_->reference_smooth_sigma);

    setEnabled(true);
  }
}



QFrameRegistrationBaseSettings::QFrameRegistrationBaseSettings(QWidget * parent)
    : Base("QFrameRegistrationBaseSettings", parent)
{

  reference_channel_ctl = add_enum_combobox<color_channel_type>(
      "Registration channel:",
      [this](color_channel_type v) {
        if ( options_ && v != options_->registration_channel ) {
          options_->registration_channel = v;
          emit parameterChanged();
        }
      });

  motion_type_ctl = add_enum_combobox<ECC_MOTION_TYPE>(
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

  interpolation_ctl =  add_enum_combobox<ECC_INTERPOLATION_METHOD>(
      "Interpolation method:",
      [this](ECC_INTERPOLATION_METHOD v) {
        if ( options_ && v != options_->interpolation ) {
          options_->interpolation = v;
          emit parameterChanged();
        }
      });

  border_ctl = add_enum_combobox<ECC_BORDER_MODE>(
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
  construct();
}

QFeatureBasedRegistrationSettings::QFeatureBasedRegistrationSettings(const QString & prefix, QWidget * parent)
  : Base(prefix, parent)
{
  construct();
}

void QFeatureBasedRegistrationSettings::construct()
{
  hessianThreshold_ctl =
      add_numeric_box<double>("Hessian threshold",
          [this](double value) {
            if (feature_options_ && value != feature_options_->hessianThreshold ) {
              feature_options_->hessianThreshold = value;
              emit parameterChanged();
            }
          });

  nOctaves_ctl = add_numeric_box<int>("Num Octaves",
      [this](int value) {
        if (feature_options_ && value != feature_options_->nOctaves ) {
          feature_options_->nOctaves = value;
          emit parameterChanged();
        }
      });

  nOctaveLayers_ctl = add_numeric_box<int>("Num Octave Layers",
      [this](int value) {
        if (feature_options_ && value != feature_options_->nOctaveLayers ) {
          feature_options_->nOctaveLayers = value;
          emit parameterChanged();
        }
      });

  extended_ctl = add_checkbox("Extended",
      [this](int state) {
        if (feature_options_ ) {
          bool checked = state == Qt::Checked;
          if ( checked != feature_options_->extended ) {
            feature_options_->extended = checked;
            emit parameterChanged();
          }
        }
      });

  upright_ctl = add_checkbox("Upright",
      [this](int state) {
        if (feature_options_ ) {
          bool checked = state == Qt::Checked;
          if ( checked != feature_options_->upright ) {
            feature_options_->upright = checked;
            emit parameterChanged();
          }
        }
      });
}


void QFeatureBasedRegistrationSettings::set_feature_options(c_feature_based_registration_options * options)
{
  this->feature_options_ = options;
  updateControls();
}

const c_feature_based_registration_options * QFeatureBasedRegistrationSettings::feature_options() const
{
  return this->feature_options_;
}

void QFeatureBasedRegistrationSettings::onupdatecontrols()
{
  if ( !feature_options_ )  {
    setEnabled(false);
  }
  else {
    hessianThreshold_ctl->setValue(feature_options_->hessianThreshold);
    nOctaves_ctl->setValue(feature_options_->nOctaves);
    nOctaveLayers_ctl->setValue(feature_options_->nOctaveLayers);
    extended_ctl->setChecked(feature_options_->extended);
    upright_ctl->setChecked(feature_options_->upright);
    setEnabled(true);
  }
}

QPlanetaryDiskRegistrationSettings::QPlanetaryDiskRegistrationSettings(QWidget * parent) :
    Base("QPlanetaryDiskRegistrationSettings", parent)
{
  construct();
}

QPlanetaryDiskRegistrationSettings::QPlanetaryDiskRegistrationSettings(const QString & prefix, QWidget * parent)
  : Base(prefix, parent)
{
  construct();
}

void QPlanetaryDiskRegistrationSettings::construct()
{
  align_planetary_disk_masks_ctl =
      add_checkbox("Align Planetary Disk masks instead of images",
          [this](int state) {
            if ( planetary_disk_options_ ) {
              const bool checked = state == Qt::Checked;
              if ( planetary_disk_options_->align_planetary_disk_masks != checked ) {
                planetary_disk_options_->align_planetary_disk_masks = checked;
                emit parameterChanged();
              }
            }
          });
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

    align_planetary_disk_masks_ctl->setChecked(
        planetary_disk_options_->align_planetary_disk_masks);

    setEnabled(true);
  }
}

QJovianDerotationSettings::QJovianDerotationSettings(QWidget * parent)
  : Base("QJovianDerotationSettings", parent)
{

  min_rotation_ctl =
      add_numeric_box<double>("Min Rotation [deg]:",
          [this](double v) -> void {
            if ( jovian_derotation_options_ ) {
              if ( jovian_derotation_options_->min_rotation != v ) {
                jovian_derotation_options_->min_rotation = v * M_PI / 180;
                emit parameterChanged();
              }
            }
          });

  max_rotation_ctl =
      add_numeric_box<double>("Max Rotation [deg]:",
          [this](double v) -> void {
            if ( jovian_derotation_options_ ) {
              if ( jovian_derotation_options_->max_rotation != v ) {
                jovian_derotation_options_->max_rotation = v * M_PI / 180;
                emit parameterChanged();
              }
            }
          });


  eccflow_support_scale_ctl =
      add_numeric_box<int>("ECC flow support scale:",
          [this](int v) -> void {
            if ( jovian_derotation_options_ ) {
              if ( jovian_derotation_options_->eccflow_support_scale != v ) {
                jovian_derotation_options_->eccflow_support_scale = v;
                emit parameterChanged();
              }
            }
          });

  eccflow_normalization_scale_ctl =
      add_numeric_box<int>("ECC flow normalization scale:",
          [this](int v) -> void {
            if ( jovian_derotation_options_ ) {
              if ( jovian_derotation_options_->eccflow_normalization_scale != v ) {
                jovian_derotation_options_->eccflow_normalization_scale = v;
                emit parameterChanged();
              }
            }
          });


  eccflow_max_pyramid_level_ctl =
      add_numeric_box<int>("ECC flow max pyramid level:",
          [this](int v) -> void {
            if ( jovian_derotation_options_ ) {
              if ( jovian_derotation_options_->eccflow_max_pyramid_level != v ) {
                jovian_derotation_options_->eccflow_max_pyramid_level = v;
                emit parameterChanged();
              }
            }
          });


  align_jovian_disk_horizontally_ctl =
      add_checkbox("Rotate jovian disk horizontally:",
          [this](int state) -> void {
            if ( jovian_derotation_options_ ) {
              const bool checked = state == Qt::Checked;
              if ( jovian_derotation_options_->align_jovian_disk_horizontally != checked ) {
                jovian_derotation_options_->align_jovian_disk_horizontally = checked;
                emit parameterChanged();
              }
            }
          });

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
    min_rotation_ctl->setValue(jovian_derotation_options_->min_rotation * 180 / M_PI);
    max_rotation_ctl->setValue(jovian_derotation_options_->max_rotation * 180 / M_PI);
    eccflow_support_scale_ctl->setValue(jovian_derotation_options_->eccflow_support_scale);
    eccflow_normalization_scale_ctl->setValue(jovian_derotation_options_->eccflow_normalization_scale);
    eccflow_max_pyramid_level_ctl->setValue(jovian_derotation_options_->eccflow_max_pyramid_level);
    align_jovian_disk_horizontally_ctl->setChecked(jovian_derotation_options_->align_jovian_disk_horizontally);
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

QMMRegistrationSettings::QMMRegistrationSettings(QWidget * parent)
  : Base("QMMRegistrationSettings", parent)
{
}

void QMMRegistrationSettings::set_mm_options(c_mm_registration_options * options)
{
  mm_options_ = options;
  updateControls();
}

const c_mm_registration_options * QMMRegistrationSettings::mm_options() const
{
  return mm_options_;
}

void QMMRegistrationSettings::onupdatecontrols()
{
  Base::onupdatecontrols();
  if ( !mm_options_ ) {
    setEnabled(false);
  }
  else {
    // ...
  }
}



QFrameRegistrationOptions::QFrameRegistrationOptions(QWidget * parent)
    : Base("QFrameRegistrationOptions", parent)
{
  Q_INIT_RESOURCE(qstackingoptions_resources);


  frameRegistrationMethod_ctl =
      add_enum_combobox<frame_registration_method>(
          "Registration method:",
          [this](frame_registration_method) {
            updatemethodspecificpage();
            emit parameterChanged();
          });

  masterFrame_ctl =
      add_widget<QMasterFrameOptions>(); // "* Master Frame Options"

  accumulateAndCompensateTurbulentFlow_ctl =
      add_checkbox("Accumulate and compensate turbulent flow",
          [this](int state) {
            if ( options_ ) {
              bool checked = state == Qt::Checked;
              if ( options_->frame_registration_options().accumulate_and_compensate_turbulent_flow != checked ) {
                options_->frame_registration_options().accumulate_and_compensate_turbulent_flow = checked;
                emit parameterChanged();
              }
            }
          });


  alignedFramesProcessor_ctl =
      add_combobox<QImageProcessorSelectionCombo>("Process aligned frames:",
          [this](int index) {
            if ( options_ ) {
              options_->frame_registration_options().aligned_frame_processor =
                  alignedFramesProcessor_ctl->processor(index);
              emit parameterChanged();
            }
          });


  form->addRow(featureBasedRegistrationSettings = new QFeatureBasedRegistrationSettings(this));
  form->addRow(planetaryDiskRegistrationSettings = new QPlanetaryDiskRegistrationSettings(this));
  form->addRow(jovianDerotationSettings = new QJovianDerotationSettings(this));
  form->addRow(starFieldRegistrationSettings = new QStarFieldRegistrationSettings(this));
  form->addRow(mmRegistrationSettings = new QMMRegistrationSettings(this));
  form->addRow(frameRegistrationBaseSettings = new QFrameRegistrationBaseSettings(this));


  connect(masterFrame_ctl, &QMasterFrameOptions::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(featureBasedRegistrationSettings, &QFeatureBasedRegistrationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(planetaryDiskRegistrationSettings, &QPlanetaryDiskRegistrationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(jovianDerotationSettings, &QJovianDerotationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(starFieldRegistrationSettings, &QStarFieldRegistrationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(mmRegistrationSettings, &QMMRegistrationSettings::parameterChanged,
      this, &ThisClass::parameterChanged);
  connect(frameRegistrationBaseSettings, &QFrameRegistrationBaseSettings::parameterChanged,
      this, &ThisClass::parameterChanged);

//  connect(masterFrame_ctl, &QMasterFrameOptions::applyMasterFrameSettingsToAllRequested,
//      this, &ThisClass::applyMasterFrameSettingsToAllRequested);


  applyToAll_ctl = new QToolButton(this);
  applyToAll_ctl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  applyToAll_ctl->setIconSize(QSize(16,16));
  applyToAll_ctl->setStyleSheet(borderless_style);
  applyToAll_ctl->setIcon(getIcon(ICON_check_all));
  applyToAll_ctl->setText("Copy these parameters to all currently selected in treeview");
  connect(applyToAll_ctl, &QToolButton::clicked,
      [this]() {
        if ( options_ ) {
          emit applyFrameRegistrationOptionsToAllRequested(options_);
        }
      });

  form->addRow(applyToAll_ctl);

  updatemethodspecificpage();
  setEnabled(false);
}


void QFrameRegistrationOptions::set_stacking_options(const c_image_stacking_options::ptr & options)
{
  this->options_ = options;
  updateControls();
}

const c_image_stacking_options::ptr & QFrameRegistrationOptions::stacking_options() const
{
  return this->options_;
}

void QFrameRegistrationOptions::onupdatecontrols()
{
  if ( !options_ ) {
    setEnabled(false);

    masterFrame_ctl->set_master_frame_options(nullptr, nullptr);
    frameRegistrationBaseSettings->set_registration_options(nullptr);
    featureBasedRegistrationSettings->set_feature_options(nullptr);
    planetaryDiskRegistrationSettings->set_planetary_disk_options(nullptr);

    jovianDerotationSettings->set_planetary_disk_options(nullptr);
    jovianDerotationSettings->set_jovian_derotation_options(nullptr);

    mmRegistrationSettings->set_feature_options(nullptr);
    mmRegistrationSettings->set_mm_options(nullptr);

    starFieldRegistrationSettings->set_registration_options(nullptr);


  }
  else {

    masterFrame_ctl->set_master_frame_options(
        &options_->master_frame_options(),
        options_->input_sequence());

    c_frame_registration_options & registration_options =
        options_->frame_registration_options();

    frameRegistrationMethod_ctl->setCurrentItem(
        registration_options.registration_method);

    accumulateAndCompensateTurbulentFlow_ctl->setChecked(
        registration_options.accumulate_and_compensate_turbulent_flow);

    frameRegistrationBaseSettings->set_registration_options(
        &registration_options.base_options);

    featureBasedRegistrationSettings->set_feature_options(
        &registration_options.feature_options);

    planetaryDiskRegistrationSettings->set_planetary_disk_options(
        &registration_options.planetary_disk_options);

    jovianDerotationSettings->set_planetary_disk_options(
        &registration_options.planetary_disk_options);

    jovianDerotationSettings->set_jovian_derotation_options(
        &registration_options.jovian_derotation_options);

    mmRegistrationSettings->set_feature_options(
        &registration_options.feature_options);

    mmRegistrationSettings->set_mm_options(
        &registration_options.mm_options);

    starFieldRegistrationSettings->set_registration_options(
        &registration_options.star_field_options);

    if ( !alignedFramesProcessor_ctl->setCurrentProcessor(registration_options.aligned_frame_processor) ) {
      registration_options.aligned_frame_processor.reset();
    }

    setEnabled(true);
  }

  updatemethodspecificpage();
}

void QFrameRegistrationOptions::updatemethodspecificpage()
{

  if ( !options_ ) {
    masterFrame_ctl->setVisible(false);
    accumulateAndCompensateTurbulentFlow_ctl->setVisible(false);
    alignedFramesProcessor_ctl->setVisible(false);
    featureBasedRegistrationSettings->setVisible(false);
    planetaryDiskRegistrationSettings->setVisible(false);
    jovianDerotationSettings->setVisible(false);
    mmRegistrationSettings->setVisible(false);
    starFieldRegistrationSettings->setVisible(false);
  }
  else {

    c_frame_registration_options & registration_options =
        options_->frame_registration_options();

    switch ( registration_options.registration_method = frameRegistrationMethod_ctl->currentItem() ) {
    case frame_registration_method_surf :
      masterFrame_ctl->setVisible(true);
      accumulateAndCompensateTurbulentFlow_ctl->setVisible(true);
      alignedFramesProcessor_ctl->setVisible(true);
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(true);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(false);
      mmRegistrationSettings->setVisible(false);
      break;
    case frame_registration_method_planetary_disk :
      masterFrame_ctl->setVisible(true);
      accumulateAndCompensateTurbulentFlow_ctl->setVisible(true);
      alignedFramesProcessor_ctl->setVisible(true);
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(true);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(false);
      mmRegistrationSettings->setVisible(false);
      break;
    case frame_registration_method_star_field :
      masterFrame_ctl->setVisible(true);
      accumulateAndCompensateTurbulentFlow_ctl->setVisible(true);
      alignedFramesProcessor_ctl->setVisible(true);
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(true);
      mmRegistrationSettings->setVisible(false);
      break;
    case frame_registration_method_jovian_derotate :
      masterFrame_ctl->setVisible(true);
      accumulateAndCompensateTurbulentFlow_ctl->setVisible(true);
      alignedFramesProcessor_ctl->setVisible(true);
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(true);
      starFieldRegistrationSettings->setVisible(false);
      mmRegistrationSettings->setVisible(false);
      break;
    case frame_registration_method_mm :
      masterFrame_ctl->setVisible(true);
      accumulateAndCompensateTurbulentFlow_ctl->setVisible(true);
      alignedFramesProcessor_ctl->setVisible(true);
      frameRegistrationBaseSettings->setVisible(true);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(false);
      mmRegistrationSettings->setVisible(true);
      break;
    case frame_registration_none :
      masterFrame_ctl->setVisible(false);
      accumulateAndCompensateTurbulentFlow_ctl->setVisible(false);
      alignedFramesProcessor_ctl->setVisible(false);
      frameRegistrationBaseSettings->setVisible(false);
      featureBasedRegistrationSettings->setVisible(false);
      planetaryDiskRegistrationSettings->setVisible(false);
      jovianDerotationSettings->setVisible(false);
      starFieldRegistrationSettings->setVisible(false);
      mmRegistrationSettings->setVisible(false);
      break;
    }

  }

}






