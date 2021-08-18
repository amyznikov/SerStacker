/*
 * QFrameRegistrationSettings.h
 *
 *  Created on: Feb 9, 2021
 *      Author: amyznikov
 */

#ifndef __QFrameRegistrationSettings_h__
#define __QFrameRegistrationSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QEnumComboBox.h>
#include <gui/widgets/QLineEditBox.h>
#include <core/pipeline/c_image_stacking_pipeline.h>

QString toString(enum frame_registration_method v);
enum frame_registration_method fromString(const QString  & s,
    enum frame_registration_method defval );

QString toString(enum ECC_MOTION_TYPE v);
enum ECC_MOTION_TYPE fromString(const QString  & s,
    enum ECC_MOTION_TYPE defval );

QString toString(enum color_channel_type v);
enum color_channel_type fromString(const QString  & s,
    enum color_channel_type defval );


class QFrameRegistrationMethodCombo :
    public QEnumComboBox<frame_registration_method>
{
  Q_OBJECT;
public:
  typedef QEnumComboBox<frame_registration_method> Base;

  QFrameRegistrationMethodCombo(QWidget * parent = Q_NULLPTR)
      : Base(parent, frame_registration_methods)
    {}
};

class QEccMotionTypeCombo :
    public QEnumComboBox<ECC_MOTION_TYPE>
{
  Q_OBJECT;
public:
  typedef QEnumComboBox<ECC_MOTION_TYPE> Base;

  QEccMotionTypeCombo(QWidget * parent = Q_NULLPTR)
      : Base(parent, ecc_motion_types)
    {}
};



class QRegistrationColorChannelCombo :
    public QEnumComboBox<color_channel_type>
{
  Q_OBJECT;
public:
  typedef QEnumComboBox<color_channel_type> Base;

  QRegistrationColorChannelCombo(QWidget * parent = Q_NULLPTR)
      : Base(parent, color_channel_types)
    {}
};



class QEccSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QEccSettings ThisClass;
  typedef QSettingsWidget Base;

  QEccSettings(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_ecc_options * options);
  const c_ecc_options * registration_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_ecc_options * options_ = Q_NULLPTR;

  QNumberEditBox * scale_ctl = Q_NULLPTR;
  QNumberEditBox * eps_ctl = Q_NULLPTR;
  QNumberEditBox * min_rho_ctl = Q_NULLPTR;
  QNumberEditBox * input_smooth_sigma_ctl = Q_NULLPTR;
  QNumberEditBox * reference_smooth_sigma_ctl = Q_NULLPTR;
  QNumberEditBox * update_step_scale_ctl = Q_NULLPTR;
  QNumberEditBox * normalization_noise_ctl = Q_NULLPTR;
  QNumberEditBox * normalization_scale_ctl = Q_NULLPTR;
  QNumberEditBox * max_iterations_ctl = Q_NULLPTR;
};

class QEccflowSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QEccflowSettings ThisClass;
  typedef QSettingsWidget Base;

  QEccflowSettings(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_eccflow_options * options);
  const c_eccflow_options * registration_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_eccflow_options * options_ = Q_NULLPTR;

  QNumberEditBox * support_scale_ctl = Q_NULLPTR;
  QNumberEditBox * update_multiplier_ctl = Q_NULLPTR;

  QNumberEditBox * input_smooth_sigma_ctl = Q_NULLPTR;
  QNumberEditBox * reference_smooth_sigma_ctl = Q_NULLPTR;

  QNumberEditBox * max_iterations_ctl = Q_NULLPTR;
  QNumberEditBox * normalization_scale_ctl = Q_NULLPTR;
};

class QFrameRegistrationBaseSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFrameRegistrationBaseSettings ThisClass;
  typedef QSettingsWidget Base;

  QFrameRegistrationBaseSettings(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_frame_registration_base_options * options);
  const c_frame_registration_base_options * registration_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_frame_registration_base_options * options_ = Q_NULLPTR;

  QRegistrationColorChannelCombo * reference_channel_ctl = Q_NULLPTR;

  QEccMotionTypeCombo * motion_type_ctl = Q_NULLPTR;

  //  void set_registration_channel(int channel);
  //  int registration_channel() const;

  //  void set_interpolation_flags(int v);
  //  int interpolation_flags() const;
  QLineEditBox * feature_scale_ctl = Q_NULLPTR;

  QCheckBox * enable_ecc_ctl = Q_NULLPTR;
  QEccSettings * ecc_ctl = Q_NULLPTR;

  QCheckBox * enable_eccflow_ctl = Q_NULLPTR;
  QEccflowSettings * eccflow_ctl = Q_NULLPTR;
};

class QFeatureBasedRegistrationSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFeatureBasedRegistrationSettings ThisClass;
  typedef QSettingsWidget Base;

  QFeatureBasedRegistrationSettings(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_feature_based_registration_options * options);
  const c_feature_based_registration_options * registration_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_feature_based_registration_options * options_ =  Q_NULLPTR;

  QNumberEditBox * hessianThreshold_ctl = Q_NULLPTR;
  QNumberEditBox * nOctaves_ctl = Q_NULLPTR;
  QNumberEditBox * nOctaveLayers_ctl = Q_NULLPTR;
  QCheckBox * extended_ctl = Q_NULLPTR;
  QCheckBox * upright_ctl = Q_NULLPTR;

};


class QPlanetaryDiskRegistrationSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QPlanetaryDiskRegistrationSettings ThisClass;
  typedef QSettingsWidget Base;

  QPlanetaryDiskRegistrationSettings(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_planetary_disk_registration_options * options);
  const c_planetary_disk_registration_options * registration_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_planetary_disk_registration_options * options_ = Q_NULLPTR;
  //QNumberEditBox * crop_size_ctl = Q_NULLPTR;
};

class QStarFieldRegistrationSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QStarFieldRegistrationSettings ThisClass;
  typedef QSettingsWidget Base;

  QStarFieldRegistrationSettings(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_star_field_registration_options * options);
  const c_star_field_registration_options * registration_options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_star_field_registration_options * options_ = Q_NULLPTR;
};


class QFrameRegistrationOptions
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QFrameRegistrationOptions ThisClass;
  typedef QSettingsWidget Base;

  QFrameRegistrationOptions(QWidget * parent = Q_NULLPTR);

  void set_registration_options(c_frame_registration_options * options);
  const c_frame_registration_options * registration_options() const;

signals:
  void applyFrameRegistrationOptionsToAllRequested(
      const c_frame_registration_options & options);

protected:
  void onupdatecontrols() override;
  void updatemethodspecificpage();

protected:
  c_frame_registration_options * options_ = Q_NULLPTR;
  QFrameRegistrationMethodCombo * frameRegistrationMethod_ctl = Q_NULLPTR;
  QFrameRegistrationBaseSettings * frameRegistrationBaseSettings_ctl = Q_NULLPTR;
  QFeatureBasedRegistrationSettings * featureBasedRegistrationSettings_ctl = Q_NULLPTR;
  QPlanetaryDiskRegistrationSettings * planetaryDiskRegistrationSettings_ctl = Q_NULLPTR;
  QStarFieldRegistrationSettings * starFieldRegistrationSettings_ctl = Q_NULLPTR;
  QToolButton * applyToAll_ctl = Q_NULLPTR;
};




#endif /* __QFrameRegistrationSettings_h__ */
