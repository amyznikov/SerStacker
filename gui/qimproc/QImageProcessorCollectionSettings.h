/*
 * QImageProcessorCollectionSettings.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessorCollectionSettings_h__
#define __QImageProcessorCollectionSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/addctrl.h>
#include <gui/qmtfcontrols/QMtfControl.h>
#include <core/improc/c_image_processor.h>
#include <core/debug.h>


class QImageProcessorCollectionSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageProcessorCollectionSettings ThisClass;
  typedef QSettingsWidget Base;

  QImageProcessorCollectionSettings(QWidget * parent = Q_NULLPTR);

  void set_available_processors(const c_image_processor_collection::ptr & processors);
  const c_image_processor_collection::ptr available_processors() const;

  void set_current_processor(const c_image_processor::ptr & processor);
  const c_image_processor::ptr current_processor() const;

protected:
  void onupdatecontrols() override;
  QSettingsWidget * createProcessorSettingWidged(const c_image_processor_routine::ptr & proc) const;

protected:
  c_image_processor_collection::ptr available_processors_;
  //QComboBox * processor_selector_ctl = Q_NULLPTR;

  c_image_processor::ptr current_processor_;
  QCheckBox * enabled_ctl = Q_NULLPTR;
};


template<class ImageProcessorType>
class QImageProcessorRoutineSettings
  : public QSettingsWidget
{
public:
  typedef QImageProcessorRoutineSettings ThisClass;
  typedef QSettingsWidget Base;
  typedef std::shared_ptr<ImageProcessorType> ImageProcessor;

  QImageProcessorRoutineSettings(const ImageProcessor & proc, QWidget * parent = Q_NULLPTR) :
      Base("", parent)
  {
    enabled_ctl = add_checkbox(form, "Enabled",
        [this]( int state) {
          if ( processor_ && !updatingControls() ) {
            bool enabled = state == Qt::Checked;
            if ( enabled != processor_->enabled() ) {
              LOCK();
              processor_->set_enabled(enabled);
              UNLOCK();
              emit parameterChanged();
            }
          }
        });

    set_processor(proc);
  }

  void set_processor(const ImageProcessor & proc) {
    processor_ = proc;
    updateControls();
  }
  const ImageProcessor & processor() const {
    return processor_;
  }


protected:
  void onupdatecontrols() override {
    if ( processor_ ) {
      enabled_ctl->setChecked(processor_->enabled());
    }
  }

protected:
  template<class ObjType, class PropType>
  QNumberEditBox * add_numeric_box(const char * name, std::shared_ptr<ObjType> * obj,
      PropType (ObjType::*getfn)() const, void (ObjType::*setfn)(PropType))
  {
    QNumberEditBox * ctl = new QNumberEditBox();
    form->addRow(name, ctl);
    QObject::connect(ctl, &QLineEditBox::textChanged,
        [this, ctl, obj, getfn, setfn]() {
          if ( *obj && !updatingControls() ) {
            PropType value;
            if ( fromString(ctl->text(), &value) && (obj->get()->*getfn)() != value ) {
              LOCK();
              (obj->get()->*setfn)(value);
              UNLOCK();
              emit parameterChanged();
            }
          }
        });

    return ctl;
  }

  template<class ObjType, class PropType>
  void add_combobox(const char * name, QEnumComboBox<PropType> * ctl, std::shared_ptr<ObjType> * obj,
      PropType (ObjType::*getfn)() const, void (ObjType::*setfn)(PropType))
  {
    form->addRow(name, ctl);
    QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
        [this, ctl, obj, getfn, setfn]() {
          if ( *obj && !updatingControls() ) {
            PropType value = ctl->currentItem();
            if ( (obj->get()->*getfn)() != value ) {
              LOCK();
              (obj->get()->*setfn)(value);
              UNLOCK();
              emit parameterChanged();
            }
          }
        });
  }

protected:
  ImageProcessor processor_;
  QCheckBox * enabled_ctl = Q_NULLPTR;
};


class QUnsharpMaskSettings
  : public QImageProcessorRoutineSettings<c_unsharp_mask_routine>
{
  Q_OBJECT;
public:
  typedef QUnsharpMaskSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  QUnsharpMaskSettings(const c_unsharp_mask_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * sigma_ctl = Q_NULLPTR;
  QNumberEditBox * alpha_ctl = Q_NULLPTR;
};


class QAnscombeSettings
  : public QImageProcessorRoutineSettings<c_anscombe_routine>
{
  Q_OBJECT;
public:
  typedef QAnscombeSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  static QString toString(enum anscombe_method v) {
    return QString::fromStdString(toStdString(v));
  }

  static enum anscombe_method fromString(const QString  & s, enum anscombe_method defval ) {
    return fromStdString(s.toStdString(), defval);
  }

  class QAnscombeMethodCombo :
      public QEnumComboBox<anscombe_method>
  {
  public:
    typedef QEnumComboBox<anscombe_method> Base;
    QAnscombeMethodCombo(QWidget * parent = Q_NULLPTR)
        : Base(parent, anscombe_methods)
      {}
  };

  QAnscombeSettings(const c_anscombe_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QAnscombeMethodCombo * method_ctl = Q_NULLPTR;
};


class QNoiseMapSettings
  : public QImageProcessorRoutineSettings<c_noisemap_routine>
{
  Q_OBJECT;
public:
  typedef QNoiseMapSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  QNoiseMapSettings(const c_noisemap_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
};




class QAlignColorChannelsSettings
    : public QImageProcessorRoutineSettings<c_align_color_channels_routine>
{
  Q_OBJECT;
public:
  typedef QAlignColorChannelsSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  QAlignColorChannelsSettings(const c_align_color_channels_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * reference_channel_ctl = Q_NULLPTR;
};

class QMtfSettings
  : public QImageProcessorRoutineSettings<c_mtf_routine>
{
  Q_OBJECT;
public:
  typedef QImageProcessorCollectionSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  QMtfSettings(const c_mtf_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QMtfControl * mtf_ctl = Q_NULLPTR;
};

class QAutoClipSettings
  : public QImageProcessorRoutineSettings<c_autoclip_routine>
{
  Q_OBJECT;
public:
  typedef QImageProcessorCollectionSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  QAutoClipSettings(const c_autoclip_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * lclip_ctl = Q_NULLPTR;
  QNumberEditBox * hclip_ctl = Q_NULLPTR;
};


class QSmapSettings
  : public QImageProcessorRoutineSettings<c_smap_routine>
{
  Q_OBJECT;
public:
  typedef QSmapSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  QSmapSettings(const c_smap_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * minv_ctl = Q_NULLPTR;
  QNumberEditBox * scale_ctl = Q_NULLPTR;
};


class QTestSettings
  : public QImageProcessorRoutineSettings<c_test_routine>
{
  Q_OBJECT;
public:
  typedef QTestSettings ThisClass;
  typedef QImageProcessorRoutineSettings Base;

  QTestSettings(const c_test_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * level_ctl = Q_NULLPTR;
  QNumberEditBox * scale_ctl = Q_NULLPTR;
};



#endif /* __QImageProcessorCollectionSettings_h__ */
