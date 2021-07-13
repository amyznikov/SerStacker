/*
 * QImageProcessorChainSettings.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessorChainSettings_h__
#define __QImageProcessorChainSettings_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/addctrl.h>
#include <gui/qmtfcontrols/QMtfControl.h>
#include <core/improc/c_image_processor.h>
#include <core/debug.h>


class QImageProcessorChainSettings
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageProcessorChainSettings ThisClass;
  typedef QSettingsWidget Base;

  QImageProcessorChainSettings(QWidget * parent = Q_NULLPTR);

  void set_available_processors(const c_image_processor_chains::ptr & processors);
  const c_image_processor_chains::ptr available_processors() const;

  void set_current_processor(const c_image_processor_chain::ptr & processor);
  const c_image_processor_chain::ptr current_processor() const;

protected:
  void onupdatecontrols() override;
  QSettingsWidget * createProcessorSettingWidged(const c_image_processor::ptr & proc) const;

protected:
  c_image_processor_chains::ptr available_processors_;
  //QComboBox * processor_selector_ctl = Q_NULLPTR;

  c_image_processor_chain::ptr current_processor_;
  QCheckBox * enabled_ctl = Q_NULLPTR;
};


template<class ImageProcessorType>
class QImageProcessorSettings
  : public QSettingsWidget
{
public:
  typedef QImageProcessorSettings ThisClass;
  typedef QSettingsWidget Base;
  typedef std::shared_ptr<ImageProcessorType> ImageProcessor;

  QImageProcessorSettings(const ImageProcessor & proc, QWidget * parent = Q_NULLPTR) :
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

protected:
  ImageProcessor processor_;
  QCheckBox * enabled_ctl = Q_NULLPTR;
};


class QUnsharpmaskSettigsWidget
  : public QImageProcessorSettings<c_unsharp_mask_image_processor>
{
  Q_OBJECT;
public:
  typedef QUnsharpmaskSettigsWidget ThisClass;
  typedef QImageProcessorSettings Base;

  QUnsharpmaskSettigsWidget(const c_unsharp_mask_image_processor::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * sigma_ctl = Q_NULLPTR;
  QNumberEditBox * alpha_ctl = Q_NULLPTR;
};

class QAlignColorChannelsSettigsWidget
    : public QImageProcessorSettings<c_align_color_channels_image_processor>
{
  Q_OBJECT;
public:
  typedef QAlignColorChannelsSettigsWidget ThisClass;
  typedef QImageProcessorSettings Base;

  QAlignColorChannelsSettigsWidget(const c_align_color_channels_image_processor::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * reference_channel_ctl = Q_NULLPTR;
};

class QMtfSettigsWidget
  : public QImageProcessorSettings<c_mtf_image_processor>
{
  Q_OBJECT;
public:
  typedef QImageProcessorChainSettings ThisClass;
  typedef QImageProcessorSettings Base;

  QMtfSettigsWidget(const c_mtf_image_processor::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QMtfControl * mtf_ctl = Q_NULLPTR;
};

class QAutoClipSettigsWidget
  : public QImageProcessorSettings<c_autoclip_image_processor>
{
  Q_OBJECT;
public:
  typedef QImageProcessorChainSettings ThisClass;
  typedef QImageProcessorSettings Base;

  QAutoClipSettigsWidget(const c_autoclip_image_processor::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * lclip_ctl = Q_NULLPTR;
  QNumberEditBox * hclip_ctl = Q_NULLPTR;
};


class QSmapSettigsWidget
  : public QImageProcessorSettings<c_smap_image_processor>
{
  Q_OBJECT;
public:
  typedef QSmapSettigsWidget ThisClass;
  typedef QImageProcessorSettings Base;

  QSmapSettigsWidget(const c_smap_image_processor::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * minv_ctl = Q_NULLPTR;
  QNumberEditBox * scale_ctl = Q_NULLPTR;
};


class QTestImageProcessorSettigsWidget
  : public QImageProcessorSettings<c_test_image_processor>
{
  Q_OBJECT;
public:
  typedef QTestImageProcessorSettigsWidget ThisClass;
  typedef QImageProcessorSettings Base;

  QTestImageProcessorSettigsWidget(const c_test_image_processor::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QNumberEditBox * level_ctl = Q_NULLPTR;
  QNumberEditBox * scale_ctl = Q_NULLPTR;
};



#endif /* __QImageProcessorChainSettings_h__ */
