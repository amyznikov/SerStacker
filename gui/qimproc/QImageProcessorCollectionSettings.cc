/*
 * QImageProcessorCollectionSettings.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorCollectionSettings.h"
#include <core/debug.h>

QImageProcessorCollectionSettings::QImageProcessorCollectionSettings(QWidget * parent)
  : Base("QImageProcessorChainSettings", parent)
{

//  processor_selector_ctl = add_combobox(form, "Image processing",
//      [](int) {
//      });
//
//  processor_selector_ctl->setEditable(false);
//  processor_selector_ctl->setEnabled(false);


  enabled_ctl = add_checkbox(form, "Enabled",
      [this](int state) {
        if ( !updatingControls() && current_processor_ ) {
          bool enable = state == Qt::Checked;
          if ( enable != current_processor_ ->enabled() ) {
            LOCK();
            current_processor_->set_enabled(enable);
            UNLOCK();
            emit parameterChanged();
          }
        }
      });

  updateControls();
}

void QImageProcessorCollectionSettings::set_available_processors(const c_image_processor_collection::ptr & processors)
{
  available_processors_ = processors;
  updateControls();
}

const c_image_processor_collection::ptr QImageProcessorCollectionSettings::available_processors() const
{
  return available_processors_;
}


void QImageProcessorCollectionSettings::set_current_processor(const c_image_processor::ptr & processor)
{
  QLayoutItem * item;
  QWidget * widget;


  current_processor_ = processor;

  while ( form->count() > 1 && (item = form->takeAt(1)) ) {
    form->removeWidget(widget = item->widget());
    delete item;
    delete widget;
  }

  if ( current_processor_ ) {

    for ( const c_image_processor_routine::ptr & proc : *current_processor_ ) {

      QSettingsWidget * w = createProcessorSettingWidged(proc);
      if ( w ) {

        add_expandable_groupbox(form, proc->display_name().c_str(), w);

        connect(w, &QSettingsWidget::parameterChanged,
            this, &ThisClass::parameterChanged);
      }
    }

  }

  updateControls();
}

const c_image_processor::ptr QImageProcessorCollectionSettings::current_processor() const
{
  return current_processor_;
}

void QImageProcessorCollectionSettings::onupdatecontrols()
{
  if ( !available_processors_ ) {
    //CF_DEBUG("H: processor_selector_ctl=%p", processor_selector_ctl);
    //processor_selector_ctl->clear();
    //CF_DEBUG("H");
    //processor_selector_ctl->setEnabled(false);
    //CF_DEBUG("H");
  }
  else {
//    CF_DEBUG("H");
//    processor_selector_ctl->clear();
//    CF_DEBUG("H");
//    for ( size_t i = 0, n = available_processors_->size(); i < n; ++i ) {
//      const c_image_processor_chain::ptr & processor = available_processors_->at(i);
//      if ( processor ) {
//        processor_selector_ctl->addItem(processor->name().c_str(), QVariant((int) (i)));
//      }
//    }
//
//    CF_DEBUG("H");
//    if ( processor_selector_ctl->count() > 0 ) {
//      processor_selector_ctl->setCurrentIndex(0);
//    }
//    CF_DEBUG("H");
//
//    processor_selector_ctl->setEnabled(true);
//    CF_DEBUG("H");
  }


  if ( !current_processor_ ) {
    setEnabled(false);
  }
  else {
    enabled_ctl->setChecked(current_processor_ ->enabled());
    setEnabled(true);
  }
  Base::onupdatecontrols();
}

QSettingsWidget * QImageProcessorCollectionSettings::createProcessorSettingWidged(const c_image_processor_routine::ptr & proc) const
{
  if ( std::dynamic_pointer_cast<c_unsharp_mask_routine>(proc) ) {
    return new QUnsharpMaskSettings(std::dynamic_pointer_cast<c_unsharp_mask_routine>(proc));
  }

  if ( std::dynamic_pointer_cast<c_mtf_routine>(proc) ) {
    return new QMtfSettings(std::dynamic_pointer_cast<c_mtf_routine>(proc));
  }

  if ( std::dynamic_pointer_cast<c_autoclip_routine>(proc) ) {
    return new QAutoClipSettings(std::dynamic_pointer_cast<c_autoclip_routine>(proc));
  }

  if ( std::dynamic_pointer_cast<c_smap_routine>(proc) ) {
    return new QSmapSettings(std::dynamic_pointer_cast<c_smap_routine>(proc));
  }

  if ( std::dynamic_pointer_cast<c_test_routine>(proc) ) {
    return new QTestSettings(std::dynamic_pointer_cast<c_test_routine>(proc));
  }

  if ( std::dynamic_pointer_cast<c_align_color_channels_routine>(proc) ) {
    return new QAlignColorChannelsSettings(std::dynamic_pointer_cast<c_align_color_channels_routine>(proc));
  }

  if ( std::dynamic_pointer_cast<c_anscombe_routine>(proc) ) {
    return new QAnscombeSettings (std::dynamic_pointer_cast<c_anscombe_routine>(proc));
  }

  if ( std::dynamic_pointer_cast<c_noisemap_routine>(proc) ) {
    return new QNoiseMapSettings(std::dynamic_pointer_cast<c_noisemap_routine>(proc));
  }

  QSettingsWidget * w = // empty widget
      new QSettingsWidget(proc->class_name().c_str());
  return w;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QUnsharpMaskSettings::QUnsharpMaskSettings(const c_unsharp_mask_routine::ptr & processor, QWidget * parent) :
    Base(processor, parent)
{
  sigma_ctl = add_numeric_box("sigma", &processor_,
      &c_unsharp_mask_routine::sigma,
      &c_unsharp_mask_routine::set_sigma);

  alpha_ctl = add_numeric_box("alpha", &processor_,
      &c_unsharp_mask_routine::alpha,
      &c_unsharp_mask_routine::set_alpha);

  updateControls();
}

void QUnsharpMaskSettings::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
  }
  else {
    sigma_ctl->setValue(processor_->sigma());
    alpha_ctl->setValue(processor_->alpha());

    setEnabled(true);
  }
  Base::onupdatecontrols();
}


///////////////////////////////////////////////////////////////////////////////////////////////////


QAnscombeSettings::QAnscombeSettings(const c_anscombe_routine::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{

  add_combobox("Method:",
      method_ctl = new QAnscombeMethodCombo(this),
      &processor_,
      &c_anscombe_routine::method,
      &c_anscombe_routine::set_method);

  updateControls();
}

void QAnscombeSettings::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
  }
  else {
    method_ctl->setCurrentItem(processor_->method());
    setEnabled(true);
  }

  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QNoiseMapSettings::QNoiseMapSettings(const c_noisemap_routine::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{
  updateControls();
}

void QNoiseMapSettings::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }

  Base::onupdatecontrols();
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QAlignColorChannelsSettings::QAlignColorChannelsSettings(const c_align_color_channels_routine::ptr & processor, QWidget * parent) :
    Base(processor, parent)

{
  reference_channel_ctl = add_numeric_box("reference channel", &processor_,
      &c_align_color_channels_routine::reference_channel,
      &c_align_color_channels_routine::set_reference_channel);

  updateControls();
}

void QAlignColorChannelsSettings::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
  }
  else {
    reference_channel_ctl->setValue(processor_->reference_channel());

    setEnabled(true);
  }
  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QMtfSettings::QMtfSettings(const c_mtf_routine::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{
  mtf_ctl = new QMtfControl(this);
  form->addWidget(mtf_ctl);

  updateControls();

  connect(mtf_ctl, &QMtfControl::mtfChanged,
      this, &ThisClass::parameterChanged);
}

void QMtfSettings::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
    mtf_ctl->setMtf(nullptr);
  }
  else {
    mtf_ctl->setMtf(processor_->mtf());
    setEnabled(true);
  }

  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
QAutoClipSettings::QAutoClipSettings(const c_autoclip_routine::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{
  lclip_ctl = add_numeric_box("lclip [%]:", &processor_,
      &c_autoclip_routine::lclip,
      &c_autoclip_routine::set_lclip);

  hclip_ctl = add_numeric_box("hclip [%]:", &processor_,
      &c_autoclip_routine::hclip,
      &c_autoclip_routine::set_hclip);

  updateControls();
}

void QAutoClipSettings::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
  }
  else {
    lclip_ctl->setValue(processor_->lclip());
    hclip_ctl->setValue(processor_->hclip());
    setEnabled(true);
  }
  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QSmapSettings::QSmapSettings(const c_smap_routine::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{
  minv_ctl = add_numeric_box("Minv:", &processor_,
      &c_smap_routine::minv,
      &c_smap_routine::set_minv);

  scale_ctl = add_numeric_box("Scale:", &processor_,
      &c_smap_routine::scale,
      &c_smap_routine::set_scale);

  updateControls();
}

void QSmapSettings::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
  }
  else {
    minv_ctl->setValue(processor_->minv());
    scale_ctl->setValue(processor_->scale());
    setEnabled(true);
  }
  Base::onupdatecontrols();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QTestSettings::QTestSettings(const c_test_routine::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{

  level_ctl = add_numeric_box("Level:", &processor_,
      &c_test_routine::level,
      &c_test_routine::set_level);

  scale_ctl = add_numeric_box("Scale:", &processor_,
      &c_test_routine::scale,
      &c_test_routine::set_scale);

  updateControls();
}

void QTestSettings::onupdatecontrols()
{
  if ( !processor_ ) {
    setEnabled(false);
  }
  else {
    level_ctl->setValue(processor_->level());
    scale_ctl->setValue(processor_->scale());
    setEnabled(true);
  }
  Base::onupdatecontrols();
}

