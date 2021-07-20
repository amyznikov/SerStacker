/*
 * QImageProcessorChainSettings.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorChainSettings.h"
#include <core/debug.h>

QImageProcessorChainSettings::QImageProcessorChainSettings(QWidget * parent)
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

void QImageProcessorChainSettings::set_available_processors(const c_image_processor_chains::ptr & processors)
{
  available_processors_ = processors;
  updateControls();
}

const c_image_processor_chains::ptr QImageProcessorChainSettings::available_processors() const
{
  return available_processors_;
}


void QImageProcessorChainSettings::set_current_processor(const c_image_processor_chain::ptr & processor)
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

    for ( const c_image_processor::ptr & proc : *current_processor_ ) {

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

const c_image_processor_chain::ptr QImageProcessorChainSettings::current_processor() const
{
  return current_processor_;
}

void QImageProcessorChainSettings::onupdatecontrols()
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

QSettingsWidget * QImageProcessorChainSettings::createProcessorSettingWidged(const c_image_processor::ptr & proc) const
{
  if ( std::dynamic_pointer_cast<c_unsharp_mask_image_processor>(proc) ) {
    return new QUnsharpmaskSettigsWidget(std::dynamic_pointer_cast<c_unsharp_mask_image_processor>(proc));
  }

  if ( std::dynamic_pointer_cast<c_mtf_image_processor>(proc) ) {
    return new QMtfSettigsWidget(std::dynamic_pointer_cast<c_mtf_image_processor>(proc));
  }

  if ( std::dynamic_pointer_cast<c_autoclip_image_processor>(proc) ) {
    return new QAutoClipSettigsWidget(std::dynamic_pointer_cast<c_autoclip_image_processor>(proc));
  }

  if ( std::dynamic_pointer_cast<c_smap_image_processor>(proc) ) {
    return new QSmapSettigsWidget(std::dynamic_pointer_cast<c_smap_image_processor>(proc));
  }

  if ( std::dynamic_pointer_cast<c_test_image_processor>(proc) ) {
    return new QTestImageProcessorSettigsWidget(std::dynamic_pointer_cast<c_test_image_processor>(proc));
  }

  if ( std::dynamic_pointer_cast<c_align_color_channels_image_processor>(proc) ) {
    return new QAlignColorChannelsSettigsWidget(std::dynamic_pointer_cast<c_align_color_channels_image_processor>(proc));
  }

  if ( std::dynamic_pointer_cast<c_anscombe_image_processor>(proc) ) {
    return new QAnscombeSettigsWidget (std::dynamic_pointer_cast<c_anscombe_image_processor>(proc));
  }

  if ( std::dynamic_pointer_cast<c_noisemap_image_processor>(proc) ) {
    return new QNoiseMapSettigsWidget(std::dynamic_pointer_cast<c_noisemap_image_processor>(proc));
  }

  QSettingsWidget * w = // empty widget
      new QSettingsWidget(proc->name().c_str());
  return w;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QUnsharpmaskSettigsWidget::QUnsharpmaskSettigsWidget(const c_unsharp_mask_image_processor::ptr & processor, QWidget * parent) :
    Base(processor, parent)
{
  sigma_ctl = add_numeric_box("sigma", &processor_,
      &c_unsharp_mask_image_processor::sigma,
      &c_unsharp_mask_image_processor::set_sigma);

  alpha_ctl = add_numeric_box("alpha", &processor_,
      &c_unsharp_mask_image_processor::alpha,
      &c_unsharp_mask_image_processor::set_alpha);

  updateControls();
}

void QUnsharpmaskSettigsWidget::onupdatecontrols()
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


QAnscombeSettigsWidget::QAnscombeSettigsWidget(const c_anscombe_image_processor::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{

  add_combobox("Method:",
      method_ctl = new QAnscombeMethodCombo(this),
      &processor_,
      &c_anscombe_image_processor::method,
      &c_anscombe_image_processor::set_method);

  updateControls();
}

void QAnscombeSettigsWidget::onupdatecontrols()
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

QNoiseMapSettigsWidget::QNoiseMapSettigsWidget(const c_noisemap_image_processor::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{
  updateControls();
}

void QNoiseMapSettigsWidget::onupdatecontrols()
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

QAlignColorChannelsSettigsWidget::QAlignColorChannelsSettigsWidget(const c_align_color_channels_image_processor::ptr & processor, QWidget * parent) :
    Base(processor, parent)

{
  reference_channel_ctl = add_numeric_box("reference channel", &processor_,
      &c_align_color_channels_image_processor::reference_channel,
      &c_align_color_channels_image_processor::set_reference_channel);

  updateControls();
}

void QAlignColorChannelsSettigsWidget::onupdatecontrols()
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

QMtfSettigsWidget::QMtfSettigsWidget(const c_mtf_image_processor::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{
  mtf_ctl = new QMtfControl(this);
  form->addWidget(mtf_ctl);

  updateControls();

  connect(mtf_ctl, &QMtfControl::mtfChanged,
      this, &ThisClass::parameterChanged);
}

void QMtfSettigsWidget::onupdatecontrols()
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
QAutoClipSettigsWidget::QAutoClipSettigsWidget(const c_autoclip_image_processor::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{
  lclip_ctl = add_numeric_box("lclip [%]:", &processor_,
      &c_autoclip_image_processor::lclip,
      &c_autoclip_image_processor::set_lclip);

  hclip_ctl = add_numeric_box("hclip [%]:", &processor_,
      &c_autoclip_image_processor::hclip,
      &c_autoclip_image_processor::set_hclip);

  updateControls();
}

void QAutoClipSettigsWidget::onupdatecontrols()
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

QSmapSettigsWidget::QSmapSettigsWidget(const c_smap_image_processor::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{
  minv_ctl = add_numeric_box("Minv:", &processor_,
      &c_smap_image_processor::minv,
      &c_smap_image_processor::set_minv);

  scale_ctl = add_numeric_box("Scale:", &processor_,
      &c_smap_image_processor::scale,
      &c_smap_image_processor::set_scale);

  updateControls();
}

void QSmapSettigsWidget::onupdatecontrols()
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

QTestImageProcessorSettigsWidget::QTestImageProcessorSettigsWidget(const c_test_image_processor::ptr & processor, QWidget * parent)
  : Base(processor, parent)
{

  level_ctl = add_numeric_box("Level:", &processor_,
      &c_test_image_processor::level,
      &c_test_image_processor::set_level);

  scale_ctl = add_numeric_box("Scale:", &processor_,
      &c_test_image_processor::scale,
      &c_test_image_processor::set_scale);

  updateControls();
}

void QTestImageProcessorSettigsWidget::onupdatecontrols()
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

