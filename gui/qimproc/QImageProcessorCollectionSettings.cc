/*
 * QImageProcessorCollectionSettings.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorCollectionSettings.h"
#include "QUnsharpMaskSettings.h"
#include "QSmapSettings.h"
#include "QNoiseMapSettings.h"
#include "QMtfSettings.h"
#include "QAutoClipSettings.h"
#include "QAnscombeSettings.h"
#include "QAlignColorChannelsSettings.h"
#include <core/debug.h>

QImageProcessorCollectionSettings::QImageProcessorCollectionSettings(QWidget * parent)
  : Base("QImageProcessorChainSettings", parent)
{

  QImageProcessorRoutineSettingsBase::registrerAllClassFactories();

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

      QSettingsWidget * w = createRoutineSettingWidged(proc);
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

QSettingsWidget * QImageProcessorCollectionSettings::createRoutineSettingWidged(
    const c_image_processor_routine::ptr & routine, QWidget * parent) const
{
  QSettingsWidget * w = QImageProcessorRoutineSettingsBase::create(routine, parent);
  if ( !w ) {  // create empty widget
    w = new QSettingsWidget("", parent);
  }
  return w;
}



///////////////////////////////////////////////////////////////////////////////////////////////////


static std::vector<const QImageProcessorRoutineSettingsBase::ClassFactory*> QImageProcessorRoutineSettingsClassList_;

void QImageProcessorRoutineSettingsBase::registrerClassFactory(const ClassFactory * classFactory)
{
  ClassFactoryGuardLock lock;

  std::vector<const QImageProcessorRoutineSettingsBase::ClassFactory*>::iterator ii =
      std::find(QImageProcessorRoutineSettingsClassList_.begin(),
          QImageProcessorRoutineSettingsClassList_.end(),
          classFactory);

  if ( ii == QImageProcessorRoutineSettingsClassList_.end() ) {
    QImageProcessorRoutineSettingsClassList_.emplace_back(classFactory);
  }

}

void QImageProcessorRoutineSettingsBase::registrerAllClassFactories()
{
  static bool registered = false;
  if ( !registered ) {
    registered = true;

    registrerClassFactory(&QUnsharpMaskSettings::classFactory);
    registrerClassFactory(&QSmapSettings::classFactory);
    registrerClassFactory(&QNoiseMapSettings::classFactory);
    registrerClassFactory(&QMtfSettings::classFactory);
    registrerClassFactory(&QAutoClipSettings::classFactory);
    registrerClassFactory(&QAnscombeSettings::classFactory);
    registrerClassFactory(&QAlignColorChannelsSettings::classFactory);

  }
}

QImageProcessorRoutineSettingsBase::QImageProcessorRoutineSettingsBase(const ClassFactory * factory, QWidget * parent)
  : Base("", parent), class_factory_(factory)
{

}

QImageProcessorRoutineSettingsBase * QImageProcessorRoutineSettingsBase::create(const c_image_processor_routine::ptr & routine, QWidget * parent)
{
  if ( !routine ) {
    return nullptr;
  }

  ClassFactoryGuardLock lock;

  for ( const ClassFactory * f : QImageProcessorRoutineSettingsClassList_ ) {
    if ( f->routine_factory == routine->classfactory() ) {
      return f->create_widget_instance(routine, parent);
    }
  }

  return nullptr;
}

