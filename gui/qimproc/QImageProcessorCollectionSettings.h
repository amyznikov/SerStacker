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
  QSettingsWidget * createRoutineSettingWidged(const c_image_processor_routine::ptr & proc, QWidget * parent = Q_NULLPTR) const;

protected:
  c_image_processor_collection::ptr available_processors_;
  //QComboBox * processor_selector_ctl = Q_NULLPTR;

  c_image_processor::ptr current_processor_;
  QCheckBox * enabled_ctl = Q_NULLPTR;
};


class QImageProcessorRoutineSettingsBase
  : public QSettingsWidget
{
public:
  typedef QImageProcessorRoutineSettingsBase ThisClass;
  typedef QSettingsWidget Base;
  typedef std::function<ThisClass*(const c_image_processor_routine::ptr & routine, QWidget * parent)> SettingsWidgetFactory;

  struct ClassFactory {
    const c_image_processor_routine::class_factory * const routine_factory;
    const SettingsWidgetFactory create_widget_instance;
    ClassFactory(const c_image_processor_routine::class_factory * _routine_factory,
        const SettingsWidgetFactory & _create_widget_instance) :
          routine_factory(_routine_factory),
          create_widget_instance(_create_widget_instance)
    {}
  };

  struct ClassFactoryGuardLock
  {
    ClassFactoryGuardLock() {
      mtx().lock();
    }

    ~ClassFactoryGuardLock() {
      mtx().unlock();
    }

    static QMutex & mtx() {
      static QMutex mtx_;
      return mtx_;
    }
  };

  static void registrerClassFactory(const ClassFactory * classFactory);
  static void registrerAllClassFactories();


  static ThisClass * create(const c_image_processor_routine::ptr & routine, QWidget * parent = Q_NULLPTR);

protected:
  QImageProcessorRoutineSettingsBase(const ClassFactory * factory, QWidget * parent = Q_NULLPTR);
  const ClassFactory * const class_factory_;
};


template<class ImageProcessorRoutine>
class QImageProcessorRoutineSettings
  : public QImageProcessorRoutineSettingsBase
{
public:
  typedef QImageProcessorRoutineSettings ThisClass;
  typedef QImageProcessorRoutineSettingsBase Base;
  typedef ImageProcessorRoutine RoutineType;
  typedef std::shared_ptr<RoutineType> RoutinePtr;


  QImageProcessorRoutineSettings(const ClassFactory * classFactory, const RoutinePtr & routine, QWidget * parent = Q_NULLPTR) :
      Base(classFactory, parent)
  {
    enabled_ctl = add_checkbox(form, "Enabled",
        [this]( int state) {
          if ( routine_ && !updatingControls() ) {
            bool enabled = state == Qt::Checked;
            if ( enabled != routine_->enabled() ) {
              LOCK();
              routine_->set_enabled(enabled);
              UNLOCK();
              emit parameterChanged();
            }
          }
        });

    set_routine(routine);
  }

  void set_routine(const RoutinePtr & routine) {
    routine_ = routine;
    updateControls();
  }
  const RoutinePtr & routine() const {
    return routine_;
  }


protected:
  void onupdatecontrols() override {
    if ( routine_ ) {
      enabled_ctl->setChecked(routine_->enabled());
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
  RoutinePtr routine_;
  QCheckBox * enabled_ctl = Q_NULLPTR;
};



#endif /* __QImageProcessorCollectionSettings_h__ */
