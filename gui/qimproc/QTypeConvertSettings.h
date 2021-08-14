/*
 * QTypeConvertSettings.h
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#ifndef __QTypeConvertSettings_h__
#define __QTypeConvertSettings_h__

#include "QImageProcessorRoutineSettings.h"
#include <core/improc/c_type_convert_routine.h>



class QTypeConvertSettings:
    public QImageProcessorRoutineSettings<c_type_convert_routine>
{
public:
  typedef QTypeConvertSettings ThisClass;
  typedef QImageProcessorRoutineSettings<c_type_convert_routine> Base;

  static const struct ClassFactory : public Base::ClassFactory {
    ClassFactory() :
        Base::ClassFactory(&RoutineType::class_factory,
            SettingsWidgetFactory([](const c_image_processor_routine::ptr & routine, QWidget * parent) {
              return new ThisClass(std::dynamic_pointer_cast<RoutineType>(routine), parent );
            }))
    {}
  } classFactory;


  struct QDDEPTHCombo:
      public QEnumComboBox<c_type_convert_routine::DDEPTH>
  {
    QDDEPTHCombo(QWidget * parent = Q_NULLPTR) :
        QEnumComboBox<c_type_convert_routine::DDEPTH>(parent, c_type_convert_routine::ddepths)
    {
    }
  };


  QTypeConvertSettings(const c_type_convert_routine::ptr & processor,
      QWidget * parent = Q_NULLPTR);

protected:
  void onupdatecontrols() override;

protected:
  QDDEPTHCombo * ddept_ctl = Q_NULLPTR;
  QCheckBox * auto_scale_ctl = Q_NULLPTR;
  QNumberEditBox * alpha_ctl = Q_NULLPTR;
  QNumberEditBox * beta_ctl = Q_NULLPTR;
};

#endif /* __QTypeConvertSettings_h__ */
