/*
 * QImageProcessorSelector.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessorCollectionSettings_h__
#define __QImageProcessorCollectionSettings_h__

#include "QImageProcessorChainEditor.h"


class QImageProcessorSelector
    : public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QImageProcessorSelector ThisClass;
  typedef QSettingsWidget Base;

  QImageProcessorSelector(QWidget * parent = Q_NULLPTR);

//  void set_available_processors(const c_image_processor_collection::ptr & processors);
//  const c_image_processor_collection::ptr available_processors() const;

  c_image_processor::ptr current_processor() const;

  bool imageProcessingEnabled() const;

signals:
  //void currentImageProcessorChanged();
  //void imageProcessingEnableChanged(bool enable);

protected:
  void onupdatecontrols() override;
  void updatecurrentprocessor();

protected slots:
  void onProcessorSelectorCurrentIndexChanged(int);
  void addProcessor();
  void deleteCurrentProcessor();
  void renameCurrentProcessor();
  void addCopyOfCurrentProcessor();

protected:
  //c_image_processor_collection::ptr available_processors_;
  c_image_processor::ptr current_processor_;

  QCheckBox * enabled_ctl = Q_NULLPTR;
  QToolBar * selectorToolbar = Q_NULLPTR;
  QComboBox * selector_ctl = Q_NULLPTR;
  QToolButton * selectorMenu_ctl = Q_NULLPTR;

  QImageProcessorChainEditor * chain_ctl = Q_NULLPTR;

};



#endif /* __QImageProcessorCollectionSettings_h__ */
