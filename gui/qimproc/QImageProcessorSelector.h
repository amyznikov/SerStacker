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

  QImageProcessorSelector(QWidget * parent = nullptr);

  c_image_processor::ptr current_processor() const;

  bool imageProcessingEnabled() const;

Q_SIGNALS:
  //void currentImageProcessorChanged();
  //void imageProcessingEnableChanged(bool enable);

protected:
  void onupdatecontrols() override;
  void updatecurrentprocessor();

protected Q_SLOTS:
  void onProcessorSelectorCurrentIndexChanged(int);
  void addProcessor();
  void deleteCurrentProcessor();
  void renameCurrentProcessor();
  void addCopyOfCurrentProcessor();

protected:
  //c_image_processor_collection::ptr available_processors_;
  c_image_processor::ptr current_processor_;

  QCheckBox * enabled_ctl = nullptr;
  QToolBar * selectorToolbar = nullptr;
  QComboBox * selector_ctl = nullptr;
  QToolButton * selectorMenu_ctl = nullptr;

  QImageProcessorChainEditor * chain_ctl = nullptr;

};



#endif /* __QImageProcessorCollectionSettings_h__ */
