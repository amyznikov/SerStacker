/*
 * QImageProcessorSelector.h
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessorCollectionSettings_h__
#define __QImageProcessorCollectionSettings_h__

#include "QImageProcessorChainEditor.h"
#include <gui/widgets/UpdateControls.h>


class QImageProcessorSelector :
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QImageProcessorSelector ThisClass;
  typedef QFrame Base;

  QImageProcessorSelector(QWidget * parent = nullptr);

  c_image_processor::sptr current_processor() const;

  bool imageProcessingEnabled() const;

Q_SIGNALS:
  void parameterChanged();

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
  c_image_processor::sptr current_processor_;

  QVBoxLayout * lv_ = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QImageProcessorChainEditor * chain_ctl = nullptr;

  QCheckBox * enable_ctl = nullptr;
  QComboBox * selector_ctl = nullptr;
  QToolButton * selectorMenu_ctl = nullptr;
};



#endif /* __QImageProcessorCollectionSettings_h__ */
