/*
 * QMasterFrameSelectionOptions.h
 *
 *  Created on: May 23, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMasterFrameSelectionOptions_h__
#define __QMasterFrameSelectionOptions_h__

#include <gui/qpipeline/QInputSourceSelectionControl.h>
#include <gui/widgets/UpdateControls.h>
#include <core/proc/image_registration/c_frame_registration.h>

class QMasterSourceSelectionCombo :
    public QWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QMasterSourceSelectionCombo ThisClass;
  typedef QWidget Base;

  struct InputSourceData {
    std::string source_pathfilename;
    int source_size = 0;
  };

  QMasterSourceSelectionCombo(QWidget * parent = nullptr);

  void refreshInputSources(c_image_processing_pipeline * pipeline);
  void setEnableExternalFile(bool v);
  bool enableExternalFile() const;

  void setCurrentInputSource(const std::string & pathfilename);
  InputSourceData currentInputSource() const;

Q_SIGNALS:
  void currentSourceChanged();

protected Q_SLOTS:
  void onBrowseButtonClicked();
  void onComboboxCurrentIndexChanged(int index);

protected:
  QComboBox * combo_ = nullptr;
  QToolButton * browse_ctl = nullptr;
};

// must be declared outside of any namespace
Q_DECLARE_METATYPE(QMasterSourceSelectionCombo::InputSourceData);


class QMasterFrameSelectionControl :
    public QSettingsWidgetTemplate<c_master_frame_selection_options>,
    public QInputSourceSelectionControl
{
  Q_OBJECT;
public:
  typedef QMasterFrameSelectionControl ThisClass;
  typedef QSettingsWidgetTemplate<c_master_frame_selection_options> Base;

  QMasterFrameSelectionControl(QWidget * parent = nullptr);

  void refreshInputSources(c_image_processing_pipeline * pipeline) override;
  void setEnableExternalFile(bool v) override;
  bool enableExternalFile() const override;

protected Q_SLOTS:
  void updateMasterSourceControlStates();


protected:
  QEnumComboBox<master_frame_selection_method> * masterFrameSelectionMethod_ctl = nullptr;
  QMasterSourceSelectionCombo * masterSource_ctl = nullptr;
  QSpinBox * masterFrameIndex_ctl = nullptr;
};

#endif /* __QMasterFrameSelectionOptions_h__ */
