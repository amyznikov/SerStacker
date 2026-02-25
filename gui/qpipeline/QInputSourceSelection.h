/*
 * QInputSourceSelection.h
 *
 *  Created on: Feb 22, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputSourceSelectionOptions_h__
#define __QInputSourceSelectionOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/pipeline/stereo/c_stereo_input_options.h>

class QInputSourceSelection :
    public QSettingsWidget // Template<c_input_source_options>
{
public:
  typedef QInputSourceSelection ThisClass;
  typedef QSettingsWidget/*Template<c_stereo_input_selection> */Base;

  QInputSourceSelection(QWidget * parent = nullptr);

protected:
  void refreshControlStates();

protected:
//  QComboBox * source_ctl = nullptr;
};


class QMasterFrameSelection :
    public QSettingsWidgetTemplate<c_master_frame_selection_options>
{
public:
  typedef QMasterFrameSelection ThisClass;
  typedef QSettingsWidgetTemplate<c_master_frame_selection_options> Base;

  QMasterFrameSelection(QWidget * parent = nullptr);

protected:
  void refreshControlStates();
  void populateInputSources();
  bool browseForExternalSource();

protected:
  QEnumComboBox<master_frame_selection_method> * master_frame_selection_method_ctl = nullptr;
  QComboBox * input_source_ctl = nullptr;
  QNumericBox * master_frame_index_ctl = nullptr;
  //c_input_source::sptr _extranal_source;
};


class QStereoInputSourceSelection :
    public QSettingsWidgetTemplate<c_stereo_input_source_options>
{
public:
  typedef QStereoInputSourceSelection ThisClass;
  typedef QSettingsWidgetTemplate<c_stereo_input_source_options> Base;

  QStereoInputSourceSelection(QWidget * parent = nullptr);

protected:
  void refreshControlStates();
  void populateInputSources();

protected:
  QEnumComboBox<stereo_input_frame_layout_type> * layout_type_ctl = nullptr;
  QComboBox * left_stereo_source_ctl = nullptr;
  QComboBox * right_stereo_source_ctl = nullptr;
  QCheckBox * swap_cameras_ctl = nullptr;
};

#endif /* __QInputSourceSelectionOptions_h__ */
