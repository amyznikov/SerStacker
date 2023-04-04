/*
 * QGenericImageProcessorOutputOptions.h
 *
 *  Created on: Apr 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGenericImageProcessorOutputOptions_h__
#define __QGenericImageProcessorOutputOptions_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/generic/c_generic_image_processor.h>

class QGenericImageProcessorOutputOptions :
    public QSettingsWidget
{
public:
  typedef QGenericImageProcessorOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QGenericImageProcessorOutputOptions(QWidget * parent = nullptr);

  void set_options(c_generic_image_processor_output_options * options);
  c_generic_image_processor_output_options * options() const;

protected:
  void onupdatecontrols() override;

protected:
  c_generic_image_processor_output_options * options_ = nullptr;

  QBrowsePathCombo * output_directory_ctl = nullptr;

  QCheckBox * save_processed_frames_ctl = nullptr;
  QLineEditBox * processed_frames_filename_ctl = nullptr;

};

#endif /* __QGenericImageProcessorOutputOptions_h__ */
