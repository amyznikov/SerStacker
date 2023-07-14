/*
 * QPipelineOutputOption.h
 *
 *  Created on: Jul 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QPipelineOutputOptionsBase_h__
#define __QPipelineOutputOptionsBase_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/widgets/QBrowsePathCombo.h>
#include <core/pipeline/c_image_processing_pipeline.h>

template<class c_optioon_type>
class QPipelineOutputOptions :
    public QSettingsWidget
{
public:
  typedef QPipelineOutputOptions ThisClass;
  typedef QSettingsWidget Base;

  QPipelineOutputOptions(QWidget * parent = nullptr) :
      Base("", parent)
  {
    output_directory_ctl =
        add_browse_for_path("", "Output directory:",
            QFileDialog::AcceptSave,
            QFileDialog::Directory,
            [this](const QString & v) {
              if ( options_ ) {
                options_->output_directory = v.toStdString();
                Q_EMIT parameterChanged();
              }
            },
            [this](QString * v) {
              if ( options_ ) {
                *v = options_->output_directory.c_str();
                return true;
              }
              return false;
            });
  }

  void set_output_options(c_optioon_type * options)
  {
    options_ = options;
    updateControls();
  }

  const c_optioon_type * output_options() const
  {
    return options_;
  }

protected:
  void onupdatecontrols() override
  {
    if ( !options_ ) {
      setEnabled(false);
    }
    else {
      Q_EMIT Base::populatecontrols();
      setEnabled(true);
    }
  }

protected:
  c_optioon_type * options_ = nullptr;
  QBrowsePathCombo * output_directory_ctl = nullptr;
};

#endif /* __QPipelineOutputOptionsBase_h__ */
