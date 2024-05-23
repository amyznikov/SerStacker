/*
 * QImageProcessingPipeline.cc
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#include "QImageProcessingPipeline.h"
#include "QOutputFrameWriterOptions.h"
#include "QImageStackingPipeline/QImageStackingPipeline.h"
#include "QStereoCalibrationPipeline/QStereoCalibrationPipeline.h"
#include "QCameraCalibrationPipeline/QCameraCalibrationPipeline.h"
#include "QRegularStereoMatcherPipeline/QRegularStereoMatcherPipeline.h"
#include "QGenericImageProcessingPipeline/QGenericImageProcessingPipeline.h"
#include "QLveStackingPipeline/QLveStackingPipeline.h"
#include "QRunningAveragePipeline/QRunningAveragePipeline.h"
#include "QVirtualStereoPipeline/QVirtualStereoPipeline.h"
#include "QRoiTrackerPipeline/QRoiTrackerPipeline.h"
#include "QEpipolarAlignmentPipeline/QEpipolarAlignmentPipeline.h"

#include <gui/widgets/QMatrixEdit.h>
#include <gui/widgets/QCameraIntrinsicsEditBox.h>
#include <gui/qfeature2d/QFeature2dOptions.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <gui/qpipeline/QFrameRegistrationOptions.h>
#include <gui/qpipeline/QMasterFrameSelectionControl.h>
#include <gui/qpipeline/stereo/QStereoMatcherOptions.h>

#include <core/debug.h>

void registerPipelineClasses()
{
#define REGISTER_PIPELINE_CLASS(C) \
    c_image_processing_pipeline::register_class(C::class_name(), C::tooltip(), \
        [](const std::string & name, const c_input_sequence::sptr & input_sequence) { \
          return c_image_processing_pipeline::sptr(new C(name.c_str(), input_sequence)); \
        });


  REGISTER_PIPELINE_CLASS(QImageStackingPipeline);
  REGISTER_PIPELINE_CLASS(QCameraCalibrationPipeline);
  REGISTER_PIPELINE_CLASS(QStereoCalibrationPipeline);
  REGISTER_PIPELINE_CLASS(QRegularStereoMatcherPipeline);
  REGISTER_PIPELINE_CLASS(QGenericImageProcessingPipeline);
  REGISTER_PIPELINE_CLASS(QLveStackingPipeline);
  REGISTER_PIPELINE_CLASS(QRunningAveragePipeline);
  REGISTER_PIPELINE_CLASS(QVirtualStereoPipeline);
  REGISTER_PIPELINE_CLASS(QRoiTrackerPipeline);
  REGISTER_PIPELINE_CLASS(QEpipolarAlignmentPipeline);


#undef REGISTER_PIPELINE_CLASS
}


///////////////////////////////////////////////////////////////////////////////

QPipelineSettingsWidget::QPipelineSettingsWidget(QWidget * parent) :
  Base("", parent)
{
  connect(this, &ThisClass::parameterChanged,
      this, &ThisClass::update_control_states);
}

void QPipelineSettingsWidget::update_pipeline_input_sources()
{
  c_update_controls_lock lock(this);
  for ( QInputSourceSelectionControl * combo : inputSourceCombos_ ) {
    combo->refreshInputSources(pipeline_);
  }
}

void QPipelineSettingsWidget::update_control_states()
{
  if( !pipeline_ || pipeline_->is_running() ) {
    setEnabled(false);
  }
  else {
    for( auto &p : state_ctls_ ) {
      QWidget *w = p.first;
      const std::function<bool(const c_image_processing_pipeline*)> &is_enabled = p.second;
      w->setEnabled(is_enabled(pipeline_));
    }
    setEnabled(true);
  }
}

void QPipelineSettingsWidget::setup_controls(const std::vector<c_image_processing_pipeline_ctrl> & ctrls)
{
  std::vector<QExpandableGroupBox*> groups;
  QExpandableGroupBox *currentgroup = nullptr;
  QSettingsWidget *currentsettings = this;

  for( int i = 0; i < ctrls.size(); ++i ) {

    const c_image_processing_pipeline_ctrl &ctrl =
        ctrls[i];

    switch (ctrl.type) {

      /////////////////////
      case c_image_processor_pipeline_ctl_begin_group: {

        if( currentgroup ) {
          groups.emplace_back(currentgroup);
        }

        currentgroup =
            currentsettings->add_expandable_groupbox(ctrl.name.c_str(),
                currentsettings = new QSettingsWidget("", this));

        connect(currentsettings, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        connect(this, &QSettingsWidget::populatecontrols,
            currentsettings, &QSettingsWidget::populatecontrols);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(currentsettings, ctrl.is_enabled);
        }

        break;
      }

        /////////////////////
      case c_image_processor_pipeline_ctl_end_group: {
        if( groups.empty() ) {
          currentgroup = nullptr;
          currentsettings = this;
        }
        else {
          currentgroup = groups.back();
          currentsettings = (QSettingsWidget*) currentgroup->view();
          groups.pop_back();
        }

        break;
      }

        /////////////////////
      case c_image_processor_pipeline_ctl_numeric_box: {

        QWidget *w =
            currentsettings->add_numeric_box<std::string>(ctrl.name.c_str(),
                ctrl.tooltip.c_str(),
                [this, ctrl](const std::string & v) {
                  if ( !updatingControls() && ctrl.set_value && ctrl.set_value(pipeline_, v) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, ctrl](std::string * v) -> bool {
                  return ctrl.get_value ? ctrl.get_value(pipeline_, v) : false;
                });

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

      case c_image_processor_pipeline_ctl_check_box: {

        QWidget *w =
            currentsettings->add_checkbox(ctrl.name.c_str(),
                ctrl.tooltip.c_str(),
                [this, ctrl](bool checked) {
                  if (!updatingControls() && ctrl.set_value && ctrl.set_value(pipeline_, checked ? "1" : "0") ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, ctrl](bool * checked) {
                  std::string s;
                  if ( ctrl.get_value && ctrl.get_value(pipeline_, &s)) {
                    return fromString(s, checked);
                  }
                  return false;
                });

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

        /////////////////////
      case c_image_processor_pipeline_ctl_enum_combobox: {

        QWidget *w =
            currentsettings->add_enum_combobox_base(ctrl.name.c_str(),
                ctrl.tooltip.c_str(),
                ctrl.get_enum_members(),
                [this, ctrl](int v) {
                  if (!updatingControls() && ctrl.set_value && ctrl.set_value(pipeline_, toString(v))) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, ctrl](int * v) {
                  std::string s;
                  if ( ctrl.get_value && ctrl.get_value(pipeline_, &s) ) {
                    const c_enum_member* m = fromString(s, ctrl.get_enum_members());
                    if (m ) {
                      * v = m->value;
                      return true;
                    }
                  }
                  return false;
                });

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }
        /////////////////////

      case c_image_processor_pipeline_ctl_spinbox: {

        QSpinBox *w =
            currentsettings->add_spinbox(ctrl.name.c_str(),
                ctrl.tooltip.c_str(),
                [this, ctrl](int v) {
                  if (!updatingControls() && ctrl.set_value && ctrl.set_value(pipeline_, toString(v)) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, ctrl](int * v) {
                  std::string s;
                  if ( ctrl.get_value && ctrl.get_value(pipeline_, &s) ) {
                    return fromString(s, v);
                  }
                  return false;
                });

        w->setRange(ctrl.range.min, ctrl.range.max);
        w->setSingleStep(ctrl.range.step);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

        /////////////////////
      case c_image_processor_pipeline_ctl_flags_chkbox: {

        QFlagsEditBoxBase *w =
            currentsettings->add_flags_editbox_base(ctrl.name.c_str(),
                ctrl.tooltip.c_str(),
                ctrl.get_enum_members(),
                [this, ctrl](int v) {
                  if (!updatingControls() && ctrl.set_value && ctrl.set_value(pipeline_, toString(v)) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, ctrl](int * v) {
                  std::string s;
                  if ( ctrl.get_value && ctrl.get_value(pipeline_, &s) ) {
                    *v = flagsFromString(s, ctrl.get_enum_members());
                    return true;
                  }
                  return false;
                });

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

        /////////////////////
      case c_image_processor_pipeline_ctl_browse_for_existing_file: {

        QWidget *w =
            currentsettings->add_browse_for_path(ctrl.name.c_str(),
                "",
                QFileDialog::AcceptOpen,
                QFileDialog::ExistingFile,
                [this, ctrl](const QString & v) {
                  if (!updatingControls() && ctrl.set_value && ctrl.set_value(pipeline_, v.toStdString()) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, ctrl](QString * v) {
                  std::string s;
                  if ( ctrl.get_value && ctrl.get_value(pipeline_, &s) ) {
                    *v = s.c_str();
                    return true;
                  }
                  return false;
                });

        w->setToolTip(ctrl.tooltip.c_str());

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

        /////////////////////
      case c_image_processor_pipeline_ctl_browse_for_directory: {

        QWidget *w =
            currentsettings->add_browse_for_path(ctrl.name.c_str(),
                "",
                QFileDialog::AcceptOpen,
                QFileDialog::Directory,
                [this, ctrl](const QString & v) {
                  if (!updatingControls() && ctrl.set_value && ctrl.set_value(pipeline_, v.toStdString()) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, ctrl](QString * v) {
                  std::string s;
                  if ( ctrl.get_value && ctrl.get_value(pipeline_, &s) ) {
                    *v = s.c_str();
                    return true;
                  }
                  return false;
                });

        w->setToolTip(ctrl.tooltip.c_str());

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

        /////////////////////
      case c_image_processor_pipeline_ctl_image_processor_selection_combo: {

        QImageProcessorSelectionCombo *w =
            currentsettings->add_combobox<QImageProcessorSelectionCombo>(ctrl.name.c_str(),
                ctrl.tooltip.c_str(),
                [this, ctrl](int index,
                    QImageProcessorSelectionCombo * combo) {
                      if( !updatingControls() && ctrl.set_image_processor && ctrl.set_image_processor(pipeline_, combo->processor(index)) ) {
                        Q_EMIT parameterChanged();
                      }
                    },
                [this, ctrl](int * index, QImageProcessorSelectionCombo * combo) -> bool {
                  if ( !ctrl.get_image_processor || !combo->setCurrentProcessor(ctrl.get_image_processor(pipeline_)) ) {
                    if( ctrl.set_image_processor ) {
                      ctrl.set_image_processor(pipeline_, nullptr);
                    }
                  }
                  return false;
                });

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

        /////////////////////

      case c_image_processor_pipeline_ctl_input_source_selection_combo: {

        QInputSourceSelectionCombo *w =
            currentsettings->add_combobox<QInputSourceSelectionCombo>(ctrl.name.c_str(),
                ctrl.tooltip.c_str(),
                [this, ctrl](int index,
                    QInputSourceSelectionCombo * combo) {
                      if( !updatingControls() && ctrl.set_value && ctrl.set_value(pipeline_, combo->itemData(index).toString().toStdString()) ) {
                        Q_EMIT parameterChanged();
                      }
                    },
                [this, ctrl](int * index, QInputSourceSelectionCombo * combo) -> bool {
                  std::string s;
                  if ( ctrl.get_value && ctrl.get_value(pipeline_, &s)) {
                    combo->setCurrentIndex(combo->findData(QString(s.c_str())));
                  }
                  return false;
                });

        inputSourceCombos_.append(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

//      case c_image_processor_pipeline_ctl_image_registration_options: {
//
//        QFrameRegistrationOptions *w =
//            new QFrameRegistrationOptions(this);
//
//        if( ctrl.get_image_registration_options ) {
//          connect(this, &Base::populatecontrols,
//              [this, w, ctrl]() {
//                w->set_registration_options(ctrl.get_image_registration_options(pipeline_));
//              });
//        }
//
//        connect(w, &QSettingsWidget::parameterChanged,
//            [this]() {
//              if ( !updatingControls() ) {
//                Q_EMIT parameterChanged();
//              }
//            });
//
//        currentsettings->addRow(w);
//        inputSourceCombos_.append(w);
//
//        if( ctrl.is_enabled ) {
//          state_ctls_.emplace(w, ctrl.is_enabled);
//        }
//
//        break;
//      }

      case c_image_processor_pipeline_ctl_feature2d_detector_options: {

        QSparseFeatureDetectorOptions *w =
            new QSparseFeatureDetectorOptions(this);

        if( ctrl.get_feature2d_detector_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_feature_detector_options(ctrl.get_feature2d_detector_options(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

      case c_image_processor_pipeline_ctl_feature2d_descriptor_options: {

        QSparseFeatureDescriptorOptions *w =
            new QSparseFeatureDescriptorOptions(this);

        if( ctrl.get_feature2d_descriptor_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_feature_descriptor_options(ctrl.get_feature2d_descriptor_options(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

      case c_image_processor_pipeline_ctl_feature2d_matcher_options: {

        QSparseFeatureMatcherOptions *w =
            new QSparseFeatureMatcherOptions(this);

        if( ctrl.get_feature2d_matcher_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_feature_matcher_options(ctrl.get_feature2d_matcher_options(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

      case c_image_processor_pipeline_ctl_camera_intrinsicts : {

        QCameraIntrinsicsEditBox * w =
            new QCameraIntrinsicsEditBox(this);

        if( ctrl.get_camera_intrinsicts ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_options(ctrl.get_camera_intrinsicts(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }


      case c_image_processor_pipeline_ctl_stereo_matcher_options: {

        QStereoMatcherOptions * w =
            new QStereoMatcherOptions(this);

        if( ctrl.get_stereo_matcher ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_stereo_matcher(ctrl.get_stereo_matcher(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }
        /////////////////////

      case c_image_processor_pipeline_ctl_output_writer: {

        QOutputFrameWriterOptions * w =
            new QOutputFrameWriterOptions(this);

        if( ctrl.get_output_writer_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_options(ctrl.get_output_writer_options(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }
      /////////////////////

      case c_image_processor_pipeline_ctl_master_frame_selection: {

        QMasterFrameSelectionControl * w =
            new QMasterFrameSelectionControl(this);

        if( ctrl.get_master_frame_selection_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_options(ctrl.get_master_frame_selection_options(pipeline_));
              });
        }


        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);
        inputSourceCombos_.append(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

        /////////////////////
      case c_image_processor_pipeline_ctl_feature_registration_options: {

        QFeatureBasedRegistrationOptions * w =
            new QFeatureBasedRegistrationOptions(this);

        if( ctrl.get_feature_registration_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_options(ctrl.get_feature_registration_options(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

      /////////////////////
      case c_image_processor_pipeline_ctl_ecc_registration_options: {

        QEccRegistrationOptions * w =
            new QEccRegistrationOptions(this);

        if( ctrl.get_ecc_registration_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_options(ctrl.get_ecc_registration_options(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

      /////////////////////
      case c_image_processor_pipeline_ctl_eccflow_registration_options : {

        QEccFlowRegistrationOptions * w =
            new QEccFlowRegistrationOptions(this);

        if( ctrl.get_eccflow_registration_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_options(ctrl.get_eccflow_registration_options(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

      /////////////////////
      case c_image_processor_pipeline_ctl_jovian_derotation_options : {

        QJovianDerotationOptions * w =
            new QJovianDerotationOptions(this);

        if( ctrl.get_jovian_derotation_options ) {
          connect(this, &Base::populatecontrols,
              [this, w, ctrl]() {
                w->set_options(ctrl.get_jovian_derotation_options(pipeline_));
              });
        }

        connect(w, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentsettings->addRow(w);

        if( ctrl.is_enabled ) {
          state_ctls_.emplace(w, ctrl.is_enabled);
        }

        break;
      }

      /////////////////////

      default:
        break;
    }
  }

  updateControls();
}

