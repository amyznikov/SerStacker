/*
 * QPipelineOptionsCtrl.cc
 *
 *  Created on: Jul 20, 2023
 *      Author: amyznikov
 */

#include "QPipelineSettingsCtrl.h"
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/ssprintf.h>

///////////////////////////////////////////////////////////////////////////////


QInputSourceSelectionCombo::QInputSourceSelectionCombo(QWidget * parent) :
  Base(parent)
{
  setEditable(false);
}

void QInputSourceSelectionCombo::setEnableExternalFile(bool v)
{
  enableExternalFile_ = v;
}

bool QInputSourceSelectionCombo::enableExternalFile() const
{
  return enableExternalFile_;
}

void QInputSourceSelectionCombo::refreshInputSources(const c_image_processing_pipeline * pipeline)
{
  Base::clear();

  if ( !pipeline ) {
    setEnabled(false);
    return;
  }

  const c_input_sequence::sptr & input_sequence =
      pipeline->input_sequence();

  if ( !input_sequence ) {
    setEnabled(false);
    return;
  }

  for ( const c_input_source::sptr & source : input_sequence->sources() ) {
    Base::addItem(QFileInfo(source->cfilename()).fileName(), QString(source->cfilename()));
  }

  if ( enableExternalFile_ ) {
    Base::addItem("Browse...");
  }
}


///////////////////////////////////////////////////////////////////////////////

QPipelineSettingsCtrl::QPipelineSettingsCtrl(QWidget * parent) :
  Base("", parent)
{
  connect(this, &ThisClass::parameterChanged,
      this, &ThisClass::update_control_states);
}

QPipelineSettingsCtrl::QPipelineSettingsCtrl(const std::vector<c_image_processing_pipeline_ctrl> & ctrls, QWidget * parent) :
  Base("", parent)
{
  connect(this, &ThisClass::parameterChanged,
      this, &ThisClass::update_control_states);

  setup_controls(ctrls);
  updateControls();
}

void QPipelineSettingsCtrl::set_pipeline(c_image_processing_pipeline * pipeline)
{
  pipeline_ = pipeline;
  update_pipeline_input_sources();
  updateControls();
}

c_image_processing_pipeline * QPipelineSettingsCtrl::pipeline() const
{
  return pipeline_;
}

void QPipelineSettingsCtrl::onupdatecontrols()
{
  if( !pipeline_ ) {
    setEnabled(false);
  }
  else {
    Q_EMIT Base::populatecontrols();
    update_control_states();
    setEnabled(true);
  }
}

void QPipelineSettingsCtrl::update_control_states()
{
  if( pipeline_ ) {
    for( auto &p : state_ctls_ ) {
      QWidget *w = p.first;
      const std::function<bool(const c_image_processing_pipeline*)> &is_enabled = p.second;
      w->setEnabled(is_enabled(pipeline_));
    }
  }
}


void QPipelineSettingsCtrl::update_pipeline_input_sources()
{
  for ( QInputSourceSelectionCombo * combo : inputSourceCombos_ ) {
    combo->refreshInputSources(pipeline_);
  }
}


void QPipelineSettingsCtrl::setup_controls(const std::vector<c_image_processing_pipeline_ctrl> & ctrls)
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
            this, &ThisClass::parameterChanged);

        connect(this, &QSettingsWidget::populatecontrols,
            currentsettings, &QSettingsWidget::populatecontrols);

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
                  if ( ctrl.set_value && ctrl.set_value(pipeline_, v) ) {
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
                  if (ctrl.set_value && ctrl.set_value(pipeline_, checked ? "1" : "0") ) {
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
                  if (ctrl.set_value && ctrl.set_value(pipeline_, toString(v))) {
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
                  if (ctrl.set_value && ctrl.set_value(pipeline_, toString(v)) ) {
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

        QFlagsEditBoxBase * w =
            currentsettings->add_flags_editbox_base(ctrl.name.c_str(),
                ctrl.tooltip.c_str(),
                ctrl.get_enum_members(),
                [this, ctrl](int v) {
                  if (ctrl.set_value && ctrl.set_value(pipeline_, toString(v)) ) {
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
                  if (ctrl.set_value && ctrl.set_value(pipeline_, v.toStdString()) ) {
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
                  if (ctrl.set_value && ctrl.set_value(pipeline_, v.toStdString()) ) {
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
                [this, ctrl](int index, QImageProcessorSelectionCombo * combo) {
                  if( ctrl.set_processor && ctrl.set_processor(pipeline_, combo->processor(index)) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, ctrl](int * index, QImageProcessorSelectionCombo * combo) -> bool {
                  if ( !ctrl.get_processor || !combo->setCurrentProcessor(ctrl.get_processor(pipeline_)) ) {
                    if( ctrl.set_processor ) {
                      ctrl.set_processor(pipeline_, nullptr);
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
                [this, ctrl](int index, QInputSourceSelectionCombo * combo) {
                  if( ctrl.set_value && ctrl.set_value(pipeline_, combo->itemData(index).toString().toStdString()) ) {
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


      /////////////////////
      default:
        break;
    }
  }

  updateControls();
}
