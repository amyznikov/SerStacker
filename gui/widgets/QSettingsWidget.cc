/*
 * QSettingsWidget.cc
 *
 *  Created on: Jan 11, 2019
 *      Author: amyznikov
 */

#include "QSettingsWidget.h"
#include <gui/qfeature2d/QFeature2dOptions.h>
#include <gui/widgets/style.h>

#define ICON_menu   ":/gui/icons/menu"

QSettingsWidget::QSettingsWidget(const QString & prefix, QWidget * parent) :
  Base(parent), PREFIX(prefix)
{
  setFrameShape(NoFrame);
  form = new QFormLayout(this);

  connect(this, &ThisClass::parameterChanged,
      this, &ThisClass::update_control_states);
}

void QSettingsWidget::setSettingsPrefix(const QString & v)
{
  PREFIX = v;
}

const QString & QSettingsWidget::settingsPrefix() const
{
  return PREFIX;
}


void QSettingsWidget::set_mutex(std::mutex * mtx)
{
  this->mtx_ = mtx;
}

std::mutex * QSettingsWidget::mutex()
{
  return this->mtx_;

}

void QSettingsWidget::lock()
{
  if ( mtx_ ) {
    mtx_->lock();
  }
}

void QSettingsWidget::unlock()
{
  if ( mtx_ ) {
    mtx_->unlock();
  }
}

bool QSettingsWidget::updatingControls()
{
  return updatingControls_ > 0;
}

void QSettingsWidget::setUpdatingControls(bool v)
{
  if ( v ) {
    ++updatingControls_;
  }
  else if ( updatingControls_ && --updatingControls_ < 0 ) {
    updatingControls_ = 0;
  }
}

void QSettingsWidget::updateControls()
{
  setUpdatingControls(true);
  onupdatecontrols();
  setUpdatingControls(false);
}

void QSettingsWidget::loadParameters()
{
  QSettings settings;
  loadSettings(settings);
}

void QSettingsWidget::loadSettings(QSettings & settings)
{
  onload(settings);
  updateControls();
}


void QSettingsWidget::onload(QSettings & /*settings*/)
{
}

void QSettingsWidget::onupdatecontrols()
{
  populatecontrols();
}

#ifdef __ctrlbind_h__

void QSettingsWidget::update_control_states()
{
  for( auto &p : _bound_state_ctls ) {
    QWidget *w = p.first;
    const auto &is_enabled = p.second;
    w->setEnabled(is_enabled());
  }
}

void QSettingsWidget::setup_controls(const std::vector<c_ctrl_bind> & ctls)
{
  std::vector<QExpandableGroupBox*> groups;
  QExpandableGroupBox *currentGroup = nullptr;
  QSettingsWidget *currentSettings = this;

  for( const struct c_ctrl_bind & p : ctls ) {

    switch (p.ctl_type) {

      case ctrl_bind_begin_group: {

        if( currentGroup ) {
          groups.emplace_back(currentGroup);
        }

        currentGroup =
            currentSettings->add_expandable_groupbox(p.ctl_name.c_str(),
                currentSettings = new QSettingsWidget("", this));

        connect(currentSettings, &QSettingsWidget::parameterChanged,
            [this]() {
              if ( !updatingControls() ) {
                Q_EMIT parameterChanged();
              }
            });

        currentGroup->expand();

        connect(this, &QSettingsWidget::populatecontrols,
            currentSettings, &QSettingsWidget::populatecontrols);

        connect(currentGroup, &QExpandableGroupBox::expanded,
            this, &ThisClass::groupExpanded);

        connect(currentGroup, &QExpandableGroupBox::collapsed,
            this, &ThisClass::groupCollapsed);

        break;
      }

      case ctrl_bind_end_group: {

        if( groups.empty() ) {
          currentGroup = nullptr;
          currentSettings = this;
        }
        else {
          currentGroup = groups.back();
          currentGroup->expand();
          currentSettings = (QSettingsWidget*) currentGroup->view();
          groups.pop_back();
        }

        break;
      }

      case ctrl_bind_numeric_box: {

        QWidget * ctl =
            currentSettings->add_numeric_box<std::string>(p.ctl_name.c_str(),
                p.ctl_tooltip.c_str(),
                [this, p](const std::string & v) {
                  if ( !updatingControls() && p.set_value && p.set_value(v) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](std::string * v) -> bool {
                  return p.get_value ? p.get_value(v) : false;
                });

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }

        break;
      }

      case ctrl_bind_enum_combobox: {

        QWidget * ctl =
            currentSettings->add_enum_combobox_base(p.ctl_name.c_str(),
                p.ctl_tooltip.c_str(),
                p.get_enum_members(),
                [this, p](int v) {
                  if (!updatingControls() && p.set_value && p.set_value(toString(v))) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](int * v) {
                  std::string s;
                  if ( p.get_value && p.get_value(&s) ) {
                    const c_enum_member* m = fromString(s, p.get_enum_members());
                    if (m ) {
                      * v = m->value;
                      return true;
                    }
                  }
                  return false;
                });

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }
        break;
      }

      case ctrl_bind_check_box: {

        QWidget * ctl =
            currentSettings->add_checkbox(p.ctl_name.c_str(),
                p.ctl_tooltip.c_str(),
                [this, p](bool checked) {
                  if (!updatingControls() && p.set_value && p.set_value(checked ? "1" : "0") ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](bool * checked) {
                  std::string s;
                  if ( p.get_value && p.get_value(&s)) {
                    return fromString(s, checked);
                  }
                  return false;
                });

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }

        break;
      }

      case ctrl_bind_flags_chkbox: {

        QFlagsEditBoxBase * ctl =
            currentSettings->add_flags_editbox_base(p.ctl_name.c_str(),
                p.ctl_tooltip.c_str(),
                p.get_enum_members(),
                [this, p](int v) {
                  if (!updatingControls() && p.set_value && p.set_value(toString(v)) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](int * v) {
                  std::string s;
                  if ( p.get_value && p.get_value(&s) ) {
                    *v = flagsFromString(s, p.get_enum_members());
                    return true;
                  }
                  return false;
                });

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }

        break;
      }

      case ctrl_bind_spinbox: {

        QSpinBox * ctl =
            currentSettings->add_spinbox(p.ctl_name.c_str(),
                p.ctl_tooltip.c_str(),
                [this, p](int v) {
                  if (!updatingControls() && p.set_value && p.set_value(toString(v)) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](int * v) {
                  std::string s;
                  if ( p.get_value && p.get_value(&s) ) {
                    return fromString(s, v);
                  }
                  return false;
                });

        ctl->setRange(p.range.min, p.range.max);
        ctl->setSingleStep(p.range.step);

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }

        break;
      }

      case ctrl_bind_double_slider : {

        QDoubleSliderSpinBox * ctl =
            currentSettings->add_sliderspinbox<double>(p.ctl_name.c_str(),
                p.ctl_tooltip.c_str(),
                [this, p](double v) {
                  if (!updatingControls() && p.set_value && p.set_value(toString(v)) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](double * v) {
                  std::string s;
                  if ( p.get_value && p.get_value(&s) ) {
                    return fromString(s, v);
                  }
                  return false;
                });

        ctl->setRange(p.range.min, p.range.max);
        ctl->setSingleStep(p.range.step);

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }

        break;
      }


      case ctrl_bind_browse_for_existing_file: {

        QWidget * ctl =
            currentSettings->add_browse_for_path(p.ctl_name.c_str(),
                "",
                QFileDialog::AcceptOpen,
                QFileDialog::ExistingFile,
                [this, p](const QString & v) {
                  if (!updatingControls() && p.set_value && p.set_value(v.toStdString()) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](QString * v) {
                  std::string s;
                  if ( p.get_value && p.get_value(&s) ) {
                    *v = s.c_str();
                    return true;
                  }
                  return false;
                });

        ctl->setToolTip(p.ctl_tooltip.c_str());

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }

        break;
      }

      case ctrl_bind_browse_for_directory: {

        QWidget * ctl =
            currentSettings->add_browse_for_path(p.ctl_name.c_str(),
                "",
                QFileDialog::AcceptOpen,
                QFileDialog::Directory,
                [this, p](const QString & v) {
                  if (!updatingControls() && p.set_value && p.set_value(v.toStdString()) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](QString * v) {
                  std::string s;
                  if ( p.get_value && p.get_value(&s) ) {
                    *v = s.c_str();
                    return true;
                  }
                  return false;
                });

        ctl->setToolTip(p.ctl_tooltip.c_str());

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }

        break;
      }

      case ctrl_bind_math_expression: {

        QInputMathExpressionWidget * ctl =
            currentSettings->add_math_expression(p.ctl_name.c_str(),
                p.ctl_tooltip.c_str(),
                [this, p](const QString & v) {
                  if (!updatingControls() && p.set_value && p.set_value(v.toStdString()) ) {
                    Q_EMIT parameterChanged();
                  }
                },
                [this, p](QString * v) {
                  std::string s;
                  if ( p.get_value && p.get_value(&s) ) {
                    *v = s.c_str();
                    return true;
                  }
                  return false;
                });

        if ( p.helpstring ) {
          ctl->setHelpString(p.helpstring().c_str());
        }

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }


        break;
      }

#ifdef __QFeature2dOptions_h__

      case ctrl_bind_sparse_feature_detector: {

        QSparseFeatureDetectorOptions * ctl =
            new QSparseFeatureDetectorOptions(this);


        ctl->setToolTip(p.ctl_tooltip.c_str());

        currentSettings->addRow(p.ctl_name.c_str(), ctl);

        if( p.sparse_feature_detector ) {

          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                ctl->set_feature_detector_options(p.sparse_feature_detector());
              });

          QObject::connect(ctl, &QSparseFeatureDetectorOptions::parameterChanged,
              [this]() {
                if ( !updatingControls() ) {
                  Q_EMIT parameterChanged();
                }
              });
        }

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }


        break;
      }
#endif // __QFeature2dOptions_h__

      case ctrl_bind_data_annotation_selector: {

        QDataAnnotationSelectorCtrl * ctl =
            new QDataAnnotationSelectorCtrl(this);

        ctl->setToolTip(p.ctl_tooltip.c_str());

        currentSettings->addRow(p.ctl_name.c_str(), ctl);

        if( p.get_data_annotation ) {

          static const auto updateAnnotationLabels =
            [](QDataAnnotationSelectorCtrl *ctl, const c_ctrl_bind & p) {
              int lb;
              const int cmaps = ctl->num_colormaps();
              for ( int cmap = 0; cmap < cmaps; ++cmap ) {
                if ( p.get_data_annotation(cmap, &lb) ) {
                  ctl->setAnnotationLabel(cmap, lb);
                }
              }
            };


          QObject::connect(this, &ThisClass::populatecontrols,
              [ctl, p]() {
                updateAnnotationLabels(ctl, p);
              });


          const int change_handler_id =
              c_data_annotation_labels::default_instance().add_change_handler([ctl, p]() {
                updateAnnotationLabels(ctl, p);
              });

          connect(this, &QObject::destroyed,
              [change_handler_id]() {
                c_data_annotation_labels::default_instance().remove_change_handler(change_handler_id);
              });

        }

        if (p.set_data_annotation) {

          QObject::connect(ctl, &QDataAnnotationSelectorCtrl::annotationLabelChanged,
              [this, ctl, p](int cmap, int label) {
                if ( !updatingControls() && p.set_data_annotation && p.set_data_annotation(cmap, label) ) {
                  Q_EMIT parameterChanged();
                }
              });
        }

        if( p.is_enabled ) {
          ctl->setEnabled(p.is_enabled());
          _bound_state_ctls.emplace(ctl, p.is_enabled);
        }

        break;
      }


      case ctrl_bind_command_button: {

        QToolButton * ctl =
            new QToolButton(this);

        ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonTextOnly);
        ctl->setText(p.ctl_name.c_str());
        ctl->setToolTip(p.ctl_tooltip.c_str());
        currentSettings->addRow(ctl);

        if (p.on_button_click ) {
          QObject::connect(ctl, &QToolButton::clicked,
              [this, p]() {
                if ( p.on_button_click() ) {
                  updateControls();
                  Q_EMIT parameterChanged();
                }
          });
        }

        break;
      }

      case ctrl_bind_menu_button: {
        QToolButton * ctl =
            new QToolButton(this);

        ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonTextBesideIcon);
        ctl->setIcon(getIcon(ICON_menu));
        ctl->setText(p.ctl_name.c_str());
        ctl->setToolTip(p.ctl_tooltip.c_str());
        currentSettings->addRow(ctl);

        if ( !p.menu_commands.empty() ) {

          QObject::connect(ctl, &QToolButton::clicked,
              [this, p, ctl]() {

                QMenu menu;
                for ( const auto & item : p.menu_commands ) {
                  menu.addAction(item.command_name.c_str(),
                      [this, &item]() {
                        if ( item.onclick() ) {
                          updateControls();
                          Q_EMIT parameterChanged();
                        }
                      });
                }

                if ( !menu.isEmpty() ) {
                  menu.exec(ctl->mapToGlobal(QPoint(6, 6)));
                }
              });
        }
        break;
      }


      // case ctrl_bind_string_combobox:
      default:
        break;
    }
  }

}
#endif
