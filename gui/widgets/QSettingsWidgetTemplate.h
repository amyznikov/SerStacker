/*
 * QSettingsWidgetTemplate.h
 *
 *  Created on: Feb 6, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QSettingsWidgetTemplate2_h__
#define __QSettingsWidgetTemplate2_h__

#include <gui/widgets/QSettingsWidget.h>
#include <gui/qfeature2d/QFeature2dOptions.h>
#include <gui/widgets/QCameraIntrinsicsEditBox.h>
#include <gui/qpipeline/QFrameRegistrationOptions.h>
#include <gui/qpipeline/stereo/QStereoMatcherOptions.h>
#include <gui/qpipeline/QInputSourceSelection.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <core/ctrlbind/ctrlbind.h>
#include <core/debug.h>

template<class QSettingsWidgetType, class RootObjectType>
void setupControls(QSettingsWidgetType * _this, const c_ctlist<RootObjectType> & controls)
{
  std::vector<QWidget*> groups;
  QWidget *currentGroup = nullptr;
  QSettingsWidget *currentSettings = _this;

  static const auto enablefn =
      [](QSettingsWidgetType * _this, const c_ctlbind<RootObjectType> & c) -> std::function<bool()> {
        const auto & enabled = c.enabled;
        return enabled ?  std::function<bool()>(
            [_this, enabled]() -> bool { return _this->opts() && enabled(_this->opts()); }) :
            nullptr;
  };


  for ( const auto & c : controls ) {

    using  CtlType = typename c_ctlbind<RootObjectType>::CtlType;

    switch(c.ctype) {
      ////////////////////////////////////////////////////////////////////////
      case CtlType::None: {
        break;
      }

      case CtlType::BeginGroup: {
        if( currentGroup ) {
          groups.emplace_back(currentGroup);
        }

        QSettingsWidget * newSettings = new QSettingsWidget(currentSettings);
        newSettings->setContentsMargins(0,0,0,0);
        currentSettings->addRow(currentGroup = newSettings);

        QObject::connect(newSettings, &QSettingsWidget::parameterChanged,
            currentSettings, &QSettingsWidget::parameterChanged);

        QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
            newSettings, &QSettingsWidget::updateControls);

        if ( !c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              newSettings, &QSettingsWidget::enablecontrols);
        }
        else {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, newSettings, c]() {
                const bool enabled = _this->opts() && c.enabled(_this->opts());
                newSettings->setEnabled(enabled);
                if ( enabled ) {
                  newSettings->enablecontrols();
                }
              });
        }

        currentSettings = newSettings;
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::BeginExpandableGroup: {
        if( currentGroup ) {
          groups.emplace_back(currentGroup);
        }

        QSettingsWidget * newSettings = new QSettingsWidget(currentSettings);
        QExpandableGroupBox * expandableGroup = currentSettings->add_expandable_groupbox(c.cname.c_str(), newSettings);
        currentGroup = expandableGroup;

        QObject::connect(newSettings, &QSettingsWidget::parameterChanged,
            currentSettings, &QSettingsWidget::parameterChanged);

        QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
            newSettings, &QSettingsWidget::updateControls);

        if ( !c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              newSettings, &QSettingsWidget::enablecontrols);
        }
        else {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, newSettings, c]() {
                const bool enabled = _this->opts() && c.enabled(_this->opts());
                newSettings->setEnabled(enabled);
                if ( enabled ) {
                  newSettings->enablecontrols();
                }
              });
        }

        currentSettings = newSettings;
        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::EndGroup: {
        if( groups.empty() ) {
          currentGroup = nullptr;
          currentSettings = _this;
        }
        else {
          currentGroup = groups.back();
          groups.pop_back();

          if ( QExpandableGroupBox *expandableGroup = dynamic_cast<QExpandableGroupBox *>(currentGroup) ) {

            expandableGroup->expand();
            currentSettings = dynamic_cast<QSettingsWidget *>(expandableGroup->view());
            expandableGroup->collapse();

            QObject::connect(expandableGroup, &QExpandableGroupBox::expanded,
                _this, &QSettingsWidget::groupExpanded);

            QObject::connect(expandableGroup, &QExpandableGroupBox::collapsed,
                _this, &QSettingsWidget::groupCollapsed);
          }
          else {
            currentSettings = dynamic_cast<QSettingsWidget *>(currentGroup);
          }
        }
       break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::Textbox: {
        currentSettings->add_textbox(QString::fromStdString(c.cname),
          QString::fromStdString(c.cdesc),
          [_this, c](const QString & v) -> void {
            if ( _this->opts() && c.setvalue && c.setvalue(_this->opts(), v.toStdString()) ) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, c](QString * v) -> bool {
            if ( _this->opts() ) {
              std::string s;
              if ( c.getvalue(_this->opts(), &s) ) {
                *v = QString::fromStdString(s);
                return true;
              }
            }
            return false;
          },
          enablefn(_this, c));

        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::NumericBox: {
        currentSettings->add_numeric_box<std::string>(QString::fromStdString(c.cname),
          QString::fromStdString(c.cdesc),
          [_this, c](const std::string & v) -> void {
            if ( _this->opts() && c.setvalue && c.setvalue(_this->opts(), v) ) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, c](std::string * v) -> bool {
            return _this->opts() && c.getvalue && c.getvalue(_this->opts(), v);
          },
          enablefn(_this, c));
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::Checkbox: {
        currentSettings->add_checkbox(c.cname.c_str(),
          c.cdesc.c_str(),
          [_this, c](bool checked) {
            if ( _this->opts() && c.setvalue && c.setvalue(_this->opts(), checked ? "1" : "0") ) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, c](bool * checked) {
            if ( _this->opts() && c.getvalue ) {
              std::string s;
              if ( c.getvalue(_this->opts(), &s)) {
                return fromString(s, checked);
              }
            }
            return false;
          },
          enablefn(_this, c));
        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::FlagsCheckbox: {
        currentSettings->add_flags_editbox_base(c.cname.c_str(),
          c.cdesc.c_str(),
          c.get_enum_members(),
          [_this, c](int v) {
            if ( _this->opts() && c.setvalue && c.setvalue(_this->opts(), toString(v)) ) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, c](int * v) {
            if ( _this->opts() && c.getvalue ) {
              std::string s;
              if (  c.getvalue(_this->opts(), &s) ) {
                *v = flagsFromString(s, c.get_enum_members());
                return true;
              }
            }
            return false;
          },
          enablefn(_this, c));
        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::EnumCombobox: {
        currentSettings->add_enum_combobox_base(QString::fromStdString(c.cname),
          QString::fromStdString(c.cdesc),
          c.get_enum_members(),
          [_this, c](int v) {
            if (_this->opts() && c.setvalue && c.setvalue(_this->opts(), toString(v))) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, c](int * v) {
            if ( _this->opts() && c.getvalue ) {
              std::string s;
              if ( c.getvalue(_this->opts(), &s) ) {
                if ( const c_enum_member* m = fromString(s, c.get_enum_members()) ) {
                  * v = m->value;
                  return true;
                }
              }
            }
            return false;
          },
          enablefn(_this, c));
        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::SpinBox: {
        QSpinBox * ctl =
          currentSettings->add_spinbox(c.cname.c_str(),
            c.cdesc.c_str(),
            [_this, c](int v) {
              if (_this->opts() && c.setvalue && c.setvalue(_this->opts(), toString(v)) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, c](int * v) {
              if ( _this->opts() && c.getvalue ) {
                std::string s;
                if ( c.getvalue(_this->opts(), &s) ) {
                  return fromString(s, v);
                }
              }
              return false;
            },
            enablefn(_this, c));

        QSignalBlocker block(ctl);
        ctl->setRange(c.range.min, c.range.max);
        ctl->setSingleStep(c.range.step);
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::DoubleSpinBox: {
        QDoubleSpinBox * ctl =
          currentSettings->add_double_spinbox(c.cname.c_str(),
            c.cdesc.c_str(),
            [_this, c](double v) {
              if (_this->opts() && c.setvalue && c.setvalue(_this->opts(), toString(v)) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, c](double * v) {
              if ( _this->opts() && c.getvalue ) {
                std::string s;
                if ( c.getvalue(_this->opts(), &s) ) {
                  return fromString(s, v);
                }
              }
              return false;
            },
            enablefn(_this, c));

        QSignalBlocker block(ctl);
        ctl->setRange(c.range.min, c.range.max);
        ctl->setSingleStep(c.range.step);
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::DoublesliderSpinBox : {
        QDoubleSliderSpinBox * ctl =
          currentSettings->add_sliderspinbox<double>(c.cname.c_str(),
            c.cdesc.c_str(),
            [_this, c](double v) {
              if (_this->opts() && c.setvalue && c.setvalue(_this->opts(), toString(v)) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, c](double * v) {
              if ( _this->opts() && c.getvalue ) {
                std::string s;
                if ( c.getvalue(_this->opts(), &s) ) {
                  return fromString(s, v);
                }
              }
              return false;
            },
            enablefn(_this, c));

        QSignalBlocker block(ctl);
        ctl->setSingleStep(c.range.step);
        ctl->setRange(c.range.min, c.range.max);
        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::BrowseForDirectory: {
        QWidget * ctl =
          currentSettings->add_browse_for_path(c.cname.c_str(),
            "",
            QFileDialog::AcceptOpen,
            QFileDialog::Directory,
            [_this, c](const QString & v) {
              if ( _this->opts() && c.setvalue && c.setvalue(_this->opts(), v.toStdString()) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, c](QString * v) {
              if ( _this->opts() && c.getvalue  ) {
                std::string s;
                if ( c.getvalue(_this->opts(), &s) ) {
                  *v = QString::fromStdString(s);
                  return true;
                }
              }
              return false;
            },
            enablefn(_this, c));

        ctl->setToolTip(c.cdesc.c_str());
        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::BrowseForExistingFile: {
        QWidget * ctl =
          currentSettings->add_browse_for_path(c.cname.c_str(),
            "",
            QFileDialog::AcceptOpen,
            QFileDialog::ExistingFile,
            [_this, c](const QString & v) {
              if ( _this->opts() && c.setvalue && c.setvalue(_this->opts(), v.toStdString()) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, c](QString * v) {
              if ( _this->opts() && c.getvalue ) {
                std::string s;
                if ( c.getvalue(_this->opts(), &s) ) {
                  *v = QString::fromStdString(s);
                  return true;
                }
              }
              return false;
            },
            enablefn(_this, c));

        ctl->setToolTip(c.cdesc.c_str());
        break;
      }
      ////////////////////////////////////////////////////////////////////////

      case CtlType::MathExpression: {
        QInputMathExpressionWidget * ctl =
          currentSettings->add_math_expression(c.cname.c_str(),
              c.cdesc.c_str(),

              [_this, c](const QString & v) {
                if ( _this->opts() && c.setvalue && c.setvalue(_this->opts(), v.toStdString()) ) {
                  Q_EMIT _this->parameterChanged();
                }
              },

              [_this, c](QString * v) {
                if ( _this->opts() && c.getvalue ) {
                  std::string s;
                  if ( c.getvalue(_this->opts(), &s) ) {
                    *v = QString::fromStdString(s);
                    return true;
                  }
                }
                return false;
              },

              c.helpstring ? std::function(
                  [_this, c](QString * v) {
                    return _this->opts() ? *v = QString::fromStdString(c.helpstring(_this->opts())), true : false;
                  }) :  nullptr,

              enablefn(_this, c));

        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::CommandButton : {
        QToolButton * ctl = new QToolButton(_this);
        QSignalBlocker block(ctl);
        ctl->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonTextOnly);
        ctl->setText(c.cname.c_str());
        ctl->setToolTip(c.cdesc.c_str());
        currentSettings->addRow(ctl);

        if (c.onclick ) {
         QObject::connect(ctl, &QToolButton::clicked,
             [_this, c]() {
               if ( _this->opts() && c.onclick(_this->opts()) ) {
                 _this->updateControls();
                 Q_EMIT _this->parameterChanged();
               }
         });
        }

        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::MenuButton: {
        QToolButton * ctl = new QToolButton(_this);
        QSignalBlocker block(ctl);
        ctl->setToolButtonStyle(c.cname.empty() ? Qt::ToolButtonIconOnly : Qt::ToolButtonTextOnly);
        //ctl->setToolButtonStyle(p.ctl_name.empty() ? Qt::ToolButtonIconOnly : Qt::ToolButtonTextBesideIcon);
        // ctl->setIcon(getIcon(ICON_menu));
        ctl->setText(c.cname.c_str());
        ctl->setToolTip(c.cdesc.c_str());
        currentSettings->addRow(ctl);

        if ( !c.menu.empty() ) {

          QObject::connect(ctl, &QToolButton::clicked,
              [_this, c, ctl]() {
               if ( _this->opts() ) {
                  QMenu menu;
                  for ( const auto & item : c.menu ) {
                    menu.addAction(item.name.c_str(),
                        [_this, &item]() {
                          if ( item.onclick(_this->opts()) ) {
                            _this->updateControls();
                            Q_EMIT _this->parameterChanged();
                          }
                        });
                  }

                  if ( !menu.isEmpty() ) {
                    menu.exec(ctl->mapToGlobal(QPoint(6, 6)));
                  }
               }
              });
        }
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::ImageProcessorCombobox: {
        currentSettings->add_combobox<QImageProcessorSelectionCombo2>(c.cname.c_str(),
            c.cdesc.c_str(),
            false,
            [_this, c](int index, QImageProcessorSelectionCombo2 * combo) {
              if( _this->opts() && c.setvalue && c.setvalue(_this->opts(), combo->processor(index).toStdString()) ) {
                Q_EMIT _this->parameterChanged();
             }},
            [_this, c](int * index, QImageProcessorSelectionCombo2 * combo) -> bool {
              std::string s;
              if ( c.getvalue ) {
                c.getvalue(_this->opts(), &s);
              }
              combo->setCurrentProcessor(QString::fromStdString(s));
              return true;
             },
             enablefn(_this, c));

        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::CameraIntrinsicts: {
        QCameraIntrinsicsEditBox * ctl = new QCameraIntrinsicsEditBox(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.camera_intrinsicts ) {

          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, ctl, c]() {
               ctl->set_camera_intrinsics(c.camera_intrinsicts(_this->opts()));
              });

          QObject::connect(ctl, &QSparseFeatureMatcherOptions::parameterChanged,
              currentSettings, &QSettingsWidget::parameterChanged);
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::SparseFeatureDetector: {
        QSparseFeatureDetectorOptions * ctl = new QSparseFeatureDetectorOptions(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.sparse_feature_detector ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, c, ctl]() {
                ctl->setOpts(c.sparse_feature_detector(_this->opts()));
          });

          QObject::connect(ctl, &QSparseFeatureMatcherOptions::parameterChanged,
              currentSettings, &QSettingsWidget::parameterChanged);
          }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }

        break;
      }


      ////////////////////////////////////////////////////////////////////////
      case CtlType::SparseFeatureDescriptor: {
        QSparseFeatureDescriptorOptions * ctl = new QSparseFeatureDescriptorOptions(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.sparse_feature_descriptor ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, c, ctl]() {
                ctl->setOpts(c.sparse_feature_descriptor(_this->opts()));
          });

          QObject::connect(ctl, &QSparseFeatureDescriptorOptions::parameterChanged,
              _this, &QSettingsWidget::parameterChanged);
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }

        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::SparseFeatureMatcher: {
        QSparseFeatureMatcherOptions * ctl = new QSparseFeatureMatcherOptions(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.sparse_feature_matcher ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, c, ctl]() {
                ctl->setOpts(c.sparse_feature_matcher(_this->opts()));
          });

          QObject::connect(ctl, &QSparseFeatureMatcherOptions::parameterChanged,
              currentSettings, &QSettingsWidget::parameterChanged);
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }

        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::FeatureRegistrationOptions: {
        QFeatureBasedRegistrationOptions * ctl = new QFeatureBasedRegistrationOptions(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.feature_registration_options ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, c, ctl]() {
                ctl->setOpts(c.feature_registration_options(_this->opts()));
          });

          QObject::connect(ctl, &QFeatureBasedRegistrationOptions::parameterChanged,
              currentSettings, &QSettingsWidget::parameterChanged);
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::ECCRegistrationOptions: {
        QEccRegistrationOptions * ctl = new QEccRegistrationOptions(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.ecc_registration_options ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, c, ctl]() {
                ctl->setOpts(c.ecc_registration_options(_this->opts()));
          });

          QObject::connect(ctl, &QFeatureBasedRegistrationOptions::parameterChanged,
              currentSettings, &QSettingsWidget::parameterChanged);
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }

        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::ECCFlowRegistrationOptions: {
        QEccFlowRegistrationOptions * ctl = new QEccFlowRegistrationOptions(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.eccflow_registration_options ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, c, ctl]() {
                ctl->setOpts(c.eccflow_registration_options(_this->opts()));
          });

          QObject::connect(ctl, &QFeatureBasedRegistrationOptions::parameterChanged,
              currentSettings, &QSettingsWidget::parameterChanged);
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }

        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::StereoInputSourceSelection: {
        QStereoInputSourceSelection * ctl = new QStereoInputSourceSelection(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.stereo_input_source_options ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, c, ctl]() {
                ctl->setOpts(c.stereo_input_source_options(_this->opts()));
          });

          QObject::connect(ctl, &QFeatureBasedRegistrationOptions::parameterChanged,
              currentSettings, &QSettingsWidget::parameterChanged);
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }

        break;
      }


      ////////////////////////////////////////////////////////////////////////
      case CtlType::MasterFrameSelection: {
        QMasterFrameSelection * ctl = new QMasterFrameSelection(_this);
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc));
        if ( c.cname.empty() ) {
          currentSettings->addRow(ctl);
        }
        else {
          currentSettings->addRow(QString::fromStdString(c.cname), ctl);
        }

        if( c.master_frame_selection ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, ctl, c]() {
                ctl->setOpts(c.master_frame_selection(_this->opts()));
              });

          QObject::connect(ctl, &QFeatureBasedRegistrationOptions::parameterChanged,
              currentSettings, &QSettingsWidget::parameterChanged);
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, c, ctl]() {
                ctl->setEnabled(_this->opts() && c.enabled(_this->opts()));
          });
        }

        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::DataAnnotationSelector:{
        break;
      }
      ////////////////////////////////////////////////////////////////////////
    }
  }

}

#endif /* __QSettingsWidgetTemplate2_h__ */
