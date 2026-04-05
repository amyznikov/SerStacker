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
#include <gui/widgets/QHFlowLayout.h>
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
              [_this, newSettings, enabled = c.enabled]() {
                const bool enable = _this->opts() && enabled(_this->opts());
                newSettings->setEnabled(enable);
                if ( enable ) {
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
              [_this, newSettings, enabled = c.enabled]() {
                const bool enable = _this->opts() && enabled(_this->opts());
                newSettings->setEnabled(enable);
                if ( enable ) {
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
          [_this, setvalue = c.setvalue](const QString & v) -> void {
            if ( _this->opts() && setvalue && setvalue(_this->opts(), v.toStdString()) ) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, getvalue = c.getvalue](QString * v) -> bool {
            if ( _this->opts() && getvalue) {
              std::string s;
              if ( getvalue(_this->opts(), &s) ) {
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
          [_this, setvalue = c.setvalue](const std::string & v) -> void {
            if ( _this->opts() && setvalue && setvalue(_this->opts(), v) ) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, getvalue = c.getvalue](std::string * v) -> bool {
            return _this->opts() && getvalue && getvalue(_this->opts(), v);
          },
          enablefn(_this, c));
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::Checkbox: {
        currentSettings->add_checkbox(c.cname.c_str(),
          c.cdesc.c_str(),
          [_this, setvalue = c.setvalue](bool checked) {
            if ( _this->opts() && setvalue && setvalue(_this->opts(), checked ? "1" : "0") ) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, getvalue = c.getvalue](bool * checked) {
            if ( _this->opts() && getvalue ) {
              std::string s;
              if ( getvalue(_this->opts(), &s)) {
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
          [_this, setvalue = c.setvalue](int v) {
            if ( _this->opts() && setvalue && setvalue(_this->opts(), toString(v)) ) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, getvalue = c.getvalue, get_enum_members = c.get_enum_members](int * v) {
            if ( _this->opts() && getvalue ) {
              std::string s;
              if (  getvalue(_this->opts(), &s) ) {
                *v = flagsFromString(s, get_enum_members());
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
          [_this, setvalue = c.setvalue](int v) {
            if (_this->opts() && setvalue && setvalue(_this->opts(), toString(v))) {
              Q_EMIT _this->parameterChanged();
            }
          },
          [_this, getvalue = c.getvalue, get_enum_members = c.get_enum_members](int * v) {
            if ( _this->opts() && getvalue ) {
              std::string s;
              if ( getvalue(_this->opts(), &s) ) {
                if ( const c_enum_member* m = fromString(s, get_enum_members()) ) {
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
            [_this, setvalue = c.setvalue](int v) {
              if (_this->opts() && setvalue && setvalue(_this->opts(), toString(v)) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, getvalue = c.getvalue](int * v) {
              if ( _this->opts() && getvalue ) {
                std::string s;
                if ( getvalue(_this->opts(), &s) ) {
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
            [_this, setvalue = c.setvalue](double v) {
              if (_this->opts() && setvalue && setvalue(_this->opts(), toString(v)) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, getvalue = c.getvalue](double * v) {
              if ( _this->opts() && getvalue ) {
                std::string s;
                if ( getvalue(_this->opts(), &s) ) {
                  return fromString(s, v);
                }
              }
              return false;
            },
            enablefn(_this, c));

        QSignalBlocker block(ctl);
        ctl->setRange(c.range.min, c.range.max);
        ctl->setSingleStep(c.range.step);
        ctl->setDecimals((c.range.step > 0) ? std::max(0, (int)std::ceil(-std::log10(c.range.step))) : 2);
        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::SliderSpinBox : {
        QIntegerSliderSpinBox * ctl =
          currentSettings->add_sliderspinbox<int>(c.cname.c_str(),
            c.cdesc.c_str(),
            [_this, setvalue = c.setvalue](int v) {
              if (_this->opts() && setvalue && setvalue(_this->opts(), toString(v)) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, getvalue = c.getvalue](int * v) {
              if ( _this->opts() && getvalue ) {
                std::string s;
                if ( getvalue(_this->opts(), &s) ) {
                  return fromString(s, v);
                }
              }
              return false;
            },
            enablefn(_this, c));

        QSignalBlocker block(ctl);
        ctl->setRange(c.range.min, c.range.max);
        //ctl->setSingleStep(c.range.step);
        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::DoubleSliderSpinBox : {
        QDoubleSliderSpinBox * ctl =
          currentSettings->add_sliderspinbox<double>(c.cname.c_str(),
            c.cdesc.c_str(),
            [_this, setvalue = c.setvalue](double v) {
              if (_this->opts() && setvalue && setvalue(_this->opts(), toString(v)) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, getvalue = c.getvalue](double * v) {
              if ( _this->opts() && getvalue ) {
                std::string s;
                if ( getvalue(_this->opts(), &s) ) {
                  return fromString(s, v);
                }
              }
              return false;
            },
            enablefn(_this, c));

        QSignalBlocker block(ctl);
        ctl->setRange(c.range.min, c.range.max);
        ctl->setSingleStep(c.range.step);
        ctl->setDecimals((c.range.step > 0) ? std::max(0, (int)std::ceil(-std::log10(c.range.step))) : 2);

//        double step = ctl->singleStep();
//        double minv = ctl->minimum();
//        double maxv = ctl->maximum();
//        CF_DEBUG("DoubleSliderSpinBox: c: min=%g max=%g step=%g ctl: min=%g max=%g step=%g ",
//            c.range.min, c.range.max, c.range.step,
//            minv, maxv, step);

        break;
      }
      ////////////////////////////////////////////////////////////////////////
      case CtlType::BrowseForDirectory: {
        QWidget * ctl =
          currentSettings->add_browse_for_path(c.cname.c_str(),
            "",
            QFileDialog::AcceptOpen,
            QFileDialog::Directory,
            [_this, setvalue = c.setvalue](const QString & v) {
              if ( _this->opts() && setvalue && setvalue(_this->opts(), v.toStdString()) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, getvalue = c.getvalue](QString * v) {
              if ( _this->opts() && getvalue  ) {
                std::string s;
                if ( getvalue(_this->opts(), &s) ) {
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
            [_this, setvalue = c.setvalue](const QString & v) {
              if ( _this->opts() && setvalue && setvalue(_this->opts(), v.toStdString()) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, getvalue = c.getvalue](QString * v) {
              if ( _this->opts() && getvalue ) {
                std::string s;
                if ( getvalue(_this->opts(), &s) ) {
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

      case CtlType::MultilineTextbox : {
        QMultiLineEditBox * ctl = currentSettings->add_widget<QMultiLineEditBox>(QString::fromStdString(c.cname));
        QSignalBlocker block(ctl);
        ctl->setToolTip(QString::fromStdString(c.cdesc) + "\n" + ctl->toolTip());

        if( c.setvalue ) {
          QObject::connect(ctl, &QMultiLineEditBox::editFinished,
              [_this, ctl, setvalue = c.setvalue]() {
                if ( _this->opts() && setvalue(_this->opts(), ctl->toPlainText().toStdString()) ) {
                  Q_EMIT _this->parameterChanged();
                }
              });
        }

        if( c.getvalue ) {
          QObject::connect(currentSettings, &QSettingsWidget::populatecontrols,
              [_this, ctl, getvalue = c.getvalue]() {
                std::string s;
                getvalue(_this->opts(), &s);
                ctl->setPlainText(QString::fromStdString(s));
              });
        }

        if( c.enabled ) {
          QObject::connect(currentSettings, &QSettingsWidget::enablecontrols,
              [_this, ctl, enabled = c.enabled]() {
                ctl->setEnabled(_this->opts() && enabled(_this->opts()));
          });
        }

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
            [_this, onclick = c.onclick]() {
               if ( _this->opts() && onclick(_this->opts()) ) {
                 _this->updateControls();
               }
               Q_EMIT _this->parameterChanged();
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

        if ( !c.items.empty() ) {

          QObject::connect(ctl, &QToolButton::clicked,
              [_this, c, ctl]() {
               if ( _this->opts() ) {
                  QMenu menu;
                  for ( const auto & item : c.items ) {
                    menu.addAction(item.name.c_str(),
                        [_this, &item]() {
                          if ( item.onclick(_this->opts()) ) {
                            _this->updateControls();
                          }
                          Q_EMIT _this->parameterChanged();
                        });
                  }

                  if ( !menu.isEmpty() ) {
                    menu.exec(ctl->mapToGlobal(QPoint(8, 8)));
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
            [_this, setvalue = c.setvalue](int index, QImageProcessorSelectionCombo2 * combo) {
              if( _this->opts() && setvalue && setvalue(_this->opts(), combo->processor(index).toStdString()) ) {
                Q_EMIT _this->parameterChanged();
             }},
            [_this, getvalue = c.getvalue](int * index, QImageProcessorSelectionCombo2 * combo) -> bool {
              std::string s;
              if ( getvalue ) {
                getvalue(_this->opts(), &s);
              }
              combo->setCurrentProcessor(QString::fromStdString(s));
              return false;
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
              [_this, ctl, enabled = c.enabled]() {
                ctl->setEnabled(_this->opts() && enabled(_this->opts()));
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
              [_this, ctl, enabled = c.enabled]() {
                ctl->setEnabled(_this->opts() && enabled(_this->opts()));
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
              [_this, ctl, enabled = c.enabled]() {
                ctl->setEnabled(_this->opts() && enabled(_this->opts()));
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
              [_this, ctl, enabled = c.enabled]() {
                ctl->setEnabled(_this->opts() && enabled(_this->opts()));
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
              [_this, ctl, enabled = c.enabled]() {
                ctl->setEnabled(_this->opts() && enabled(_this->opts()));
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
              [_this, ctl, enabled = c.enabled]() {
                ctl->setEnabled(_this->opts() && enabled(_this->opts()));
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
              [_this, ctl, enabled = c.enabled]() {
                ctl->setEnabled(_this->opts() && enabled(_this->opts()));
          });
        }

        break;
      }

      ////////////////////////////////////////////////////////////////////////
      case CtlType::DataAnnotationSelector:{
        currentSettings->add_data_annotation_ctl(QString::fromStdString(c.cname),
            QString::fromStdString(c.cdesc),
            [_this, c](int cmap, int lb) {
              if( _this->opts() && c.set_data_annotation(_this->opts(), cmap, lb) ) {
                Q_EMIT _this->parameterChanged();
              }
            },
            [_this, c](int cmap, int * lb) {
              if ( _this->opts() && c.get_data_annotation ) {
                return c.get_data_annotation(_this->opts(), cmap, lb);
              }
              return false;
            },
            enablefn(_this, c));
        break;
      }
      ////////////////////////////////////////////////////////////////////////

      case CtlType::ButtonStrip : {
        QWidget * w = new QWidget(_this);
        QHFlowLayout * l = new QHFlowLayout(w);

        for ( const auto & item : c.items ) {
          QToolButton * tb = new QToolButton(_this);
          tb->setToolButtonStyle(item.name.empty() ? Qt::ToolButtonIconOnly : Qt::ToolButtonTextOnly);
          tb->setText(QString::fromStdString(item.name));
          tb->setToolTip(QString::fromStdString(item.desc));
          if ( item.onclick ) {
            QObject::connect(tb, &QToolButton::clicked,
                [_this, onclick = item.onclick]() {
                  if ( onclick(_this->opts()) ) {
                    _this->updateControls();
                  }
                  Q_EMIT _this->parameterChanged();
                });
          }
          l->addWidget(tb);
        }

        currentSettings->addRow(w);
        break;
      }
      ////////////////////////////////////////////////////////////////////////
    }
  }

}

#endif /* __QSettingsWidgetTemplate2_h__ */
