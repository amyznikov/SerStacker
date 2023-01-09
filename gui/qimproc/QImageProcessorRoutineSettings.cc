/*
 * QImageProcessorRoutineSettings.cc
 *
 *  Created on: Aug 5, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorRoutineSettings.h"
#include "QMtfSettings.h"
#include "QRadialPolySharpSettings.h"
#include "QJovianEllipseSettings.h"
#include <gui/widgets/style.h>


#define ICON_double_arrow_down    ":/qimproc/icons/double-arrow-down"
#define ICON_double_arrow_right   ":/qimproc/icons/double-arrow-right"
#define ICON_move_down            ":/qimproc/icons/move-down2"
#define ICON_move_up              ":/qimproc/icons/move-up2"
#define ICON_delete               ":/qimproc/icons/delete"
#define ICON_add                  ":/qimproc/icons/add"
#define ICON_menu                 ":/qimproc/icons/menu"

static QAction *add_routine_action;
static QAction *remove_routine_action;
static QAction *move_up_action;
static QAction *move_down_action;

//static QIcon getIcon(const QString & name)
//{
//  return QIcon(QString(":/qimproc/icons/%1").arg(name));
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QImageProcessorRoutineSettings::QImageProcessorRoutineSettings(const c_image_processor_routine::ptr & routine,
    QWidget * parent) :
    Base(routine->class_name().c_str(), parent),
    routine_(routine)
{
  Q_INIT_RESOURCE(qimproc_resources);

#define __EXPAND_CTL_STYLE_TEXT(x) #x
  static const char expand_box_style[] = __EXPAND_CTL_STYLE_TEXT(
      QCheckBox {
        spacing: 12px;
      }
      QCheckBox::indicator {
        width: 16px;
        height: 16px;
      }
      QCheckBox::indicator:unchecked {
        image: url(:/qimproc/icons/${style}/double-arrow-right.png);
      }
      QCheckBox::indicator:checked {
        image: url(:/qimproc/icons/${style}/double-arrow-up.png);
      }
      );
#undef __EXPAND_CTL_STYLE_TEXT

  static const char borderless_style[] = ""
      "QToolButton { border: none; }";

  form->setSpacing(0);
  form->setVerticalSpacing(0);
  form->setMargin(0);

  header_ctl = new QWidget(this);
  header_layout = new QHBoxLayout(header_ctl);
  header_layout->setSpacing(6);
  header_layout->setMargin(0);
  header_layout->setAlignment(Qt::AlignLeft);

  expand_ctl = new QCheckBox(header_ctl);
  expand_ctl->setStyleSheet(  QString(expand_box_style).replace("${style}", iconStyleSelector()));
  header_layout->addWidget(expand_ctl);

  static QMenu menu;
  if( menu.isEmpty() ) {
    menu.addAction(add_routine_action = new QAction(getIcon(ICON_add), "Add ..."));
    menu.addAction(remove_routine_action = new QAction(getIcon(ICON_delete), "Delete"));
    menu.addAction(move_up_action = new QAction(getIcon(ICON_move_up), "Move Up"));
    menu.addAction(move_down_action = new QAction(getIcon(ICON_move_down), "Move Down"));
  }

  menu_ctl = new QToolButton();
  menu_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  menu_ctl->setIconSize(QSize(16, 16));
  menu_ctl->setIcon(getIcon(ICON_menu));
  menu_ctl->setStyleSheet(borderless_style);
  header_layout->addWidget(menu_ctl);

  connect(menu_ctl, &QToolButton::clicked,
      [this]() {

        QAction * selectedAction =
            menu.exec(menu_ctl->mapToGlobal(QPoint(menu_ctl->width()-2,menu_ctl->height()-4)));

        if ( selectedAction ) {
          if ( selectedAction == move_up_action ) {
            emit moveUpRequested(this);
          }
          else if ( selectedAction == move_down_action ) {
            emit moveDownRequested(this);
          }
          else if ( selectedAction == add_routine_action ) {
            emit addRoutineRequested(this);
          }
          else if ( selectedAction == remove_routine_action ) {
            emit removeRoutineRequested(this);
          }
        }
      });

  enable_ctl = new QCheckBox(routine->class_name().c_str(), header_ctl);
  header_layout->addWidget(enable_ctl);
  connect(enable_ctl, &QCheckBox::stateChanged,
      [this](int state) {
        routine_->set_enabled(state == Qt::Checked);
        if ( routine_->enabled() ) {
          populatecontrols();
        }
        emit parameterChanged();
  });

  ///////////////
  move_up_ctl = new QToolButton(this);
  move_up_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  move_up_ctl->setStyleSheet(borderless_style);
  move_up_ctl->setIconSize(QSize(16, 16));
  move_up_ctl->setIcon(getIcon(ICON_move_up));
  header_layout->addWidget(move_up_ctl, 10, Qt::AlignRight);
  connect(move_up_ctl, &QToolButton::clicked,
      [this]() {
        emit moveUpRequested(this);
        //QCursor::setPos(move_up_ctl->mapToGlobal(QPoint(move_up_ctl->width()/2, move_up_ctl->height()/2)));
    });

  ///////////////
  move_down_ctl = new QToolButton(this);
  move_down_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  move_down_ctl->setStyleSheet(borderless_style);
  move_down_ctl->setIconSize(QSize(16, 16));
  move_down_ctl->setIcon(getIcon(ICON_move_down));
  header_layout->addWidget(move_down_ctl, 0, Qt::AlignRight);
  connect(move_down_ctl, &QToolButton::clicked,
      [this]() {
        emit moveDownRequested(this);
        //QCursor::setPos(move_down_ctl->mapToGlobal(QPoint(move_down_ctl->width()/2, move_down_ctl->height()/2)));
    });

  ///////////////

  form->addRow(header_ctl);

  QFrame *frame = new QFrame(this);
  frame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  ctlform = new QFormLayout(routine_ctl = frame/*new QWidget(this)*/);
  //ctlform->setMargin(4);
  //ctlform->setContentsMargins(8, 8, 0, 0);
  form->addRow(routine_ctl);

  connect(expand_ctl, &QCheckBox::stateChanged,
      [this](int state) {
        routine_ctl->setVisible(state == Qt::Checked);
      });

  routine_ctl->setVisible(expand_ctl->isChecked());
}

const c_image_processor_routine::ptr& QImageProcessorRoutineSettings::routine() const
{
  return routine_;
}

void QImageProcessorRoutineSettings::setup_controls()
{
  std::vector<struct c_image_processor_routine_ctrl> params;
  routine_->get_parameters(&params);

  for( const struct c_image_processor_routine_ctrl &p : params ) {

    switch (p.ctl_type) {
    case c_image_processor_routine_gui_ctl_numeric_text_box: {

      QNumberEditBox *ctl = new QNumberEditBox();
      ctl->setToolTip(p.tooltip.c_str());
      ctlform->addRow(p.name.c_str(), ctl);

      if( p.set_value ) {

        QMetaObject::Connection conn =
            QObject::connect(ctl, &QNumberEditBox::textChanged,
                [this, ctl, p]() {
                  if ( !updatingControls() ) {
                    p.set_value(ctl->text().toStdString());
                    emit parameterChanged();
                  }
                });

        QObject::connect(ctl, &QObject::destroyed,
            [conn]() {
              QObject::disconnect(conn);
            });
      }

      if( p.get_value ) {
        QMetaObject::Connection conn =
            QObject::connect(this, &ThisClass::populatecontrols,
                [ctl, p]() {
                  ctl->setText(p.get_value().c_str());
                });

        QObject::connect(ctl, &QObject::destroyed,
            [conn]() {
              QObject::disconnect(conn);
            });
      }

      break;
    }
    case c_image_processor_routine_gui_ctl_enum_combobox: {

      QEnumComboBoxBase *ctl = new QEnumComboBoxBase();
      ctl->setToolTip(p.tooltip.c_str());
      ctlform->addRow(p.name.c_str(), ctl);

      if( p.get_enum_members ) {
        ctl->setupItems(p.get_enum_members());
      }

      if( p.set_value ) {

        QMetaObject::Connection conn =
            QObject::connect(ctl, &QEnumComboBoxBase::currentItemChanged,
                [this, ctl, p]() {
                  if ( !updatingControls() ) {
                    p.set_value(ctl->currentText().toStdString());
                    emit parameterChanged();
                  }
                });

        QObject::connect(ctl, &QObject::destroyed,
            [conn]() {
              QObject::disconnect(conn);
            });
      }

      if( p.get_value ) {

        QMetaObject::Connection conn =
            QObject::connect(this, &ThisClass::populatecontrols,
                [ctl, p]() {
                  ctl->setCurrentText(p.get_value().c_str());
                });

        QObject::connect(ctl, &QObject::destroyed,
            [conn]() {
              QObject::disconnect(conn);
            });
      }

      break;
    }
    case c_image_processor_routine_gui_ctl_check_box: {

      QCheckBox *ctl = new QCheckBox();
      ctl->setToolTip(p.tooltip.c_str());
      ctlform->addRow(p.name.c_str(), ctl);

      if( p.set_value ) {

        QMetaObject::Connection conn =
            QObject::connect(ctl, &QCheckBox::stateChanged,
                [this, ctl, p](int state) {
                  if ( !updatingControls() ) {
                    p.set_value(state == Qt::Checked ? "1" : "0");
                    emit parameterChanged();
                  }
                });

        QObject::connect(ctl, &QObject::destroyed,
            [conn]() {
              QObject::disconnect(conn);
            });
      }

      if( p.get_value ) {
        QMetaObject::Connection conn =
            QObject::connect(this, &ThisClass::populatecontrols,
                [ctl, p]() {
                  bool checked = false;
                  if ( fromString(p.get_value(), &checked) ) {
                    ctl->setChecked(checked);
                  }
                });

        QObject::connect(ctl, &QObject::destroyed,
            [conn]() {
              QObject::disconnect(conn);
            });
      }
      break;
    }
    default:
      break;
    }
  }

  updateControls();
}

void QImageProcessorRoutineSettings::onupdatecontrols()
{
  if( routine_ ) {
    enable_ctl->setChecked(routine_->enabled());
    emit populatecontrols();
  }
}

//void QImageProcessorRoutineSettings::focusInEvent(QFocusEvent * event)
//{
//  Base::focusInEvent(event);
//}

QImageProcessorRoutineSettings* QImageProcessorRoutineSettings::create(const c_image_processor_routine::ptr & routine,
    QWidget * parent)
{
  if( !routine ) {
    CF_ERROR("No image processing routine specifies");
    return nullptr;
  }

  QImageProcessorRoutineSettings * settingsWidget;

  if( routine->classfactory() == &c_mtf_routine::class_factory ) {
    settingsWidget = new QMtfSettings(std::dynamic_pointer_cast<c_mtf_routine>(routine), parent);
  }
  else if( routine->classfactory() == &c_radial_polysharp_routine::class_factory ) {
    settingsWidget = new QRadialPolySharpSettings(std::dynamic_pointer_cast<c_radial_polysharp_routine>(routine), parent);
  }
  else if( routine->classfactory() == &c_fit_jovian_ellipse_routine::class_factory ) {
    settingsWidget = new QJovianEllipseSettings(std::dynamic_pointer_cast<c_fit_jovian_ellipse_routine>(routine), parent);
  }
  else {
    settingsWidget = new QImageProcessorRoutineSettings(routine, parent);
  }

  settingsWidget->setup_controls();

  return settingsWidget;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
