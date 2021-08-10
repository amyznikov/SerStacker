/*
 * QImageProcessorRoutineSettings.cc
 *
 *  Created on: Aug 5, 2021
 *      Author: amyznikov
 */

#include "QImageProcessorRoutineSettings.h"
#include "QUnsharpMaskSettings.h"
#include "QSmapSettings.h"
#include "QNoiseMapSettings.h"
#include "QMtfSettings.h"
#include "QAutoClipSettings.h"
#include "QAnscombeSettings.h"
#include "QAlignColorChannelsSettings.h"
#include "QRangeClipSettings.h"
#include "QHistogramWhiteBalanceSettings.h"
#include "QRangeNormalizeSettings.h"
#include "QGradientSettings.h"


#define ICON_double_arrow_down    "double-arrow-down"
#define ICON_double_arrow_right   "double-arrow-right"
#define ICON_move_down            "move-down"
#define ICON_move_up              "move-up"
#define ICON_delete               "delete"
#define ICON_add                  "add"
#define ICON_menu                 "menu"


static QIcon getIcon(const QString & name)
{
  return QIcon(QString(":/qimproc/icons/%1").arg(name));
}

static std::vector<const QImageProcessorRoutineSettingsBase::ClassFactory*>
  QImageProcessorRoutineSettingsClasses_;

static QAction * add_routine_action;
static QAction * remove_routine_action;
static QAction * move_up_action;
static QAction * move_down_action;


void QImageProcessorRoutineSettingsBase::registrerClassFactory(const ClassFactory * classFactory)
{
  ClassFactoryGuardLock lock;

  std::vector<const QImageProcessorRoutineSettingsBase::ClassFactory*>::iterator ii =
      std::find(QImageProcessorRoutineSettingsClasses_.begin(),
          QImageProcessorRoutineSettingsClasses_.end(),
          classFactory);

  if ( ii == QImageProcessorRoutineSettingsClasses_.end() ) {
    QImageProcessorRoutineSettingsClasses_.emplace_back(classFactory);
  }

}

void QImageProcessorRoutineSettingsBase::registrerAllClasses()
{
  static bool registered = false;

  if ( !registered ) {

    registered = true;

    registrerClassFactory(&QUnsharpMaskSettings::classFactory);
    registrerClassFactory(&QSmapSettings::classFactory);
    registrerClassFactory(&QNoiseMapSettings::classFactory);
    registrerClassFactory(&QMtfSettings::classFactory);
    registrerClassFactory(&QAutoClipSettings::classFactory);
    registrerClassFactory(&QAnscombeSettings::classFactory);
    registrerClassFactory(&QAlignColorChannelsSettings::classFactory);
    registrerClassFactory(&QRangeClipSettings::classFactory);
    registrerClassFactory(&QHistogramWhiteBalanceSettings::classFactory);
    registrerClassFactory(&QRangeNormalizeSettings::classFactory);
    registrerClassFactory(&QGradientSettings::classFactory);


  }
}

void QImageProcessorRoutineSettingsBase::focusInEvent(QFocusEvent *event)
{
  Base::focusInEvent(event);
  CF_DEBUG(" focusWidget=%p this=%p", focusWidget(), this );

}


QImageProcessorRoutineSettingsBase::QImageProcessorRoutineSettingsBase(const ClassFactory * factory, QWidget * parent)
  : Base("QImageProcessorRoutine", parent), class_factory_(factory)
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
          image: url(:/qimproc/icons/double-arrow-right.png);
      }
      QCheckBox::indicator:checked {
          image: url(:/qimproc/icons/double-arrow-up.png);
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
    expand_ctl->setStyleSheet(expand_box_style);
    header_layout->addWidget(expand_ctl);


    static QMenu menu;
    if ( menu.isEmpty() ) {
      menu.addAction(add_routine_action = new QAction(getIcon(ICON_add), "Add ..."));
      menu.addAction(remove_routine_action = new QAction(getIcon(ICON_delete), "Delete"));
      menu.addAction(move_up_action = new QAction(getIcon(ICON_move_up), "Move Up"));
      menu.addAction(move_down_action = new QAction(getIcon(ICON_move_down), "Move Down"));
    }

    menu_ctl = new QToolButton();
    menu_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
    menu_ctl->setIconSize(QSize(16,16));
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






    enable_ctl = new QCheckBox(factory->routineClassName(), header_ctl);
    header_layout->addWidget(enable_ctl);
    form->addRow(header_ctl);


    QFrame * frame = new QFrame(this);
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

QImageProcessorRoutineSettingsBase * QImageProcessorRoutineSettingsBase::create(const c_image_processor_routine::ptr & routine, QWidget * parent)
{
  registrerAllClasses();

  if ( !routine ) {
    return nullptr;
  }

  ClassFactoryGuardLock lock;

  for ( const ClassFactory * f : QImageProcessorRoutineSettingsClasses_ ) {
    if ( f->routine_factory == routine->classfactory() ) {
      return f->create_widget_instance(routine, parent);
    }
  }

  return nullptr;
}
