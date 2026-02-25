/*
 * QIndigoFocuserSettings.cc
 *
 *  Created on: Nov 8, 2022
 *      Author: amyznikov
 */

#include "QIndigoFocuserWidget.h"
#include <core/ssprintf.h>
#include <core/get_time.h>
#include <core/debug.h>

#ifdef INDIGO_ERROR
# undef INDIGO_ERROR
# define INDIGO_ERROR(...) \
    (indigo_get_log_level() >= INDIGO_LOG_ERROR ? indigo_error(__VA_ARGS__) : (void)(0))
#endif // INDIGO_ERROR

#ifdef INDIGO_DEBUG
# undef INDIGO_DEBUG
# define INDIGO_DEBUG(...) \
    (indigo_get_log_level() >= INDIGO_LOG_DEBUG ? indigo_debug(__VA_ARGS__) : (void)(0))
#endif // INDIGO_DEBUG

#ifdef INDIGO_TRACE
# undef INDIGO_TRACE
# define INDIGO_TRACE(...) \
    (indigo_get_log_level() >= INDIGO_LOG_TRACE ? indigo_trace(__VA_ARGS__) : (void)(0))
#endif // INDIGO_TRACE


#define ICON_goto_position      ":/qindigo/icons/play_icon.png"
#define ICON_stop               ":/qindigo/icons/stop_icon.png"
#define ICON_left_arrow         ":/qindigo/icons/left_arrow_icon.png"
#define ICON_right_arrow        ":/qindigo/icons/right_arrow_icon.png"
#define ICON_double_left_arrow  ":/qindigo/icons/double_left_arrow_icon.png"
#define ICON_double_right_arrow ":/qindigo/icons/double_right_arrow_icon.png"
#define ICON_down_arrow         ":/qindigo/icons/down_b_arrow_icon.png"
#define ICON_up_arrow           ":/qindigo/icons/up_b_arrow_icon.png"
#define ICON_apply              ":/qindigo/icons/apply_icon.png"
#define ICON_center             ":/qindigo/icons/center.png"
#define ICON_mc                 ":/qindigo/icons/mc-light.png"

static QIcon icon_apply;
static QIcon icon_left_arrow;
static QIcon icon_right_arrow;
static QIcon icon_double_left_arrow;
static QIcon icon_double_right_arrow;
static QIcon icon_down_arrow;
static QIcon icon_up_arrow;
static QIcon icon_goto_position;
static QIcon icon_stop;
static QIcon icon_center;
//static QIcon icon_mc;

//static QIcon getIcon(const QString & name)
//{
//  return QIcon(QString(":/qindigo/icons/%1").arg(name));
//}

static void init_resources()
{
  Q_INIT_RESOURCE(qindigo_resources);

  if( icon_apply.isNull() ) {
    icon_apply = getIcon(ICON_apply);
  }
  if( icon_left_arrow.isNull() ) {
    icon_left_arrow = getIcon(ICON_left_arrow);
  }
  if( icon_right_arrow.isNull() ) {
    icon_right_arrow = getIcon(ICON_right_arrow);
  }
  if( icon_double_left_arrow.isNull() ) {
    icon_double_left_arrow = getIcon(ICON_double_left_arrow);
  }
  if( icon_double_right_arrow.isNull() ) {
    icon_double_right_arrow = getIcon(ICON_double_right_arrow);
  }
  if( icon_goto_position.isNull() ) {
    icon_goto_position = getIcon(ICON_goto_position);
  }
  if( icon_stop.isNull() ) {
    icon_stop = getIcon(ICON_stop);
  }
  if( icon_down_arrow.isNull() ) {
    icon_down_arrow = getIcon(ICON_down_arrow);
  }
  if( icon_up_arrow.isNull() ) {
    icon_up_arrow = getIcon(ICON_up_arrow);
  }
  if( icon_center.isNull() ) {
    icon_center = getIcon(ICON_center);
  }

//  if( icon_mc.isNull() ) {
//    icon_mc = getIcon(ICON_mc);
//  }

}


static void enableControl(QWidget * w, bool enabled )
{
  w->setEnabled(enabled);
}

static void enableControl(QAbstractButton * w, bool enabled, const QIcon & icon)
{
  w->setEnabled(enabled);
  if ( w->icon().cacheKey() != icon.cacheKey() ){
    w->setIcon(icon);
  }
}


static QWidget * createFocuserSelectorControl(QWidget * parent,
    QComboBox * focuserSelect_ctl,
    QToolButton * connectDiconnect_ctl)
{
  QWidget * w = new QWidget(parent);
  QHBoxLayout * hbox = new QHBoxLayout(w);

  w->setContentsMargins(0,0,0,0);
  hbox->setContentsMargins(0, 0, 0, 0);

  hbox->addWidget(focuserSelect_ctl, 100);
  hbox->addWidget(connectDiconnect_ctl, 0, Qt::AlignRight);

  return w;
}

static QWidget * createFocusLimitsControl(QWidget * parent,
    QLineEditBox * focusLimits_ctl/*,
    QToolButton * applyFocusLimits_ctl*/)
{
  QWidget * w = new QWidget(parent);
  QHBoxLayout * hbox = new QHBoxLayout(w);

  w->setContentsMargins(0,0,0,0);
  hbox->setContentsMargins(0, 0, 0, 0);

  focusLimits_ctl->setText("-1000000:1000000");
  focusLimits_ctl->setMaximumWidth(4000);
  focusLimits_ctl->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
//  applyFocusLimits_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
//  applyFocusLimits_ctl->setIcon(icon_apply);
//  applyFocusLimits_ctl->setText("Apply");
//  applyFocusLimits_ctl->setToolTip("Apply focus limits");



  hbox->addWidget(focusLimits_ctl, 1000, Qt::AlignRight);
//  hbox->addWidget(applyFocusLimits_ctl, 1, Qt::AlignRight);

  return w;

}


static QWidget * createFocuserPositionControl(QWidget * parent,
    QSpinBox * focusAbsolutePosition_ctl,
    QToolButton * moveFocusToAbsolutePosition_ctl,
    QToolButton * moveFocusToMinAbsolutePosition_ctl,
    QToolButton * moveFocusToMaxAbsolutePosition_ctl,
    QToolButton * moveFocusToMiddlePosition_ctl)
{
  QWidget * w = new QWidget(parent);
  QHBoxLayout * hbox = new QHBoxLayout(w);

  w->setContentsMargins(0,0,0,0);
  hbox->setContentsMargins(0, 0, 0, 0);

  focusAbsolutePosition_ctl->setKeyboardTracking(false);
  focusAbsolutePosition_ctl->setRange(-1000000, 1000000);
  focusAbsolutePosition_ctl->setValue(0);


  moveFocusToMiddlePosition_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  moveFocusToMiddlePosition_ctl->setIcon(icon_center);
  moveFocusToMiddlePosition_ctl->setText("Middle");
  moveFocusToMiddlePosition_ctl->setToolTip("Move focus to middle position");

  moveFocusToAbsolutePosition_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  moveFocusToAbsolutePosition_ctl->setIcon(icon_goto_position);
  moveFocusToAbsolutePosition_ctl->setText("Go");
  moveFocusToAbsolutePosition_ctl->setToolTip("Move focus to absolute position");

  moveFocusToMinAbsolutePosition_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  moveFocusToMinAbsolutePosition_ctl->setIcon(icon_down_arrow);
  moveFocusToMinAbsolutePosition_ctl->setText("Min");
  moveFocusToMinAbsolutePosition_ctl->setToolTip("Move focus to min absolute position");

  moveFocusToMaxAbsolutePosition_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  moveFocusToMaxAbsolutePosition_ctl->setIcon(icon_up_arrow);
  moveFocusToMaxAbsolutePosition_ctl->setText("Max");
  moveFocusToMaxAbsolutePosition_ctl->setToolTip("Move focus to max absolute position");


  hbox->addWidget(focusAbsolutePosition_ctl, 100);
  hbox->addWidget(moveFocusToMiddlePosition_ctl, 0, Qt::AlignRight);
  hbox->addWidget(moveFocusToMinAbsolutePosition_ctl, 0, Qt::AlignRight);
  hbox->addWidget(moveFocusToMaxAbsolutePosition_ctl, 0, Qt::AlignRight);
  hbox->addWidget(moveFocusToAbsolutePosition_ctl, 0, Qt::AlignRight);

  return w;
}

static QWidget * createRelativeMoveControl(QWidget * parent,
    QSpinBox * focusRelativeMoveSteps_ctl,
    QToolButton * focusStepsInward_ctl,
    QToolButton * focusStepsOutward_ctl,
    QToolButton * focusStepsInward4x_ctl,
    QToolButton * focusStepsOutward4x_ctl)
{
  QWidget * w = new QWidget(parent);
  QHBoxLayout * hbox = new QHBoxLayout(w);

  w->setContentsMargins(0,0,0,0);
  hbox->setContentsMargins(0, 0, 0, 0);

  focusRelativeMoveSteps_ctl->setKeyboardTracking(false);
  focusRelativeMoveSteps_ctl->setRange(0, 100000);
  focusRelativeMoveSteps_ctl->setValue(1);

  focusStepsInward4x_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  focusStepsInward4x_ctl->setIcon(getIcon(ICON_double_left_arrow));
  focusStepsInward4x_ctl->setText("<<");
  focusStepsInward4x_ctl->setToolTip("Move 4x steps inward / abort motion");

  focusStepsInward_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  focusStepsInward_ctl->setIcon(getIcon(ICON_left_arrow));
  focusStepsInward_ctl->setText("<");
  focusStepsInward_ctl->setToolTip("Move steps inward / abort motion");

  focusStepsOutward_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  focusStepsOutward_ctl->setIcon(getIcon(ICON_right_arrow));
  focusStepsOutward_ctl->setText(">");
  focusStepsOutward_ctl->setToolTip("Move steps outward / abort motion");


  focusStepsOutward4x_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  focusStepsOutward4x_ctl->setIcon(getIcon(ICON_double_right_arrow));
  focusStepsOutward4x_ctl->setText(">>");
  focusStepsOutward4x_ctl->setToolTip("Move 4x steps outward / abort motion");

  hbox->addWidget(focusRelativeMoveSteps_ctl, 100);
  hbox->addWidget(focusStepsInward4x_ctl, 0, Qt::AlignRight);
  hbox->addWidget(focusStepsInward_ctl, 0, Qt::AlignRight);
  hbox->addWidget(focusStepsOutward_ctl, 0, Qt::AlignRight);
  hbox->addWidget(focusStepsOutward4x_ctl, 0, Qt::AlignRight);

  return w;
}


static QWidget * createFocuserStateStatusLabel(QWidget * parent,
    QLabel * status_ctl,
    QLabel * temperature_ctl)
{
  QWidget * w = new QWidget(parent);
  QHBoxLayout * hbox = new QHBoxLayout(w);

  w->setContentsMargins(0,0,0,0);
  hbox->setContentsMargins(0, 0, 0, 0);

  hbox->addWidget(status_ctl, 100);
  hbox->addWidget(temperature_ctl, 0, Qt::AlignRight);

  return w;
}


QIndigoFocuserWidget::QIndigoFocuserWidget(QWidget * parent) :
  Base(parent) // "QIndigoFocuserSettings"
{
  init_resources();

  form->addRow("Focuser:",
      createFocuserSelectorControl(this,
          deviceSelector_ctl = new QComboBox(this),
          connectDisconnect_ctl = new QToolButton(this)));

  deviceSelector_ctl->setEditable(false);
  deviceSelector_ctl->addItem("FocusDreamPro");
  connectDisconnect_ctl->setText("connect");


  form->addRow("Focuser limits:",
      createFocusLimitsControl(this,
          focuserLimits_ctl = new QLineEditBox(this)/*,
          focuserLimitsApply_ctl = new QToolButton(this)*/));

  form->addRow("Focuser speed:",
      focuserSpeed_ctl = new QSpinBox(this));
  focuserSpeed_ctl->setKeyboardTracking(false);

  form->addRow("Move to position:",
      createFocuserPositionControl(this,
          focuserPosition_ctl = new QSpinBox(this),
          focuserPositionMoveToAbsolutePosition_ctl = new QToolButton(this),
          focuserPositionMoveToMinPosition_ctl = new QToolButton(this),
          focuserPositionMoveToMaxPosition_ctl = new QToolButton(this),
          focuserPositionMoveToMiddlePosition_ctl = new QToolButton(this)));

  form->addRow("Relative steps:",
      createRelativeMoveControl( this,
          focuserSteps_ctl = new QSpinBox(this),
          focuserStepsMoveInward_ctl = new QToolButton(this),
          focuserStepsMoveOutward_ctl = new QToolButton(this),
          focuserStepsMoveInward4x_ctl = new QToolButton(this),
          focuserStepsMoveOutward4x_ctl = new QToolButton(this)
          ));

  form->addRow("Enable mouse control:",
      enableMouse_ctl = new QCheckBox(this));

//#if QT_CONFIG(wheelevent)
//#endif

  form->addRow(mouseclick_ctl = new QIndigoFocuserMouseClickControl(this));
  //mouseclick_ctl->setEnabled(enableMouse_ctl->isChecked());


  form->addRow(createFocuserStateStatusLabel(this,
          status_ctl = new QLabel("Status", this),
          temperature_ctl = new QLabel("Temperature (°C)", this)));

  status_ctl->setAlignment(Qt::AlignRight);
  status_ctl->setTextInteractionFlags(Qt::TextSelectableByMouse |
      Qt::TextSelectableByKeyboard);


  ///////////////////////////////////////////////

  if( !logWidget_ctl ) {

    add_expandable_groupbox("Indigo log",
        logWidget_ctl = new QPlainTextEdit(this));

    logWidget_ctl->setReadOnly(true);
    logWidget_ctl->setWordWrapMode(QTextOption::NoWrap);
    logWidget_ctl->setMaximumBlockCount(1000);
    logWidget_ctl->document()->setDefaultFont(QFont("Monospace", 9));

    indigo_log_message_handler =
        &ThisClass::on_indigo_log_message;
  }
  ///////////////////////////////////////////////

  connect(this, &ThisClass::focuserConnectionStateChanged,
      this, &ThisClass::onFocuserConnectionStateChanged,
      Qt::AutoConnection);

  connect(this, &ThisClass::focuserMoveDirectionChanged,
      this, &ThisClass::onFocuserMoveDirectionChanged,
      Qt::AutoConnection);

  connect(this, &ThisClass::focuserPositionChanged,
      this, &ThisClass::onFocuserPositionChanged,
      Qt::AutoConnection);

  connect(this, &ThisClass::focuserLimitsChanged,
      this, &ThisClass::onFocuserLimitsChanged,
      Qt::AutoConnection);

  connect(this, &ThisClass::focuserStepsChanged,
      this, &ThisClass::onFocuserStepsChanged,
      Qt::AutoConnection);

  connect(this, &ThisClass::focuserSpeedChanged,
      this, &ThisClass::onFocuserSpeedChanged,
      Qt::AutoConnection);

  connect(this, &ThisClass::focuserTemperatureChanged,
      this, &ThisClass::onFocuserTemperatureChanged,
      Qt::AutoConnection);

  connect(connectDisconnect_ctl, &QToolButton::clicked,
      this, &ThisClass::onConnectDiconnectButtonClicked);

  connect(deviceSelector_ctl, SIGNAL(currentIndexChanged(int)),
      this, SLOT(onDeviceSelectorCurrentIndexChanged(int)));

  connect(focuserLimits_ctl, &QLineEditBox::textChanged,
      this, &ThisClass::onApplyFocusLimitsClicked);

  connect(focuserPositionMoveToAbsolutePosition_ctl, &QToolButton::clicked,
      this, &ThisClass::onFocuserPositionMoveToAbsolutePositionClicked);

  connect(focuserPositionMoveToMinPosition_ctl, &QToolButton::clicked,
      this, &ThisClass::onFocuserPositionMoveToMinPositionClicked);

  connect(focuserPositionMoveToMaxPosition_ctl, &QToolButton::clicked,
      this, &ThisClass::onFocuserPositionMoveToMaxPositionClicked);

  connect(focuserPositionMoveToMiddlePosition_ctl, &QToolButton::clicked,
      this, &ThisClass::onFocuserPositionMoveToMiddlePositionClicked);


  connect(focuserSpeed_ctl, SIGNAL(valueChanged(int)),
      this, SLOT(onFocuserSpeedControlValueChanged(int)));

  connect(focuserStepsMoveInward_ctl, &QToolButton::clicked,
      this, &ThisClass::onFocuserStepsMoveInwardClicked);

  connect(focuserStepsMoveOutward_ctl, &QToolButton::clicked,
      this, &ThisClass::onFocuserStepsMoveOutwardClicked);

  connect(focuserStepsMoveInward4x_ctl, &QToolButton::clicked,
      this, &ThisClass::onFocuserStepsMoveInward4xClicked);

  connect(focuserStepsMoveOutward4x_ctl, &QToolButton::clicked,
      this, &ThisClass::onFocuserStepsMoveOutward4xClicked);

  connect(enableMouse_ctl, &QCheckBox::stateChanged,
      [this](int state) {
        mouseclick_ctl->setEnabled(state==Qt::Checked);
      });

  connect(this, &ThisClass::enablecontrols,
      [this]() {
        if ( !client_ || currentDeviceName_.isEmpty() ) {
          setEnabled(false);
        }
        else {

          if( !isConnected() ) {
            enableControl(focuserLimits_ctl, false);
            //enableControl(focuserLimitsApply_ctl, false);
            enableControl(focuserPosition_ctl, false);
            enableControl(focuserPositionMoveToAbsolutePosition_ctl, false);
            enableControl(focuserPositionMoveToMinPosition_ctl, false);
            enableControl(focuserPositionMoveToMaxPosition_ctl, false);
            enableControl(focuserSteps_ctl, false);
            enableControl(focuserStepsMoveInward_ctl, false);
            enableControl(focuserStepsMoveOutward_ctl, false);
            enableControl(focuserStepsMoveInward4x_ctl, false);
            enableControl(focuserStepsMoveOutward4x_ctl, false);
            enableControl(focuserSpeed_ctl, false);
            enableControl(enableMouse_ctl, false);
            enableControl(mouseclick_ctl, false);
          }
          else if ( isMovingFocusNow() ) {

            enableControl(focuserLimits_ctl, false);
            //enableControl(focuserLimitsApply_ctl, false);
            enableControl(focuserPosition_ctl, false);

            enableControl(focuserPositionMoveToAbsolutePosition_ctl, true, icon_stop);
            enableControl(focuserPositionMoveToMinPosition_ctl, true, icon_stop);
            enableControl(focuserPositionMoveToMaxPosition_ctl, true, icon_stop);

            enableControl(focuserSteps_ctl, false);
            enableControl(focuserStepsMoveInward_ctl, true, icon_stop);
            enableControl(focuserStepsMoveOutward_ctl, true, icon_stop);
            enableControl(focuserStepsMoveInward4x_ctl, true, icon_stop);
            enableControl(focuserStepsMoveOutward4x_ctl, true, icon_stop);
            enableControl(focuserSpeed_ctl, true);
            enableControl(enableMouse_ctl, false);
          }
          else {

            enableControl(focuserLimits_ctl, true);
            //enableControl(focuserLimitsApply_ctl, true);

            enableControl(focuserPosition_ctl, true);
            enableControl(focuserPositionMoveToAbsolutePosition_ctl, true, icon_goto_position);
            enableControl(focuserPositionMoveToMinPosition_ctl, true, icon_down_arrow);
            enableControl(focuserPositionMoveToMaxPosition_ctl, true, icon_up_arrow);

            enableControl(focuserSteps_ctl, true);
            enableControl(focuserStepsMoveInward_ctl, true, icon_left_arrow);
            enableControl(focuserStepsMoveOutward_ctl, true, icon_right_arrow);
            enableControl(focuserStepsMoveInward4x_ctl, true, icon_double_left_arrow);
            enableControl(focuserStepsMoveOutward4x_ctl, true, icon_double_right_arrow);
            enableControl(focuserSpeed_ctl, true);
            enableControl(enableMouse_ctl, true);
            enableControl(mouseclick_ctl, enableMouse_ctl->isChecked());
          }

          setEnabled(true);
        }
      });

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( client_ && !currentDeviceName_.isEmpty() ) {

          QString statusText;

          if ( focuser_position.defined ) {
            statusText +=
            QString("%1").arg(focuser_position.value);
          }

          if( focuser_limits.defined ) {
            statusText +=
            QString(" [%1:%2]")
            .arg(focuser_limits.min_value)
            .arg(focuser_limits.max_value);
          }

          status_ctl->setText(statusText);
        }
      });


  onDeviceSelectorCurrentIndexChanged(deviceSelector_ctl->currentIndex());
  updateControls();
}

QIndigoFocuserWidget::~QIndigoFocuserWidget()
{
}

void QIndigoFocuserWidget::setIndigoClient(QIndigoClient * client)
{
  if( (client_ = client) ) {

    connect(client_, &QIndigoClient::clientAttach,
        this, &ThisClass::onIndigoClientAttach);

    connect(client_, &QIndigoClient::clientDefineProperty,
        this, &ThisClass::onIndigoClientDefineProperty);

    connect(client_, &QIndigoClient::clientUpdateProperty,
        this, &ThisClass::onIndigoClientUpdateProperty);

    connect(client_, &QIndigoClient::clientDeleteProperty,
        this, &ThisClass::onIndigoClientDeleteProperty);

    connect(client_, &QIndigoClient::clientSendMessage,
        this, &ThisClass::onIndigoClientSendMessage);

    connect(client_, &QIndigoClient::clientDetach,
        this, &ThisClass::onIndigoClientDetach);

  }

  updateControls();
}

QIndigoClient * QIndigoFocuserWidget::indigoClient() const
{
  return client_;
}

bool QIndigoFocuserWidget::isConnected() const
{
  return focuser_connection.defined &&
      focuser_connection.connection_status == INDIGO_CONNECTION_STATUS_CONNECTED &&
      focuser_connection.state == INDIGO_OK_STATE;
}

bool QIndigoFocuserWidget::isMovingFocusNow() const
{
  return isConnected() &&
      ((focuser_position.defined && focuser_position.state == INDIGO_BUSY_STATE) ||
      (focuser_steps.defined && focuser_steps.state == INDIGO_BUSY_STATE));
}

bool QIndigoFocuserWidget::canMoveFocusNow() const
{
  return isConnected() && !isMovingFocusNow();
}

QPlainTextEdit * QIndigoFocuserWidget::logWidget_ctl = nullptr;

void QIndigoFocuserWidget::on_indigo_log_message(indigo_log_levels level, const char *message)
{
  if( ThisClass::logWidget_ctl ) {

    const QString msg =
        message;

    QMetaObject::invokeMethod(logWidget_ctl, [msg]() {
      if( ThisClass::logWidget_ctl ) {
        ThisClass::logWidget_ctl->moveCursor(QTextCursor::End);
        ThisClass::logWidget_ctl->appendPlainText(msg);
        ThisClass::logWidget_ctl->moveCursor(QTextCursor::End);
        ThisClass::logWidget_ctl->ensureCursorVisible();
      }
    });

  }
  else if( indigo_log_message_handler && indigo_log_message_handler != &ThisClass::on_indigo_log_message ) {
    indigo_log_message_handler(level, message);
  }
  else {
    fprintf(stderr, "%s\n", message);
  }

}

//
//void QIndigoFocuserWidget::onupdatecontrols()
//{
//  if ( !client_ || currentDeviceName_.isEmpty() ) {
//    setEnabled(false);
//  }
//  else {
//
//    if( !isConnected() ) {
//
//      enableControl(focuserLimits_ctl, false);
//      //enableControl(focuserLimitsApply_ctl, false);
//      enableControl(focuserPosition_ctl, false);
//      enableControl(focuserPositionMoveToAbsolutePosition_ctl, false);
//      enableControl(focuserPositionMoveToMinPosition_ctl, false);
//      enableControl(focuserPositionMoveToMaxPosition_ctl, false);
//      enableControl(focuserSteps_ctl, false);
//      enableControl(focuserStepsMoveInward_ctl, false);
//      enableControl(focuserStepsMoveOutward_ctl, false);
//      enableControl(focuserStepsMoveInward4x_ctl, false);
//      enableControl(focuserStepsMoveOutward4x_ctl, false);
//      enableControl(focuserSpeed_ctl, false);
//      enableControl(enableMouse_ctl, false);
//      enableControl(mouseclick_ctl, false);
//    }
//    else if ( isMovingFocusNow() ) {
//
//      enableControl(focuserLimits_ctl, false);
//      //enableControl(focuserLimitsApply_ctl, false);
//      enableControl(focuserPosition_ctl, false);
//
//      enableControl(focuserPositionMoveToAbsolutePosition_ctl, true, icon_stop);
//      enableControl(focuserPositionMoveToMinPosition_ctl, true, icon_stop);
//      enableControl(focuserPositionMoveToMaxPosition_ctl, true, icon_stop);
//
//      enableControl(focuserSteps_ctl, false);
//      enableControl(focuserStepsMoveInward_ctl, true, icon_stop);
//      enableControl(focuserStepsMoveOutward_ctl, true, icon_stop);
//      enableControl(focuserStepsMoveInward4x_ctl, true, icon_stop);
//      enableControl(focuserStepsMoveOutward4x_ctl, true, icon_stop);
//      enableControl(focuserSpeed_ctl, true);
//      enableControl(enableMouse_ctl, false);
//    }
//    else {
//
//      enableControl(focuserLimits_ctl, true);
//      //enableControl(focuserLimitsApply_ctl, true);
//
//      enableControl(focuserPosition_ctl, true);
//      enableControl(focuserPositionMoveToAbsolutePosition_ctl, true, icon_goto_position);
//      enableControl(focuserPositionMoveToMinPosition_ctl, true, icon_down_arrow);
//      enableControl(focuserPositionMoveToMaxPosition_ctl, true, icon_up_arrow);
//
//      enableControl(focuserSteps_ctl, true);
//      enableControl(focuserStepsMoveInward_ctl, true, icon_left_arrow);
//      enableControl(focuserStepsMoveOutward_ctl, true, icon_right_arrow);
//      enableControl(focuserStepsMoveInward4x_ctl, true, icon_double_left_arrow);
//      enableControl(focuserStepsMoveOutward4x_ctl, true, icon_double_right_arrow);
//      enableControl(focuserSpeed_ctl, true);
//      enableControl(enableMouse_ctl, true);
//      enableControl(mouseclick_ctl, enableMouse_ctl->isChecked());
//    }
//
//
//    QString statusText;
//
//    if ( focuser_position.defined ) {
//      statusText +=
//          QString("%1").arg(focuser_position.value);
//    }
//
//    if( focuser_limits.defined ) {
//      statusText +=
//          QString(" [%1:%2]")
//              .arg(focuser_limits.min_value)
//              .arg(focuser_limits.max_value);
//    }
//
//    status_ctl->setText(statusText);
//    setEnabled(true);
//  }
//}
//

void QIndigoFocuserWidget::onFocuserConnectionStateChanged()
{
  if( focuser_connection.defined ) {

    if( focuser_connection.connection_status != INDIGO_CONNECTION_STATUS_CONNECTED ) {
      connectDisconnect_ctl->setText("Connect...");
    }
    else {
      connectDisconnect_ctl->setText("Disconnect...");
    }
  }

  updateControls();
}

void QIndigoFocuserWidget::onFocuserMoveDirectionChanged()
{
}

void QIndigoFocuserWidget::onFocuserPositionChanged()
{
  if ( focuser_position.defined ) {

    const int minvalue =
        focuser_limits.defined ?
            (std::max)(focuser_limits.min_value, focuser_position.min_value) :
            focuser_position.min_value;

    const int maxvalue =
        focuser_limits.defined ?
            (std::min)(focuser_limits.max_value, focuser_position.max_value) :
            focuser_position.max_value;

    if( focuserPosition_ctl->minimum() != minvalue || focuserPosition_ctl->maximum() != maxvalue ) {
      focuserPosition_ctl->setRange(minvalue, maxvalue);
    }
  }

  updateControls();
}

void QIndigoFocuserWidget::onFocuserLimitsChanged()
{
  if ( focuser_limits.defined ) {
    focuserLimits_ctl->setText(QString("%1:%2")
        .arg(focuser_limits.min_value)
        .arg(focuser_limits.max_value));
  }

  updateControls();
}

void QIndigoFocuserWidget::onFocuserStepsChanged()
{
  if( focuser_steps.defined ) {
    if( focuserSteps_ctl->minimum() != focuser_steps.min_value ||
        focuserSteps_ctl->maximum() != focuser_steps.max_value ) {
      focuserSteps_ctl->setRange(focuser_steps.min_value, focuser_steps.max_value);
    }
  }

  updateControls();
}

void QIndigoFocuserWidget::onFocuserSpeedChanged()
{
  if( focuser_speed.defined ) {
    if( focuserSpeed_ctl->minimum() != focuser_speed.min_value
        || focuserSpeed_ctl->maximum() != focuser_speed.max_value ) {
      focuserSpeed_ctl->setRange(focuser_speed.min_value, focuser_speed.max_value);
    }
    if( focuserSpeed_ctl->value() != focuser_speed.value ) {
      focuserSpeed_ctl->setValue(focuser_speed.value);
    }
  }

}

void QIndigoFocuserWidget::onFocuserTemperatureChanged()
{
  if( focuser_temperature.defined ) {
    temperature_ctl->setText(QString(" %1 °C")
        .arg(focuser_temperature.value));
  }
}


void QIndigoFocuserWidget::enumerateCurrentDeviceProperties()
{
  if( client_ && !currentDeviceName_.isEmpty() ) {

    indigo_property device_properties = { 0 };

    strncpy(device_properties.device,
        currentDeviceName_.toUtf8().constData(),
        INDIGO_NAME_SIZE);

    indigo_result status =
        client_->enumerate_properties(
            &device_properties);

    if( status != INDIGO_OK ) {
      INDIGO_ERROR("%s(): client_->enumerateProperties('%s') fails: "
          "status=%d\n",
          __func__,
          device_properties.
          device,
          status);
    }

  }
}


void QIndigoFocuserWidget::updateCurrentDeviceProperties(const indigo_property * property, const QString & message)
{
  if( match_property_name(property, CONNECTION_PROPERTY_NAME) ) {

    INDIGO_CONNECTION_STATUS new_connection_status;

    if( get_connection_status(property, &new_connection_status) ) {

      bool has_changes = false;

      if( true ) {
        c_guard_lock lock(mtx_);

        has_changes =
            focuser_connection.connection_status != new_connection_status ||
            focuser_connection.state != property->state ||
            !focuser_connection.defined;

        if ( has_changes ) {
          focuser_connection.connection_status = new_connection_status;
          focuser_connection.state = property->state;
          focuser_connection.defined = true;
        }

      }

      if ( has_changes ) {
        emit focuserConnectionStateChanged();
      }
    }
  }
  else if( match_property_name(property, FOCUSER_DIRECTION_PROPERTY_NAME) ) {

    INDIGO_FOCUSER_MOVE_DIRECTION direction;

    if ( get_focuser_move_direction(property, &direction)) {

      if( true ) {
        c_guard_lock lock(mtx_);

        focuser_direction.value = direction;
        focuser_direction.state = property->state;
        focuser_direction.defined = true;
      }

      emit focuserMoveDirectionChanged();
    }

  }
  else if ( match_property_name(property, FOCUSER_POSITION_PROPERTY_NAME) ) {

    int current_value;
    int min_value;
    int max_value;

    if( get_focuser_position(property, &current_value, &min_value, &max_value) ) {

      if ( true ) {
        c_guard_lock lock(mtx_);

        focuser_position.value = current_value;
        focuser_position.min_value = min_value;
        focuser_position.max_value = max_value;
        focuser_position.state = property->state;
        focuser_position.perm = property->perm;
        focuser_position.defined = true;
      }

      emit focuserPositionChanged();
    }

  }

  else if( match_property_name(property, FOCUSER_LIMITS_PROPERTY_NAME) ) {

    int minpos, maxpos;

    if( get_focuser_limits(property, &minpos, &maxpos) ) {

      if( true ) {
        c_guard_lock lock(mtx_);

        focuser_limits.min_value = minpos;
        focuser_limits.max_value = maxpos;
        focuser_limits.perm = property->perm;
        focuser_limits.state = property->state;
        focuser_limits.defined = true;
      }

      emit focuserLimitsChanged();
    }
  }

  else if ( match_property_name(property, FOCUSER_STEPS_PROPERTY_NAME) ) {

    indigo_property_state property_state;
    int steps;
    int min_value;
    int max_value;

    if( get_focuser_steps(property, &steps, &min_value, &max_value) ) {

      if ( true ) {
        c_guard_lock lock(mtx_);

        focuser_steps.value = steps;
        focuser_steps.min_value = min_value;
        focuser_steps.max_value = max_value;
        focuser_steps.state = property->state;
        focuser_steps.perm = property->perm;
        focuser_steps.defined = true;
      }

      emit focuserStepsChanged();

    }

  }

  else if( match_property_name(property, FOCUSER_SPEED_PROPERTY_NAME) ) {

    int current_value;
    int min_value;
    int max_value;

    if ( get_focuser_speed(property, &current_value, &min_value, &max_value) ) {

      if( true ) {
        c_guard_lock lock(mtx_);

        focuser_speed.value = current_value;
        focuser_speed.min_value = min_value;
        focuser_speed.max_value = max_value;
        focuser_speed.state = property->state;
        focuser_speed.perm = property->perm;
        focuser_speed.defined = true;
      }

      emit focuserSpeedChanged();
    }

  }

  else if( match_property_name(property, FOCUSER_TEMPERATURE_PROPERTY_NAME) ) {

    double temperature;

    if( get_focuser_temperature(property, &temperature) ) {

      if( true ) {
        c_guard_lock lock(mtx_);

        focuser_temperature.value = temperature;
        focuser_temperature.defined = true;
      }

      emit focuserTemperatureChanged();
    }

  }
  else if( property ) {
    // DEVICE_PORTS
    // DEVICE_PORT
    INDIGO_DEBUG("\nUNHANDLED: %s\n",
        dump_indigo_property(property).c_str());
  }

}

void QIndigoFocuserWidget::abortCurrentMotion()
{
  if( client_ && !currentDeviceName_.isEmpty() ) {
    client_->change_switch_property(
        currentDeviceName_.toUtf8().constData(),
        FOCUSER_ABORT_MOTION_PROPERTY_NAME,
        FOCUSER_ABORT_MOTION_ITEM_NAME,
        true);
  }
}

void QIndigoFocuserWidget::moveFocusToPosition(int target_pos)
{
  if ( target_pos != focuser_position.value ) {

    indigo_result status =
        client_->change_number_property(
            currentDeviceName_.toUtf8().constData(),
            FOCUSER_POSITION_PROPERTY_NAME,
            FOCUSER_POSITION_ITEM_NAME,
            target_pos);

    if( status != INDIGO_OK ) {
      INDIGO_ERROR("%s(): client_->change_number_property(%s:%s) fails: "
          "status=%d\n",
          __func__,
          FOCUSER_POSITION_PROPERTY_NAME,
          FOCUSER_POSITION_ITEM_NAME,
          status);
    }
  }
}

bool QIndigoFocuserWidget::setMovingDirection(INDIGO_FOCUSER_MOVE_DIRECTION direction)
{
  if( focuser_direction.defined && focuser_direction.state != INDIGO_BUSY_STATE ) {

    indigo_result status =
        INDIGO_OK;

    if( direction != focuser_direction.value ) {
      switch (direction) {
      case INDIGO_FOCUSER_MOVE_DIRECTION_INWARD:
        status =
            client_->change_switch_property(
                currentDeviceName_.toUtf8().constData(),
                FOCUSER_DIRECTION_PROPERTY_NAME,
                FOCUSER_DIRECTION_MOVE_INWARD_ITEM_NAME,
                true);
        break;

      case INDIGO_FOCUSER_MOVE_DIRECTION_OUTWARD:
        status =
            client_->change_switch_property(
                currentDeviceName_.toUtf8().constData(),
                FOCUSER_DIRECTION_PROPERTY_NAME,
                FOCUSER_DIRECTION_MOVE_OUTWARD_ITEM_NAME,
                true);
        break;
      default:
        break;
      }
    }

    if( status == INDIGO_OK ) {
      return true;
    }

    INDIGO_ERROR("%s(): client_->change_switch_property('%s') fails: "
        "status=%d\n",
        __func__,
        FOCUSER_DIRECTION_PROPERTY_NAME,
        status);
  }

  return false;
}



void QIndigoFocuserWidget::moveFocusInward(int steps)
{
  if( steps > 0 ) {

    if( !setMovingDirection(INDIGO_FOCUSER_MOVE_DIRECTION_INWARD) ) {
      INDIGO_ERROR("%s(): setMovingDirection(INWARD) fails",
          __func__);
      return;
    }

    indigo_result status =
        client_->change_number_property(
            currentDeviceName_.toUtf8().constData(),
            FOCUSER_STEPS_PROPERTY_NAME,
            FOCUSER_STEPS_ITEM_NAME,
            steps);

    if( status != INDIGO_OK ) {

      INDIGO_ERROR("%s(): client_->change_number_property('%s', %s, %s) fails: "
          "status=%d\n",
          __func__,
          currentDeviceName_.toUtf8().constData(),
          FOCUSER_STEPS_PROPERTY_NAME,
          FOCUSER_STEPS_ITEM_NAME,
          status);

      return;
    }
  }
}

void QIndigoFocuserWidget::moveFocusOutward(int steps)
{
  if( steps > 0 ) {

    if( !setMovingDirection(INDIGO_FOCUSER_MOVE_DIRECTION_OUTWARD) ) {
      INDIGO_ERROR("%s(): setMovingDirection(OUTWARD) fails",
          __func__);
      return;
    }

    indigo_result status =
        client_->change_number_property(
            currentDeviceName_.toUtf8().constData(),
            FOCUSER_STEPS_PROPERTY_NAME,
            FOCUSER_STEPS_ITEM_NAME,
            steps);

    if( status != INDIGO_OK ) {

      INDIGO_ERROR("%s(): client_->change_number_property('%s', %s, %s) fails: "
          "status=%d\n",
          __func__,
          currentDeviceName_.toUtf8().constData(),
          FOCUSER_STEPS_PROPERTY_NAME,
          FOCUSER_STEPS_ITEM_NAME,
          status);

      return;
    }
  }
}

void QIndigoFocuserWidget::onIndigoClientAttach()
{
}



void QIndigoFocuserWidget::onIndigoClientDefineProperty(const indigo_device * device, const indigo_property * property, const QString & message)
{
  if( property ) {
    INDIGO_TRACE("\n%s\n",
        dump_indigo_property(property).c_str());
  }

  if ( !message.isEmpty() ) {
    INDIGO_TRACE("\nmessage: %s\n",
        message.toUtf8().constData());
  }

  if( match_device_name(property, currentDeviceName_) ) {
    updateCurrentDeviceProperties(property, message);
  }
}

void QIndigoFocuserWidget::onIndigoClientUpdateProperty(const indigo_device * device, const indigo_property * property, const QString & message)
{
  if( property ) {
    INDIGO_TRACE("\n%s\n",
        dump_indigo_property(property).c_str());
  }

  if ( !message.isEmpty() ) {
    INDIGO_TRACE("\nmessage: %s\n",
        message.toUtf8().constData());
  }

  if( match_device_name(property, currentDeviceName_) ) {
    updateCurrentDeviceProperties(property, message);
  }
}

void QIndigoFocuserWidget::onIndigoClientDeleteProperty(const indigo_device * device, const indigo_property * property, const QString & message)
{
}

void QIndigoFocuserWidget::onIndigoClientSendMessage(const indigo_device * device, const char * message)
{
}

void QIndigoFocuserWidget::onIndigoClientDetach()
{
}



//////////////////////////////////////////////////////////////////////////
// CONNECTION MANAGEMENT

void QIndigoFocuserWidget::onDeviceSelectorCurrentIndexChanged(int cursel)
{
  if ( cursel >= 0 ) {
    currentDeviceName_ =
        deviceSelector_ctl->currentText();
  }
  else {
    currentDeviceName_.clear();
  }
}

void QIndigoFocuserWidget::onConnectDiconnectButtonClicked()
{
  if( client_ && !currentDeviceName_.isEmpty() ) {

    const char *propertyItemName =
        focuser_connection.connection_status == INDIGO_CONNECTION_STATUS_CONNECTED ?
        CONNECTION_DISCONNECTED_ITEM_NAME :
        CONNECTION_CONNECTED_ITEM_NAME;

    indigo_result status =
        client_->change_switch_property(
            currentDeviceName_.toUtf8().constData(),
            CONNECTION_PROPERTY_NAME,
            propertyItemName,
            true);

    if( status != INDIGO_OK ) {

      INDIGO_ERROR("%s(): client_->change_switch_property('%s', %s:%s) fails: "
          "status=%d\n",
          __func__,
          currentDeviceName_.toUtf8().constData(),
          CONNECTION_PROPERTY_NAME,
          propertyItemName,
          status);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
// FOCUSER LIMITS

void QIndigoFocuserWidget::onApplyFocusLimitsClicked()
{
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
  const QStringList tokens =
      focuserLimits_ctl->text().split(QRegularExpression("[ ;:\t\n]"),
          Qt::SkipEmptyParts);
#elif QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
  const QStringList tokens =
      focuserLimits_ctl->text().split(QRegExp("[ ;:\t\n]"),
          Qt::SkipEmptyParts);
#else
  const QStringList tokens =
      focuserLimits_ctl->text().split(QRegExp("[ ;:\t\n]"),
          QString::SkipEmptyParts);
#endif

  if ( tokens.size() != 2 ) {
    INDIGO_ERROR("%s(): n=%d s='%s'",
        __func__,
        tokens.size(),
        focuserLimits_ctl->text().toUtf8().constData());
    return;
  }

  int minval, maxval;

  if ( sscanf(tokens[0].toUtf8().constData(), "%d", &minval) != 1 ) {
    INDIGO_ERROR("%s(): sscanf() fails: %s'",
        __func__,
        tokens[0].toUtf8().constData());
    return;
  }

  if ( sscanf(tokens[1].toUtf8().constData(), "%d", &maxval) != 1 ) {
    INDIGO_ERROR("%s(): sscanf() fails: %s'",
        __func__,
        tokens[0].toUtf8().constData());
    return;
  }

  const char *items[] = {
      FOCUSER_LIMITS_MIN_POSITION_ITEM_NAME,
      FOCUSER_LIMITS_MAX_POSITION_ITEM_NAME,
  };

  const double values[2] = {
      (double) minval,
      (double) maxval
  };

  indigo_result status =
      client_->change_number_property(
          currentDeviceName_.toUtf8().constData(),
          FOCUSER_LIMITS_PROPERTY_NAME,
          2,
          items,
          values);

  if( status != INDIGO_OK ) {
    INDIGO_ERROR("%s(): client_->change_number_property(%s) fails: "
        "status=%d\n",
        __func__,
        FOCUSER_LIMITS_PROPERTY_NAME,
        status);
  }
}


/////////////////////////////////////////////////////////////////////////////
// FOCUSER_DIRECTION MANAGEMENT

void QIndigoFocuserWidget::onFocuserPositionMoveToAbsolutePositionClicked()
{
  if( isMovingFocusNow() ) {
    abortCurrentMotion();
  }
  else if ( canMoveFocusNow() ) {

    const int target_pos =
        focuserPosition_ctl->value();

    moveFocusToPosition(target_pos);
  }
}


void QIndigoFocuserWidget::onFocuserPositionMoveToMinPositionClicked()
{
  if( isMovingFocusNow() ) {
    abortCurrentMotion();
  }
  else if( canMoveFocusNow() ) {

    focuserPosition_ctl->setValue(
        focuserPosition_ctl->minimum());

    const int target_pos =
        focuserPosition_ctl->value();

    moveFocusToPosition(target_pos);
  }
}

void QIndigoFocuserWidget::onFocuserPositionMoveToMaxPositionClicked()
{
  if( isMovingFocusNow() ) {
    abortCurrentMotion();
  }
  else if( canMoveFocusNow() ) {

    focuserPosition_ctl->setValue(
        focuserPosition_ctl->maximum());

    const int target_pos =
        focuserPosition_ctl->value();

    moveFocusToPosition(target_pos);
  }
}

void QIndigoFocuserWidget::onFocuserPositionMoveToMiddlePositionClicked()
{
  if( isMovingFocusNow() ) {
    abortCurrentMotion();
  }
  else if( canMoveFocusNow() ) {

    focuserPosition_ctl->setValue((focuserPosition_ctl->maximum() +
        focuserPosition_ctl->minimum()) / 2);

    const int target_pos =
        focuserPosition_ctl->value();

    moveFocusToPosition(target_pos);
  }
}


// FOCUSER RELATIVE MOTION

void QIndigoFocuserWidget::onFocuserStepsMoveInwardClicked()
{
  if ( isMovingFocusNow() ) {
    abortCurrentMotion();
  }
  else {
    moveFocusInward(focuserSteps_ctl->value());
  }
}

void QIndigoFocuserWidget::onFocuserStepsMoveInward4xClicked()
{
  if( isMovingFocusNow() ) {
    abortCurrentMotion();
  }
  else {
    moveFocusInward(4 * focuserSteps_ctl->value());
  }
}

void QIndigoFocuserWidget::onFocuserStepsMoveOutwardClicked()
{
  if ( isMovingFocusNow() ) {
    abortCurrentMotion();
  }
  else {
    moveFocusOutward(focuserSteps_ctl->value());
  }
}

void QIndigoFocuserWidget::onFocuserStepsMoveOutward4xClicked()
{
  if( isMovingFocusNow() ) {
    abortCurrentMotion();
  }
  else {
    moveFocusOutward(4 * focuserSteps_ctl->value());
  }
}

//////////////////////////////////////////////////////////////////////////
// FOCUSER SPEED

void QIndigoFocuserWidget::setFocuserSpeed(int value)
{
  if( value != focuser_speed.value ) {

    indigo_result status =
        client_->change_number_property(
            currentDeviceName_.toUtf8().constData(),
            FOCUSER_SPEED_PROPERTY_NAME,
            FOCUSER_SPEED_ITEM_NAME,
            value);

    if( status != INDIGO_OK ) {

      INDIGO_ERROR("%s(): client_->change_number_property('%s') fails: "
          "status=%d\n",
          __func__,
          FOCUSER_SPEED_PROPERTY_NAME,
          status);
    }
  }
}


void QIndigoFocuserWidget::onFocuserSpeedControlValueChanged(int value)
{
  if( isConnected() && focuser_speed.defined && focuser_speed.state != INDIGO_BUSY_STATE ) {
    if( value >= focuser_speed.min_value && value <= focuser_speed.max_value ) {
      setFocuserSpeed(value);
    }
  }
}


//////////////////////////////////////////////////////////////////////////






//void QIndigoFocuserWidget::mousePressEvent(QMouseEvent *event)
//{
////  
//
//  if ( enableMouse_ctl->isChecked() && canMoveFocusNow() ) {
//
//    if ( event->button() == Qt::MouseButton::LeftButton ) {
//
//      moveFocusToPosition(focuserPosition_ctl->minimum());
//
//      return;
//    }
//
//    if ( event->button() == Qt::MouseButton::RightButton ) {
//
//      moveFocusToPosition(focuserPosition_ctl->maximum());
//
//      return;
//
//    }
//
//  }
//
//  Base::mousePressEvent(event);
//}

//void QIndigoFocuserWidget::mouseReleaseEvent(QMouseEvent *event)
//{
//  //
//
//  if ( isMovingFocusNow() ) {
//    abortCurrentMotion();
//    return;
//  }
//
//  Base::mouseReleaseEvent(event);
//}

//void QIndigoFocuserWidget::mouseDoubleClickEvent(QMouseEvent *event)
//{
//  Base::mouseDoubleClickEvent(event);
//}
//
//void QIndigoFocuserWidget::mouseMoveEvent(QMouseEvent *event)
//{
//  Base::mouseMoveEvent(event);
//}

#if QT_CONFIG(wheelevent)
void QIndigoFocuserWidget::wheelEvent(QWheelEvent * event)
{
  if( isConnected() && focuser_speed.defined && focuser_speed.state != INDIGO_BUSY_STATE ) {

    const int steps =
        event->angleDelta().y() / QWheelEvent::DefaultDeltasPerStep;

    if( steps > 0 ) {
      if( focuser_speed.value < focuser_speed.max_value ) {
        setFocuserSpeed(focuser_speed.value + 1);
        return;
      }
    }
    else if( steps < 0 ) {
      if( focuser_speed.value > focuser_speed.min_value ) {
        setFocuserSpeed(focuser_speed.value - 1);
        return;
      }
    }
  }

  Base::wheelEvent(event);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

QIndigoFocuserMouseClickControl::QIndigoFocuserMouseClickControl(QIndigoFocuserWidget * parent) :
    Base(parent),
    focuser_(parent)
{
  setPixmap(QPixmap(ICON_mc));
}

void QIndigoFocuserMouseClickControl::mousePressEvent(QMouseEvent * e)
{
  if( focuser_->enableMouse_ctl->isChecked() && focuser_->canMoveFocusNow() ) {

    if( e->button() == Qt::MouseButton::LeftButton ) {

      focuser_->moveFocusToPosition(focuser_->focuserPosition_ctl->minimum());

      return;
    }

    if( e->button() == Qt::MouseButton::RightButton ) {

      focuser_->moveFocusToPosition(focuser_->focuserPosition_ctl->maximum());

      return;
    }
  }

  e->ignore();
  // Base::mousePressEvent(e);
}

void QIndigoFocuserMouseClickControl::mouseReleaseEvent(QMouseEvent *e)
{
  if ( focuser_->isMovingFocusNow() ) {
    focuser_->abortCurrentMotion();
  }

  e->ignore();
  //Base::mouseReleaseEvent(e);
}


