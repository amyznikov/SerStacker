/*
 * QIndigoFocuserWidget.h
 *
 *  Created on: Nov 8, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QIndigoFocuserWidget_h__
#define __QIndigoFocuserWidget_h__

#include <gui/widgets/QSettingsWidget.h>
#include "QIndigoClient.h"

class QIndigoFocuserMouseClickControl;

class QIndigoFocuserWidget:
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QIndigoFocuserWidget ThisClass;
  typedef QSettingsWidget Base;
  friend class QIndigoFocuserMouseClickControl;

  typedef std::lock_guard<std::mutex>
    c_guard_lock;

  QIndigoFocuserWidget(QWidget * parent = nullptr);
  ~QIndigoFocuserWidget();

  void setIndigoClient(QIndigoClient * client);
  QIndigoClient * indigoClient() const;

  bool isConnected() const;
  bool canMoveFocusNow() const;
  bool isMovingFocusNow() const;

signals:
  void focuserConnectionStateChanged();
  void focuserMoveDirectionChanged();
  void focuserPositionChanged();
  void focuserLimitsChanged();
  void focuserStepsChanged();
  void focuserSpeedChanged();
  void focuserTemperatureChanged();

protected:
  void onload(QSettings & settings) override;
  void onupdatecontrols() override;
  void updateCurrentDeviceProperties(const indigo_property * property, const QString & message);
  void abortCurrentMotion();
  void moveFocusToPosition(int target_pos);
  bool setMovingDirection(INDIGO_FOCUSER_MOVE_DIRECTION direction);
  void moveFocusInward(int steps);
  void moveFocusOutward(int steps);
  void enumerateCurrentDeviceProperties();
  void setFocuserSpeed(int value);


protected Q_SLOTS:
  void onIndigoClientAttach();
  void onIndigoClientDefineProperty(const indigo_device * device, const indigo_property * property, const QString & message);
  void onIndigoClientUpdateProperty(const indigo_device * device, const indigo_property * property, const QString & message);
  void onIndigoClientDeleteProperty(const indigo_device * device, const indigo_property * property, const QString & message);
  void onIndigoClientSendMessage(const indigo_device * device, const char * message);
  void onIndigoClientDetach();
  void onFocuserConnectionStateChanged();
  void onFocuserMoveDirectionChanged();
  void onFocuserPositionChanged();
  void onFocuserLimitsChanged();
  void onFocuserStepsChanged();
  void onFocuserSpeedChanged();
  void onFocuserTemperatureChanged();

  // FOCUSER SELECTION AND CONNECTION STATUS
protected:
  struct c_focuser_connection {
    INDIGO_CONNECTION_STATUS connection_status = INDIGO_CONNECTION_STATUS_UNKNOWN;
    indigo_property_state state = INDIGO_IDLE_STATE;
    bool defined = false;
  } focuser_connection;

  QComboBox * deviceSelector_ctl = nullptr;
  QToolButton * connectDisconnect_ctl = nullptr;
protected Q_SLOTS:
  void onDeviceSelectorCurrentIndexChanged(int);
  void onConnectDiconnectButtonClicked();

  // DEVICE PORTS
protected:


  // FOCUSER LIMITS
protected:
  struct c_focuser_limits {
    int min_value = 0;
    int max_value = 0;
    indigo_property_state state = INDIGO_IDLE_STATE;
    indigo_property_perm perm = INDIGO_RW_PERM;
    bool defined = false;
  } focuser_limits;

  QLineEditBox * focuserLimits_ctl = nullptr;
  // QToolButton * focuserLimitsApply_ctl = nullptr;

protected Q_SLOTS:
  void onApplyFocusLimitsClicked();


  // FOCUSER POSITION
protected:
  struct c_focuser_position {
    int value = 0;
    int min_value = 0;
    int max_value = 0;
    indigo_property_state state = INDIGO_IDLE_STATE;
    indigo_property_perm perm = INDIGO_RW_PERM;
    bool defined = false;
  } focuser_position;

  QSpinBox * focuserPosition_ctl = nullptr;
  QToolButton * focuserPositionMoveToAbsolutePosition_ctl = nullptr;
  QToolButton * focuserPositionMoveToMinPosition_ctl = nullptr;
  QToolButton * focuserPositionMoveToMaxPosition_ctl = nullptr;

protected Q_SLOTS:
  void onFocuserPositionMoveToAbsolutePositionClicked();
  void onFocuserPositionMoveToMinPositionClicked();
  void onFocuserPositionMoveToMaxPositionClicked();

  // FOCUSER RELATIVE MOTION
protected:
  struct c_focuser_direction {
    INDIGO_FOCUSER_MOVE_DIRECTION value = INDIGO_FOCUSER_MOVE_DIRECTION_UNKNOWN;
    indigo_property_state state = INDIGO_IDLE_STATE;
    bool defined = false;
  } focuser_direction;

  struct c_focuser_steps {
    int value = 0;
    int min_value = 0;
    int max_value = 0;
    indigo_property_state state = INDIGO_IDLE_STATE;
    indigo_property_perm perm = INDIGO_RW_PERM;
    bool defined = false;
  } focuser_steps;

  QSpinBox * focuserSteps_ctl = nullptr;
  QToolButton * focuserStepsMoveInward_ctl = nullptr;
  QToolButton * focuserStepsMoveInward4x_ctl = nullptr;
  QToolButton * focuserStepsMoveOutward_ctl = nullptr;
  QToolButton * focuserStepsMoveOutward4x_ctl = nullptr;
protected Q_SLOTS:
  void onFocuserStepsMoveInwardClicked();
  void onFocuserStepsMoveInward4xClicked();
  void onFocuserStepsMoveOutwardClicked();
  void onFocuserStepsMoveOutward4xClicked();


  // FOCUSER SPEED
protected:
  struct focuser_speed {
    int value = 0;
    int min_value = 0;
    int max_value = 0;
    indigo_property_state state = INDIGO_IDLE_STATE;
    indigo_property_perm perm = INDIGO_RW_PERM;
    bool defined = false;
  } focuser_speed;

  QSpinBox * focuserSpeed_ctl = nullptr;
protected Q_SLOTS:
  void onFocuserSpeedControlValueChanged(int value);



  // FOCUSER TEMPERATURE
protected:
  struct c_focuser_temperature {
    double value;
    bool defined = false;
  } focuser_temperature;

protected: // overrides
  //void mousePressEvent(QMouseEvent *event) override;
  //void mouseReleaseEvent(QMouseEvent *event) override;
//  void mouseDoubleClickEvent(QMouseEvent *event) override;
//  void mouseMoveEvent(QMouseEvent *event) override;
#if QT_CONFIG(wheelevent)
  void wheelEvent(QWheelEvent *event) override;
#endif

protected:
  QIndigoClient * client_ = nullptr;
  QString currentDeviceName_;
  QIndigoFocuserMouseClickControl * mouseclick_ctl = nullptr;
  QLabel * status_ctl = nullptr;
  QLabel * temperature_ctl = nullptr;
  std::mutex mtx_;

#if QT_CONFIG(wheelevent)
  QCheckBox * enableMouse_ctl = nullptr;
#endif

  // INDIGO LOG
protected:
  static void on_indigo_log_message(const char *message);
  static QPlainTextEdit * logWidget_ctl;
};

class QIndigoFocuserMouseClickControl :
    public QLabel
{
public:
  typedef QIndigoFocuserMouseClickControl ThisClass;
  typedef QLabel Base;

  QIndigoFocuserMouseClickControl(QIndigoFocuserWidget * parent);

protected:
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

protected:
  QIndigoFocuserWidget * focuser_ = nullptr;
};

#endif /* __QIndigoFocuserWidget_h__ */
