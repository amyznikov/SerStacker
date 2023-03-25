/*
 * QIndigoClient.h
 *
 *  Created on: Nov 9, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __QIndigoClient_h__
#define __QIndigoClient_h__

#include <QtCore/QtCore>
#include <indigo/indigo_bus.h>
#include <indigo/indigo_client.h>
#include <indigo/indigo_names.h>

class QIndigoClient:
    public QObject
{
  Q_OBJECT;
public:
  typedef QIndigoClient ThisClass;
  typedef QObject Base;

  QIndigoClient(const QString & name, QObject * parent = Q_NULLPTR);
  ~QIndigoClient();

  void setEnableBlobs(bool v);
  bool enableBlobs() const;

  indigo_result start();
  bool started() const;
  void stop();

  indigo_result load_driver(const QString & name);
  indigo_result remove_driver(const QString & name);
  indigo_result enumerate_properties(indigo_property * property = nullptr);

  indigo_result change_switch_property(const char * deviceName,
      const char * propertyName,
      const char * itemName,
      bool value);

  indigo_result change_number_property(const char *deviceName,
      const char * propertyName,
      const char * itemName,
      double value);

  indigo_result change_number_property(const char *deviceName,
      const char * propertyName,
      int count,
      const char *items[],
      const double values[]);

Q_SIGNALS:
  void clientAttach();
  void clientDefineProperty(const indigo_device * device, const indigo_property * property, const QString & message);
  void clientUpdateProperty(const indigo_device * device, const indigo_property * property, const QString & message);
  void clientDeleteProperty(const indigo_device * device, const indigo_property * property, const QString & message);
  void clientSendMessage(const indigo_device * device, const char * message);
  void clientDetach();


protected:
  // callback called when client is attached to the bus
  virtual indigo_result onClientAttach();

  // callback called when device broadcast property definition
  virtual indigo_result onClientDefineProperty(indigo_device *device,
       indigo_property *property,
       const char *message);

   // callback called when device broadcast property value change
  virtual indigo_result onClientUpdateProperty(indigo_device *device,
       indigo_property *property,
       const char *message);

   // callback called when device broadcast property removal
  virtual indigo_result onClientDeleteProperty(indigo_device *device,
       indigo_property *property,
       const char *message);

   // callback called when device broadcast a message
  virtual indigo_result onClientSendMessage(indigo_device *device,
       const char *message);

   // callback called when client is detached from the bus
  virtual indigo_result onClientDetach();


private:

  // callback called when client is attached to the bus
  static indigo_result indigo_client_attach(indigo_client *client);

  // callback called when device broadcast property definition
  static indigo_result indigo_client_define_property(indigo_client *client,
      indigo_device *device,
      indigo_property *property,
      const char *message);

  // callback called when device broadcast property value change
  static indigo_result indigo_client_update_property(indigo_client *client,
      indigo_device *device,
      indigo_property *property,
      const char *message);

  // callback called when device broadcast property removal
  static indigo_result indigo_client_delete_property(indigo_client *client,
      indigo_device *device,
      indigo_property *property,
      const char *message);

  // callback called when device broadcast a message
  static indigo_result indigo_client_send_message(indigo_client *client,
      indigo_device *device,
      const char *message);

  // callback called when client is detached from the bus
  static indigo_result indigo_client_detach(indigo_client *client);

protected:
  indigo_client indigo_client_ = {0};
  QList<indigo_driver_entry*> drivers_;
  bool started_ = false;
  bool enableBlobs_ = false;
  int guider_save_bandwidth_ = 1;

};


/**
property: device='FocusDreamPro' name='CONNECTION' group='Main' label='Connection status' hints='' state=1 type=3 perm=2
   item[0]: name=CONNECTED label=Connected hints= sw.value=0
   item[1]: name=DISCONNECTED label=Disconnected hints= sw.value=1
*/
std::string dump_indigo_property(const indigo_property * property);


/**
* property: device='FocusDreamPro' name='CONNECTION' group='Main' label='Connection status' hints='' state=1 type=3 perm=2
*    item[0]: name=CONNECTED label=Connected hints= sw.value=0
*    item[1]: name=DISCONNECTED label=Disconnected hints= sw.value=1
*/
enum INDIGO_CONNECTION_STATUS {
  INDIGO_CONNECTION_STATUS_UNKNOWN,
  INDIGO_CONNECTION_STATUS_CONNECTED,
  INDIGO_CONNECTION_STATUS_DISCONNECTED
};
bool get_connection_status(const indigo_property * connection_status_property,
    /*out */INDIGO_CONNECTION_STATUS * connection_status);



/**
 * property: device='FocusDreamPro' name='FOCUSER_DIRECTION' group='Focuser' label='Movement direction' hints='' state=1 type=3 perm=2
 *  item[0]: name=MOVE_INWARD label=Move inward hints= value=1
 *  item[1]: name=MOVE_OUTWARD label=Move outward hints= value=0
 */
enum INDIGO_FOCUSER_MOVE_DIRECTION {
  INDIGO_FOCUSER_MOVE_DIRECTION_UNKNOWN,
  INDIGO_FOCUSER_MOVE_DIRECTION_INWARD,
  INDIGO_FOCUSER_MOVE_DIRECTION_OUTWARD,
};

bool get_focuser_move_direction(const indigo_property * focuser_direction_property,
    /*out*/ INDIGO_FOCUSER_MOVE_DIRECTION * direction);


/**
 * property: device='FocusDreamPro' name='FOCUSER_LIMITS' group='Focuser' label='Focuser Limits' hints='' state=1 type=2 perm=2
 *  item[0]: name=MIN_POSITION label=Minimum (steps) hints= number.value=0 format=%g min=0 max=1000 step=1 target=0
 *  item[1]: name=MAX_POSITION label=Maximum (steps) hints= number.value=1e+06 format=%g min=0 max=1000 step=1 target=1e+06
 */
bool get_focuser_limits(const indigo_property * focuser_limits_property,
    /*out */ int * minpos,
    /*out */ int * maxpos);


/**
 * property: device='FocusDreamPro' name='FOCUSER_POSITION' group='Focuser' label='Absolute position' hints='' state=2 type=2 perm=2
 *  item[0]: name=POSITION label=Absolute position hints= number.value=205 format=%g min=0 max=1e+06 step=1 target=205
 */
bool get_focuser_position(const indigo_property * focuser_position_property,
    /*out, opt */int * current_value,
    /*out, opt */int * min_value,
    /*out, opt */int * max_value);


/**
 * property: device='FocusDreamPro' name='FOCUSER_STEPS' group='Focuser' label='Relative move' hints='' state=2 type=2 perm=2
 *  item[0]: name=STEPS label=Relative move (steps) hints= number.value=300 format=%g min=0 max=100000 step=1 target=300
 */
bool get_focuser_steps(const indigo_property * focuser_steps_property,
    /*out */int * steps,
    /*out */int * min_value,
    /*out */int * max_value);

/**
 * property: device='FocusDreamPro' name='FOCUSER_SPEED' group='Focuser' label='Focuser speed' hints='' state=1 type=2 perm=2
 *  item[0]: name=SPEED label=Speed hints= number.value=1 format=%g min=0 max=5 step=1 target=1
 */
bool get_focuser_speed(const indigo_property * focuser_speed_property,
    /*out */int * current_value,
    /*out */int * min_value,
    /*out */int * max_value);

/**
 * property: device='FocusDreamPro' name='FOCUSER_TEMPERATURE' group='Focuser' label='Temperature' hints='' state=1 type=2 perm=1
 *  item[0]: name=TEMPERATURE label=Temperature (°C) hints= number.value=21.25 format=%g min=-50 max=50 step=1 target=21.25
 */
bool get_focuser_temperature(const indigo_property * focuser_temperature_property,
    /*out */ double * temperature);


inline bool match_property_name(const indigo_property * property, const char * name)
{
  return property && strcmp(property->name, name) == 0;
}

inline bool match_device_name(const indigo_property * property, const char * device)
{
  return property && device && *device && strcmp(property->device, device) == 0;
}

inline bool match_device_name(const indigo_property * property, const QString & device)
{
  return property && !device.isEmpty() && strcmp(property->device, device.toUtf8().constData()) == 0;
}

inline bool match_item_name(const indigo_item * item, const char * name)
{
  return item && strcmp(item->name, name) == 0;
}

inline bool match_item_name(const indigo_item & item, const char * name)
{
  return strcmp(item.name, name) == 0;
}




#endif /* __QIndigoClient_h__ */
