/*
 * QIndigoClient.cc
 *
 *  Created on: Nov 9, 2022
 *      Author: amyznikov
 */

#include "QIndigoClient.h"
#include <core/ssprintf.h>

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


template<>
const c_enum_member* members_of<indigo_property_state>()
{
  static constexpr c_enum_member members[] = {
      { INDIGO_IDLE_STATE, "INDIGO_IDLE_STATE", "property is passive (unused by INDIGO)" },
      { INDIGO_OK_STATE, "INDIGO_OK_STATE", "property is in correct state or if operation on property was successful" },
      { INDIGO_BUSY_STATE, "INDIGO_BUSY_STATE", "property is transient state or if operation on property is pending" },
      { INDIGO_ALERT_STATE, "INDIGO_ALERT_STATE", "property is in incorrect state or if operation on property failed" },
      { INDIGO_IDLE_STATE }
  };

  return members;
}

template<>
const c_enum_member* members_of<INDIGO_CONNECTION_STATUS>()
{
  static constexpr c_enum_member members[] = {
      { INDIGO_CONNECTION_STATUS_UNKNOWN, "CONNECTION_STATUS_UNKNOWN", "" },
      { INDIGO_CONNECTION_STATUS_CONNECTED, "CONNECTION_STATUS_CONNECTED", "" },
      { INDIGO_CONNECTION_STATUS_DISCONNECTED, "CONNECTION_STATUS_DISCONNECTED", "" },
      { INDIGO_CONNECTION_STATUS_UNKNOWN }
  };

  return members;
}

template<>
const c_enum_member* members_of<INDIGO_FOCUSER_MOVE_DIRECTION>()
{
  static constexpr c_enum_member members[] = {
      { INDIGO_FOCUSER_MOVE_DIRECTION_UNKNOWN, "FOCUSER_MOVE_DIRECTION_UNKNOWN", "" },
      { INDIGO_FOCUSER_MOVE_DIRECTION_INWARD, "FOCUSER_MOVE_DIRECTION_INWARD", "" },
      { INDIGO_FOCUSER_MOVE_DIRECTION_OUTWARD, "FOCUSER_MOVE_DIRECTION_OUTWARD", "" },
      { INDIGO_FOCUSER_MOVE_DIRECTION_UNKNOWN }
  };

  return members;
}

std::string dump_indigo_property(const indigo_property * property)
{
  std::string s;

  if( !property ) {
    s = "property: nullptr\n";
  }
  else {

    s =
        ssprintf("property: device='%s' name='%s' group='%s' label='%s' hints='%s' state=%d type=%d perm=%d\n",
            property->device,
            property->name,
            property->group,
            property->label,
            property->hints,
            property->state,
            property->type,
            property->perm);

    for( int i = 0; i < property->count; ++i ) {
      switch (property->type) {
      case INDIGO_TEXT_VECTOR:     ///< strings of limited width
        s += ssprintf("   item[%d]: name=%s label=%s hints=%s text.value='%s'\n", i,
            property->items[i].name,
            property->items[i].label,
            property->items[i].hints,
            property->items[i].text.value);
        break;
      case INDIGO_NUMBER_VECTOR:       ///< float numbers with defined min, max values and increment
        s += ssprintf("   item[%d]: name=%s label=%s hints=%s number.value=%g format=%s min=%g max=%g step=%g target=%g\n", i,
            property->items[i].name,
            property->items[i].label,
            property->items[i].hints,
            property->items[i].number.value,
            property->items[i].number.format,   ///< item format (for number properties)
            property->items[i].number.min,      ///< item min value (for number properties)
            property->items[i].number.max,      ///< item max value (for number properties)
            property->items[i].number.step,     ///< item increment value (for number properties)
            property->items[i].number.value,    ///< item value (for number properties)
            property->items[i].number.target);  ///< item target value (for number properties)
        break;
      case INDIGO_SWITCH_VECTOR:       ///< logical values representing “on” and “off” state
        s += ssprintf("   item[%d]: name=%s label=%s hints=%s sw.value=%d\n", i,
            property->items[i].name,
            property->items[i].label,
            property->items[i].hints,
            property->items[i].sw.value);
        break;
      case INDIGO_LIGHT_VECTOR: ///< status values with four possible values INDIGO_IDLE_STATE, INDIGO_OK_STATE, INDIGO_BUSY_STATE and INDIGO_ALERT_STATE
        s += ssprintf("   item[%d]: name=%s label=%s hints=%s light.value=%d\n", i,
            property->items[i].name,
            property->items[i].label,
            property->items[i].hints,
            property->items[i].light.value);
        break;
      case INDIGO_BLOB_VECTOR:          ///< binary data of any type and any length
        s += ssprintf("   item[%d]: name=%s label=%s hints=%s blob.size=%ld\n", i,
            property->items[i].name,
            property->items[i].label,
            property->items[i].hints,
            property->items[i].blob.size);
        break;
      default:
        break;
      }
    }
  }
  return s;
}

bool get_connection_status(const indigo_property * connection_status_property,
    /*out */INDIGO_CONNECTION_STATUS * connection_status)
{

  if( connection_status ) {
    *connection_status =
        INDIGO_CONNECTION_STATUS_UNKNOWN;
  }

  if( match_property_name(connection_status_property, CONNECTION_PROPERTY_NAME) ) {

    for( int i = 0; i < connection_status_property->count; ++i ) {
      if( match_item_name(connection_status_property->items[i], CONNECTION_CONNECTED_ITEM_NAME) ) {
        if( connection_status_property->items[i].sw.value ) {
          if( connection_status ) {
            *connection_status =
                INDIGO_CONNECTION_STATUS_CONNECTED;
          }
          return true;
        }
      }
      else if( match_item_name(connection_status_property->items[i], CONNECTION_DISCONNECTED_ITEM_NAME) ) {
        if( connection_status_property->items[i].sw.value ) {
          if( connection_status ) {
            *connection_status =
                INDIGO_CONNECTION_STATUS_DISCONNECTED;
          }
          return true;
        }
      }
    }
  }

  return false;
}


bool get_focuser_move_direction(const indigo_property * focuser_direction_property,
    /*out*/ INDIGO_FOCUSER_MOVE_DIRECTION * direction)
{
  if( direction ) {
    *direction =
        INDIGO_FOCUSER_MOVE_DIRECTION_UNKNOWN;
  }

  if( match_property_name(focuser_direction_property, FOCUSER_DIRECTION_PROPERTY_NAME) ) {

    for( int i = 0; i < focuser_direction_property->count; ++i ) {
      if( match_item_name(focuser_direction_property->items[i], FOCUSER_DIRECTION_MOVE_INWARD_ITEM_NAME) ) {
        if( focuser_direction_property->items[i].sw.value ) {
          if( direction ) {
            *direction =
                INDIGO_FOCUSER_MOVE_DIRECTION_INWARD;
          }
          return true;
        }
      }
      else if( match_item_name(focuser_direction_property->items[i], FOCUSER_DIRECTION_MOVE_OUTWARD_ITEM_NAME) ) {
        if( focuser_direction_property->items[i].sw.value ) {
          if( direction ) {
            * direction =
                INDIGO_FOCUSER_MOVE_DIRECTION_OUTWARD;
          }
          return true;
        }
      }
    }
  }

  return false;
}


bool get_focuser_limits(const indigo_property * focuser_limits_property, int * minpos, int * maxpos)
{
  bool haveMinPos = false;
  bool haveMaxPos = false;

  if( match_property_name(focuser_limits_property, FOCUSER_LIMITS_PROPERTY_NAME) ) {

    for( int i = 0; i < focuser_limits_property->count; ++i ) {
      if( match_item_name(focuser_limits_property->items[i], FOCUSER_LIMITS_MIN_POSITION_ITEM_NAME) ) {

        if ( minpos ) {
          *minpos =
              focuser_limits_property->items[i].number.value;
        }

        haveMinPos = true;
      }
      else if( match_item_name(focuser_limits_property->items[i], FOCUSER_LIMITS_MAX_POSITION_ITEM_NAME) ) {

        if ( maxpos ) {
          *maxpos =
              focuser_limits_property->items[i].number.value;
        }

        haveMaxPos = true;
      }

    }
  }

  return haveMinPos && haveMaxPos;
}

bool get_focuser_position(const indigo_property * focuser_position_property,
    /*out, opt */int * current_value,
    /*out, opt */int * min_value,
    /*out, opt */int * max_value)
{
  if( match_property_name(focuser_position_property, FOCUSER_POSITION_PROPERTY_NAME) ) {

    for( int i = 0; i < focuser_position_property->count; ++i ) {
      if( match_item_name(focuser_position_property->items[i], FOCUSER_POSITION_ITEM_NAME) ) {

        if ( current_value ) {
          * current_value =
              focuser_position_property->items[i].number.value;
        }

        if ( min_value ) {
          * min_value =
              focuser_position_property->items[i].number.min;
        }

        if ( max_value ) {
          * max_value =
              focuser_position_property->items[i].number.max;
        }

        return true;
      }

    }
  }

  return false;
}


bool get_focuser_steps(const indigo_property * focuser_steps_property,
    /*out */ int * steps,
    /*out */int * min_value,
    /*out */int * max_value)
{
  if( match_property_name(focuser_steps_property, FOCUSER_STEPS_PROPERTY_NAME) ) {

    for( int i = 0; i < focuser_steps_property->count; ++i ) {
      if( match_item_name(focuser_steps_property->items[i], FOCUSER_STEPS_ITEM_NAME) ) {

        if ( steps ) {
          * steps =
              focuser_steps_property->items[i].number.value;
        }

        if ( min_value ) {
          * min_value =
              focuser_steps_property->items[i].number.min;
        }

        if ( max_value ) {
          * max_value =
              focuser_steps_property->items[i].number.max;
        }

        return true;
      }

    }
  }

  return false;
}


bool get_focuser_speed(const indigo_property * focuser_speed_property,
    /*out */int * current_value,
    /*out */int * min_value,
    /*out */int * max_value)
{
  if( match_property_name(focuser_speed_property, FOCUSER_SPEED_PROPERTY_NAME) ) {

    for( int i = 0; i < focuser_speed_property->count; ++i ) {
      if( match_item_name(focuser_speed_property->items[i], FOCUSER_SPEED_ITEM_NAME) ) {

        if ( current_value ) {
          * current_value =
              focuser_speed_property->items[i].number.value;
        }

        if ( min_value ) {
          * min_value =
              focuser_speed_property->items[i].number.min;
        }

        if ( max_value ) {
          * max_value =
              focuser_speed_property->items[i].number.max;
        }

        return true;
      }
    }
  }

  return false;
}


bool get_focuser_temperature(const indigo_property * focuser_temperature_property,
    /*out */ double * temperature)
{
  if( match_property_name(focuser_temperature_property, FOCUSER_TEMPERATURE_PROPERTY_NAME) ) {

    for( int i = 0; i < focuser_temperature_property->count; ++i ) {
      if( match_item_name(focuser_temperature_property->items[i], FOCUSER_TEMPERATURE_ITEM_NAME) ) {
        * temperature = focuser_temperature_property->items[i].number.value;
        return true;
      }
    }
  }

  return false;
}


QIndigoClient::QIndigoClient(const QString & name, QObject * parent)
  : Base(parent)
{
  // client name
  strncpy(indigo_client_.name,
      name.toUtf8().data(),
      INDIGO_NAME_SIZE - 1);
}

QIndigoClient::~QIndigoClient()
{
  stop();
}

void QIndigoClient::setEnableBlobs(bool v)
{
  enableBlobs_ = v;
}

bool QIndigoClient::enableBlobs() const
{
  return enableBlobs_;
}


bool QIndigoClient::started() const
{
  return started_;
}

indigo_result QIndigoClient::start()
{
  indigo_result status =
      INDIGO_OK;

  if ( !started_ ) {

    indigo_client_.is_remote = false; // is remote client
    indigo_client_.client_context = this;   // any client specific data
    indigo_client_.last_result = INDIGO_OK; // result of last bus operation
    indigo_client_.version = INDIGO_VERSION_CURRENT;  // client version
    indigo_client_.enable_blob_mode_records  = nullptr; // enable blob mode
    indigo_client_.attach = &ThisClass::indigo_client_attach; // callback called when client is attached to the bus
    indigo_client_.define_property = &ThisClass::indigo_client_define_property; // callback called when device broadcast property definition
    indigo_client_.update_property = &ThisClass::indigo_client_update_property; // callback called when device broadcast property value change
    indigo_client_.delete_property = &ThisClass::indigo_client_delete_property; // callback called when device broadcast property removal
    indigo_client_.send_message = &ThisClass::indigo_client_send_message; // callback called when device broadcast a message
    indigo_client_.detach = &ThisClass::indigo_client_detach; // callback called when client is detached from the bus


    if( (status = indigo_start()) != INDIGO_OK ) {
      INDIGO_ERROR("%s(): indigo_start() fails: status=%d\n",
          __func__,
          status);
      return status;
    }

    if( (status = indigo_attach_client(&indigo_client_))!= INDIGO_OK ) {
      INDIGO_ERROR("%s(): indigo_attach_client() fails: status=%d\n",
          __func__,
          status);
      return status;
    }

    started_ = true;
  }

  return status;
}

void QIndigoClient::stop()
{
  if ( started_ ) {
    INDIGO_DEBUG("Shutting down client...\n");
    indigo_detach_client(&indigo_client_);
    indigo_stop();
    started_ = false;
  }
}

indigo_result QIndigoClient::indigo_client_attach(indigo_client *client)
{
  QIndigoClient * _this =
      reinterpret_cast<QIndigoClient * >(
          client->client_context);

  return _this->onClientAttach();
}

// callback called when client is attached to the bus
indigo_result QIndigoClient::onClientAttach()
{
  emit clientAttach();
  return INDIGO_OK;
}


// callback called when device broadcast property definition
indigo_result QIndigoClient::indigo_client_define_property(indigo_client *client,
    indigo_device * device,
    indigo_property *property,
    const char *message)
{
  QIndigoClient *_this =
      reinterpret_cast<QIndigoClient*>(
          client->client_context);

  return _this->onClientDefineProperty( device,
      property,
      message);

}

// callback called when device broadcast property definition
indigo_result QIndigoClient::onClientDefineProperty(indigo_device *device,
     indigo_property *property,
     const char *message)
{

  emit clientDefineProperty(device, property, message);
  return INDIGO_OK;
}

// callback called when device broadcast property value change
indigo_result QIndigoClient::indigo_client_update_property(indigo_client * client,
    indigo_device * device,
    indigo_property * property,
    const char * message)
{
  QIndigoClient *_this =
      reinterpret_cast<QIndigoClient*>(
      client->client_context);

  return _this->onClientUpdateProperty(device,
      property,
      message);
}

// callback called when device broadcast property value change
indigo_result QIndigoClient::onClientUpdateProperty(indigo_device *device,
    indigo_property *property,
    const char *message)
{
  emit clientUpdateProperty(device, property, message);
  return INDIGO_OK;
}

// callback called when device broadcast property removal
indigo_result QIndigoClient::indigo_client_delete_property(indigo_client *client,
    indigo_device *device,
    indigo_property *property,
    const char *message)
{
  QIndigoClient * _this =
      reinterpret_cast<QIndigoClient * >(client->client_context);

  return _this->onClientDeleteProperty(device,
      property,
      message);
}

// callback called when device broadcast property removal
indigo_result QIndigoClient::onClientDeleteProperty(indigo_device *device,
    indigo_property *property,
    const char *message)
{
  emit clientDeleteProperty(device, property, message);
  return INDIGO_OK;
}

// callback called when device broadcast a message
indigo_result QIndigoClient::indigo_client_send_message(indigo_client *client,
    indigo_device *device,
    const char *message)
{
  QIndigoClient * _this =
      reinterpret_cast<QIndigoClient * >(client->client_context);

  return _this->onClientSendMessage(device,
      message);
}

// callback called when device broadcast a message
indigo_result QIndigoClient::onClientSendMessage(indigo_device *device,
    const char *message)
{
  emit clientSendMessage(device, message);
  return INDIGO_OK;
}

// callback called when client is detached from the bus
indigo_result QIndigoClient::indigo_client_detach(indigo_client *client)
{
  QIndigoClient * _this =
      reinterpret_cast<QIndigoClient * >(client->client_context);

  return _this->onClientDetach();
}

// callback called when client is detached from the bus
indigo_result QIndigoClient::onClientDetach()
{
  emit clientDetach();
  return INDIGO_OK;
}

//
indigo_result QIndigoClient::load_driver(const QString & name)
{
  ////////////

  for( const indigo_driver_entry *driver : drivers_ ) {
    if( name.compare(driver->name) == 0 ) {
      INDIGO_DEBUG("%s(): Request to load diver '%s' which is already loaded\n",
          __func__,
          driver->name);
      return INDIGO_OK;
    }
  }

  ////////////


  indigo_driver_entry * driver =
      nullptr;

  indigo_result status =
      indigo_load_driver(name.toUtf8().constData(),
          true,
          &driver);

  if( status == INDIGO_OK ) {

    drivers_.append(driver);

  }
  else {
    INDIGO_ERROR("%s(): indigo_load_driver('%s') fails: status=%d\n",
        __func__,
        name.toUtf8().constData(),
        status);
  }

  ////////////

  return status;
}

indigo_result QIndigoClient::remove_driver(const QString & name)
{
  indigo_result status =
      INDIGO_NOT_FOUND;

  for( int i = 0, n = drivers_.size(); i < n; ++i ) {
    if( name.compare(drivers_[i]->name) == 0 ) {

      status =
          indigo_remove_driver(drivers_[i]);

      drivers_.removeAt(i);
      break;
    }
  }

  return status;
}

indigo_result QIndigoClient::enumerate_properties(indigo_property * property)
{
  indigo_result status;

  if ( !started_ ) {

    INDIGO_ERROR("%s(): Client not started\n",
        __func__);

    status =
        INDIGO_FAILED;
  }
  else {

    status =
        indigo_enumerate_properties(&indigo_client_,
            property ?
                property :
                &INDIGO_ALL_PROPERTIES);

    if ( status != INDIGO_OK ) {
      INDIGO_ERROR("%s(): indigo_enumerate_properties(property=%p) fails: status=%d\n",
          __func__,
          property,
          status);
    }

  }

  return status;
}

indigo_result QIndigoClient::change_switch_property(const char * deviceName,
    const char * propertyName,
    const char * itemName,
    bool value)
{
  indigo_result status;

  if( !started_ ) {

    INDIGO_ERROR("%s(): Client not started\n",
        __func__);

    status = INDIGO_FAILED;
  }
  else {

    status =
        indigo_change_switch_property_1(&indigo_client_,
            deviceName,
            propertyName,
            itemName,
            value);

    if( status != INDIGO_OK ) {

      INDIGO_ERROR("%s(): indigo_change_switch_property_1() fails: status = %d\n",
          __func__,
          status);

    }
  }

  return status;
}

indigo_result QIndigoClient::change_number_property(const char * deviceName,
    const char * propertyName,
    const char * itemName,
    double value)
{
  indigo_result status;

  if( !started_ ) {

    INDIGO_ERROR("%s(): Client not started\n",
        __func__);

    status =
        INDIGO_FAILED;
  }
  else {

    status =
        indigo_change_number_property_1(&indigo_client_,
            deviceName,
            propertyName,
            itemName,
            value);

    if( status != INDIGO_OK ) {
      INDIGO_ERROR("%s(): indigo_change_number_property_1(%s:%s = %g) fails: status = %d\n",
          __func__,
          propertyName,
          itemName,
          value,
          status);
    }
  }

  return status;

}

indigo_result QIndigoClient::change_number_property(const char * deviceName,
    const char * propertyName,
    int count,
    const char * items[],
    const double values[])
{
  indigo_result status;

  if( !started_ ) {

    INDIGO_ERROR("%s(): Client not started\n",
        __func__);

    status =
        INDIGO_FAILED;
  }
  else {
    status =
        indigo_change_number_property(&indigo_client_,
            deviceName,
            propertyName,
            count,
            items,
            values);
  }

  return status;
}

