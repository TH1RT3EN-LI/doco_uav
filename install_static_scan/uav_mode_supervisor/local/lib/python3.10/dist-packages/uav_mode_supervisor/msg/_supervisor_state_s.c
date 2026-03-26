// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "uav_mode_supervisor/msg/detail/supervisor_state__struct.h"
#include "uav_mode_supervisor/msg/detail/supervisor_state__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool uav_mode_supervisor__msg__supervisor_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[58];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("uav_mode_supervisor.msg._supervisor_state.SupervisorState", full_classname_dest, 57) == 0);
  }
  uav_mode_supervisor__msg__SupervisorState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // owner
    PyObject * field = PyObject_GetAttrString(_pymsg, "owner");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->owner, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // active_tracking_height_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "active_tracking_height_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->active_tracking_height_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // last_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "last_command");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->last_command, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // pending_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "pending_command");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->pending_command, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // command_in_progress
    PyObject * field = PyObject_GetAttrString(_pymsg, "command_in_progress");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->command_in_progress = (Py_True == field);
    Py_DECREF(field);
  }
  {  // last_message
    PyObject * field = PyObject_GetAttrString(_pymsg, "last_message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->last_message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // fusion_diagnostics_seen
    PyObject * field = PyObject_GetAttrString(_pymsg, "fusion_diagnostics_seen");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fusion_diagnostics_seen = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fusion_initialized
    PyObject * field = PyObject_GetAttrString(_pymsg, "fusion_initialized");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fusion_initialized = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fusion_relocalize_requested
    PyObject * field = PyObject_GetAttrString(_pymsg, "fusion_relocalize_requested");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fusion_relocalize_requested = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fusion_ready
    PyObject * field = PyObject_GetAttrString(_pymsg, "fusion_ready");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fusion_ready = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fusion_reason
    PyObject * field = PyObject_GetAttrString(_pymsg, "fusion_reason");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->fusion_reason, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // visual_state_seen
    PyObject * field = PyObject_GetAttrString(_pymsg, "visual_state_seen");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->visual_state_seen = (Py_True == field);
    Py_DECREF(field);
  }
  {  // visual_active
    PyObject * field = PyObject_GetAttrString(_pymsg, "visual_active");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->visual_active = (Py_True == field);
    Py_DECREF(field);
  }
  {  // visual_target_detected
    PyObject * field = PyObject_GetAttrString(_pymsg, "visual_target_detected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->visual_target_detected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // visual_phase
    PyObject * field = PyObject_GetAttrString(_pymsg, "visual_phase");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->visual_phase, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // visual_committed
    PyObject * field = PyObject_GetAttrString(_pymsg, "visual_committed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->visual_committed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // visual_capture_observed
    PyObject * field = PyObject_GetAttrString(_pymsg, "visual_capture_observed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->visual_capture_observed = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * uav_mode_supervisor__msg__supervisor_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SupervisorState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("uav_mode_supervisor.msg._supervisor_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SupervisorState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  uav_mode_supervisor__msg__SupervisorState * ros_message = (uav_mode_supervisor__msg__SupervisorState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // owner
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->owner.data,
      strlen(ros_message->owner.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "owner", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // active_tracking_height_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->active_tracking_height_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "active_tracking_height_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // last_command
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->last_command.data,
      strlen(ros_message->last_command.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "last_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pending_command
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->pending_command.data,
      strlen(ros_message->pending_command.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pending_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // command_in_progress
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->command_in_progress ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "command_in_progress", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // last_message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->last_message.data,
      strlen(ros_message->last_message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "last_message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fusion_diagnostics_seen
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fusion_diagnostics_seen ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fusion_diagnostics_seen", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fusion_initialized
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fusion_initialized ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fusion_initialized", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fusion_relocalize_requested
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fusion_relocalize_requested ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fusion_relocalize_requested", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fusion_ready
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fusion_ready ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fusion_ready", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fusion_reason
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->fusion_reason.data,
      strlen(ros_message->fusion_reason.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "fusion_reason", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // visual_state_seen
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->visual_state_seen ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "visual_state_seen", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // visual_active
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->visual_active ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "visual_active", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // visual_target_detected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->visual_target_detected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "visual_target_detected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // visual_phase
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->visual_phase.data,
      strlen(ros_message->visual_phase.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "visual_phase", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // visual_committed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->visual_committed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "visual_committed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // visual_capture_observed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->visual_capture_observed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "visual_capture_observed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
