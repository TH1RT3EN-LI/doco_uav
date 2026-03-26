// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
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
#include "uav_visual_landing/msg/detail/landing_controller_state__struct.h"
#include "uav_visual_landing/msg/detail/landing_controller_state__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool uav_visual_landing__msg__landing_controller_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[72];
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
    assert(strncmp("uav_visual_landing.msg._landing_controller_state.LandingControllerState", full_classname_dest, 71) == 0);
  }
  uav_visual_landing__msg__LandingControllerState * ros_message = _ros_message;
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
  {  // active
    PyObject * field = PyObject_GetAttrString(_pymsg, "active");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->active = (Py_True == field);
    Py_DECREF(field);
  }
  {  // phase
    PyObject * field = PyObject_GetAttrString(_pymsg, "phase");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->phase, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // target_detected
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_detected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->target_detected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // observation_age_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "observation_age_s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->observation_age_s = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // height_source
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_source");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->height_source, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // terminal_trigger_source
    PyObject * field = PyObject_GetAttrString(_pymsg, "terminal_trigger_source");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->terminal_trigger_source, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // odom_height_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "odom_height_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->odom_height_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // height_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->height_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // height_measurement_source
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_measurement_source");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->height_measurement_source, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // height_measurement_fresh
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_measurement_fresh");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->height_measurement_fresh = (Py_True == field);
    Py_DECREF(field);
  }
  {  // raw_flow_fresh
    PyObject * field = PyObject_GetAttrString(_pymsg, "raw_flow_fresh");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->raw_flow_fresh = (Py_True == field);
    Py_DECREF(field);
  }
  {  // height_measurement_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_measurement_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->height_measurement_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // control_height_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "control_height_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->control_height_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tag_depth_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "tag_depth_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->tag_depth_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // tag_depth_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "tag_depth_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tag_depth_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // align_enter_lateral_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "align_enter_lateral_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->align_enter_lateral_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // align_exit_lateral_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "align_exit_lateral_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->align_exit_lateral_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // active_max_vxy
    PyObject * field = PyObject_GetAttrString(_pymsg, "active_max_vxy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->active_max_vxy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_u_norm_filtered
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_u_norm_filtered");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_u_norm_filtered = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_v_norm_filtered
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_v_norm_filtered");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_v_norm_filtered = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_u_rate_norm_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_u_rate_norm_s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_u_rate_norm_s = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_v_rate_norm_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_v_rate_norm_s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_v_rate_norm_s = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lateral_error_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "lateral_error_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->lateral_error_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // lateral_error_x_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "lateral_error_x_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lateral_error_x_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lateral_error_y_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "lateral_error_y_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lateral_error_y_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lateral_error_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "lateral_error_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lateral_error_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lateral_error_rate_x_mps
    PyObject * field = PyObject_GetAttrString(_pymsg, "lateral_error_rate_x_mps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lateral_error_rate_x_mps = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lateral_error_rate_y_mps
    PyObject * field = PyObject_GetAttrString(_pymsg, "lateral_error_rate_y_mps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lateral_error_rate_y_mps = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_target_height_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_target_height_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_target_height_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_error_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_error_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_error_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // xy_control_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "xy_control_mode");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->xy_control_mode, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // cmd_vx
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_vx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cmd_vx = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cmd_vy
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_vy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cmd_vy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cmd_vz
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_vz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cmd_vz = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cmd_yaw_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_yaw_rate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cmd_yaw_rate = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * uav_visual_landing__msg__landing_controller_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LandingControllerState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("uav_visual_landing.msg._landing_controller_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LandingControllerState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  uav_visual_landing__msg__LandingControllerState * ros_message = (uav_visual_landing__msg__LandingControllerState *)raw_ros_message;
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
  {  // active
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->active ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "active", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // phase
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->phase.data,
      strlen(ros_message->phase.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "phase", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_detected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->target_detected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_detected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // observation_age_s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->observation_age_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "observation_age_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_source
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->height_source.data,
      strlen(ros_message->height_source.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_source", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // terminal_trigger_source
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->terminal_trigger_source.data,
      strlen(ros_message->terminal_trigger_source.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "terminal_trigger_source", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // odom_height_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->odom_height_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "odom_height_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->height_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_measurement_source
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->height_measurement_source.data,
      strlen(ros_message->height_measurement_source.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_measurement_source", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_measurement_fresh
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->height_measurement_fresh ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_measurement_fresh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // raw_flow_fresh
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->raw_flow_fresh ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "raw_flow_fresh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_measurement_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->height_measurement_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_measurement_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // control_height_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->control_height_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "control_height_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tag_depth_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->tag_depth_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tag_depth_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tag_depth_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tag_depth_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tag_depth_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // align_enter_lateral_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->align_enter_lateral_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "align_enter_lateral_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // align_exit_lateral_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->align_exit_lateral_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "align_exit_lateral_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // active_max_vxy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->active_max_vxy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "active_max_vxy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_u_norm_filtered
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_u_norm_filtered);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_u_norm_filtered", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_v_norm_filtered
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_v_norm_filtered);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_v_norm_filtered", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_u_rate_norm_s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_u_rate_norm_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_u_rate_norm_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_v_rate_norm_s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_v_rate_norm_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_v_rate_norm_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lateral_error_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->lateral_error_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lateral_error_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lateral_error_x_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lateral_error_x_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lateral_error_x_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lateral_error_y_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lateral_error_y_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lateral_error_y_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lateral_error_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lateral_error_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lateral_error_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lateral_error_rate_x_mps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lateral_error_rate_x_mps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lateral_error_rate_x_mps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lateral_error_rate_y_mps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lateral_error_rate_y_mps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lateral_error_rate_y_mps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_target_height_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_target_height_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_target_height_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_error_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_error_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_error_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // xy_control_mode
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->xy_control_mode.data,
      strlen(ros_message->xy_control_mode.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "xy_control_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cmd_vx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cmd_vx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_vx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cmd_vy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cmd_vy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_vy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cmd_vz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cmd_vz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_vz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cmd_yaw_rate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cmd_yaw_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_yaw_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
