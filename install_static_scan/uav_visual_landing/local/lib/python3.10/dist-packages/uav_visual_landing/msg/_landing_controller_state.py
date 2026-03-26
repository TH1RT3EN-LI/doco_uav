# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uav_visual_landing:msg/LandingControllerState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LandingControllerState(type):
    """Metaclass of message 'LandingControllerState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uav_visual_landing')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uav_visual_landing.msg.LandingControllerState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__landing_controller_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__landing_controller_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__landing_controller_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__landing_controller_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__landing_controller_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LandingControllerState(metaclass=Metaclass_LandingControllerState):
    """Message class 'LandingControllerState'."""

    __slots__ = [
        '_header',
        '_active',
        '_phase',
        '_target_detected',
        '_observation_age_s',
        '_target_confidence',
        '_height_source',
        '_terminal_trigger_source',
        '_odom_height_m',
        '_height_valid',
        '_height_measurement_source',
        '_height_measurement_fresh',
        '_raw_flow_fresh',
        '_height_measurement_m',
        '_control_height_m',
        '_tag_depth_valid',
        '_tag_depth_m',
        '_align_enter_lateral_m',
        '_align_exit_lateral_m',
        '_active_max_vxy',
        '_err_u_norm_filtered',
        '_err_v_norm_filtered',
        '_err_u_rate_norm_s',
        '_err_v_rate_norm_s',
        '_lateral_error_valid',
        '_lateral_error_x_m',
        '_lateral_error_y_m',
        '_lateral_error_m',
        '_lateral_error_rate_x_mps',
        '_lateral_error_rate_y_mps',
        '_z_target_height_m',
        '_z_error_m',
        '_xy_control_mode',
        '_cmd_vx',
        '_cmd_vy',
        '_cmd_vz',
        '_cmd_yaw_rate',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'active': 'boolean',
        'phase': 'string',
        'target_detected': 'boolean',
        'observation_age_s': 'float',
        'target_confidence': 'float',
        'height_source': 'string',
        'terminal_trigger_source': 'string',
        'odom_height_m': 'float',
        'height_valid': 'boolean',
        'height_measurement_source': 'string',
        'height_measurement_fresh': 'boolean',
        'raw_flow_fresh': 'boolean',
        'height_measurement_m': 'float',
        'control_height_m': 'float',
        'tag_depth_valid': 'boolean',
        'tag_depth_m': 'float',
        'align_enter_lateral_m': 'float',
        'align_exit_lateral_m': 'float',
        'active_max_vxy': 'float',
        'err_u_norm_filtered': 'float',
        'err_v_norm_filtered': 'float',
        'err_u_rate_norm_s': 'float',
        'err_v_rate_norm_s': 'float',
        'lateral_error_valid': 'boolean',
        'lateral_error_x_m': 'float',
        'lateral_error_y_m': 'float',
        'lateral_error_m': 'float',
        'lateral_error_rate_x_mps': 'float',
        'lateral_error_rate_y_mps': 'float',
        'z_target_height_m': 'float',
        'z_error_m': 'float',
        'xy_control_mode': 'string',
        'cmd_vx': 'float',
        'cmd_vy': 'float',
        'cmd_vz': 'float',
        'cmd_yaw_rate': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.active = kwargs.get('active', bool())
        self.phase = kwargs.get('phase', str())
        self.target_detected = kwargs.get('target_detected', bool())
        self.observation_age_s = kwargs.get('observation_age_s', float())
        self.target_confidence = kwargs.get('target_confidence', float())
        self.height_source = kwargs.get('height_source', str())
        self.terminal_trigger_source = kwargs.get('terminal_trigger_source', str())
        self.odom_height_m = kwargs.get('odom_height_m', float())
        self.height_valid = kwargs.get('height_valid', bool())
        self.height_measurement_source = kwargs.get('height_measurement_source', str())
        self.height_measurement_fresh = kwargs.get('height_measurement_fresh', bool())
        self.raw_flow_fresh = kwargs.get('raw_flow_fresh', bool())
        self.height_measurement_m = kwargs.get('height_measurement_m', float())
        self.control_height_m = kwargs.get('control_height_m', float())
        self.tag_depth_valid = kwargs.get('tag_depth_valid', bool())
        self.tag_depth_m = kwargs.get('tag_depth_m', float())
        self.align_enter_lateral_m = kwargs.get('align_enter_lateral_m', float())
        self.align_exit_lateral_m = kwargs.get('align_exit_lateral_m', float())
        self.active_max_vxy = kwargs.get('active_max_vxy', float())
        self.err_u_norm_filtered = kwargs.get('err_u_norm_filtered', float())
        self.err_v_norm_filtered = kwargs.get('err_v_norm_filtered', float())
        self.err_u_rate_norm_s = kwargs.get('err_u_rate_norm_s', float())
        self.err_v_rate_norm_s = kwargs.get('err_v_rate_norm_s', float())
        self.lateral_error_valid = kwargs.get('lateral_error_valid', bool())
        self.lateral_error_x_m = kwargs.get('lateral_error_x_m', float())
        self.lateral_error_y_m = kwargs.get('lateral_error_y_m', float())
        self.lateral_error_m = kwargs.get('lateral_error_m', float())
        self.lateral_error_rate_x_mps = kwargs.get('lateral_error_rate_x_mps', float())
        self.lateral_error_rate_y_mps = kwargs.get('lateral_error_rate_y_mps', float())
        self.z_target_height_m = kwargs.get('z_target_height_m', float())
        self.z_error_m = kwargs.get('z_error_m', float())
        self.xy_control_mode = kwargs.get('xy_control_mode', str())
        self.cmd_vx = kwargs.get('cmd_vx', float())
        self.cmd_vy = kwargs.get('cmd_vy', float())
        self.cmd_vz = kwargs.get('cmd_vz', float())
        self.cmd_yaw_rate = kwargs.get('cmd_yaw_rate', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.active != other.active:
            return False
        if self.phase != other.phase:
            return False
        if self.target_detected != other.target_detected:
            return False
        if self.observation_age_s != other.observation_age_s:
            return False
        if self.target_confidence != other.target_confidence:
            return False
        if self.height_source != other.height_source:
            return False
        if self.terminal_trigger_source != other.terminal_trigger_source:
            return False
        if self.odom_height_m != other.odom_height_m:
            return False
        if self.height_valid != other.height_valid:
            return False
        if self.height_measurement_source != other.height_measurement_source:
            return False
        if self.height_measurement_fresh != other.height_measurement_fresh:
            return False
        if self.raw_flow_fresh != other.raw_flow_fresh:
            return False
        if self.height_measurement_m != other.height_measurement_m:
            return False
        if self.control_height_m != other.control_height_m:
            return False
        if self.tag_depth_valid != other.tag_depth_valid:
            return False
        if self.tag_depth_m != other.tag_depth_m:
            return False
        if self.align_enter_lateral_m != other.align_enter_lateral_m:
            return False
        if self.align_exit_lateral_m != other.align_exit_lateral_m:
            return False
        if self.active_max_vxy != other.active_max_vxy:
            return False
        if self.err_u_norm_filtered != other.err_u_norm_filtered:
            return False
        if self.err_v_norm_filtered != other.err_v_norm_filtered:
            return False
        if self.err_u_rate_norm_s != other.err_u_rate_norm_s:
            return False
        if self.err_v_rate_norm_s != other.err_v_rate_norm_s:
            return False
        if self.lateral_error_valid != other.lateral_error_valid:
            return False
        if self.lateral_error_x_m != other.lateral_error_x_m:
            return False
        if self.lateral_error_y_m != other.lateral_error_y_m:
            return False
        if self.lateral_error_m != other.lateral_error_m:
            return False
        if self.lateral_error_rate_x_mps != other.lateral_error_rate_x_mps:
            return False
        if self.lateral_error_rate_y_mps != other.lateral_error_rate_y_mps:
            return False
        if self.z_target_height_m != other.z_target_height_m:
            return False
        if self.z_error_m != other.z_error_m:
            return False
        if self.xy_control_mode != other.xy_control_mode:
            return False
        if self.cmd_vx != other.cmd_vx:
            return False
        if self.cmd_vy != other.cmd_vy:
            return False
        if self.cmd_vz != other.cmd_vz:
            return False
        if self.cmd_yaw_rate != other.cmd_yaw_rate:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def active(self):
        """Message field 'active'."""
        return self._active

    @active.setter
    def active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'active' field must be of type 'bool'"
        self._active = value

    @builtins.property
    def phase(self):
        """Message field 'phase'."""
        return self._phase

    @phase.setter
    def phase(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'phase' field must be of type 'str'"
        self._phase = value

    @builtins.property
    def target_detected(self):
        """Message field 'target_detected'."""
        return self._target_detected

    @target_detected.setter
    def target_detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'target_detected' field must be of type 'bool'"
        self._target_detected = value

    @builtins.property
    def observation_age_s(self):
        """Message field 'observation_age_s'."""
        return self._observation_age_s

    @observation_age_s.setter
    def observation_age_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'observation_age_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'observation_age_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._observation_age_s = value

    @builtins.property
    def target_confidence(self):
        """Message field 'target_confidence'."""
        return self._target_confidence

    @target_confidence.setter
    def target_confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_confidence = value

    @builtins.property
    def height_source(self):
        """Message field 'height_source'."""
        return self._height_source

    @height_source.setter
    def height_source(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'height_source' field must be of type 'str'"
        self._height_source = value

    @builtins.property
    def terminal_trigger_source(self):
        """Message field 'terminal_trigger_source'."""
        return self._terminal_trigger_source

    @terminal_trigger_source.setter
    def terminal_trigger_source(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'terminal_trigger_source' field must be of type 'str'"
        self._terminal_trigger_source = value

    @builtins.property
    def odom_height_m(self):
        """Message field 'odom_height_m'."""
        return self._odom_height_m

    @odom_height_m.setter
    def odom_height_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'odom_height_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'odom_height_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._odom_height_m = value

    @builtins.property
    def height_valid(self):
        """Message field 'height_valid'."""
        return self._height_valid

    @height_valid.setter
    def height_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'height_valid' field must be of type 'bool'"
        self._height_valid = value

    @builtins.property
    def height_measurement_source(self):
        """Message field 'height_measurement_source'."""
        return self._height_measurement_source

    @height_measurement_source.setter
    def height_measurement_source(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'height_measurement_source' field must be of type 'str'"
        self._height_measurement_source = value

    @builtins.property
    def height_measurement_fresh(self):
        """Message field 'height_measurement_fresh'."""
        return self._height_measurement_fresh

    @height_measurement_fresh.setter
    def height_measurement_fresh(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'height_measurement_fresh' field must be of type 'bool'"
        self._height_measurement_fresh = value

    @builtins.property
    def raw_flow_fresh(self):
        """Message field 'raw_flow_fresh'."""
        return self._raw_flow_fresh

    @raw_flow_fresh.setter
    def raw_flow_fresh(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'raw_flow_fresh' field must be of type 'bool'"
        self._raw_flow_fresh = value

    @builtins.property
    def height_measurement_m(self):
        """Message field 'height_measurement_m'."""
        return self._height_measurement_m

    @height_measurement_m.setter
    def height_measurement_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'height_measurement_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'height_measurement_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._height_measurement_m = value

    @builtins.property
    def control_height_m(self):
        """Message field 'control_height_m'."""
        return self._control_height_m

    @control_height_m.setter
    def control_height_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'control_height_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'control_height_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._control_height_m = value

    @builtins.property
    def tag_depth_valid(self):
        """Message field 'tag_depth_valid'."""
        return self._tag_depth_valid

    @tag_depth_valid.setter
    def tag_depth_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'tag_depth_valid' field must be of type 'bool'"
        self._tag_depth_valid = value

    @builtins.property
    def tag_depth_m(self):
        """Message field 'tag_depth_m'."""
        return self._tag_depth_m

    @tag_depth_m.setter
    def tag_depth_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tag_depth_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tag_depth_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tag_depth_m = value

    @builtins.property
    def align_enter_lateral_m(self):
        """Message field 'align_enter_lateral_m'."""
        return self._align_enter_lateral_m

    @align_enter_lateral_m.setter
    def align_enter_lateral_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'align_enter_lateral_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'align_enter_lateral_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._align_enter_lateral_m = value

    @builtins.property
    def align_exit_lateral_m(self):
        """Message field 'align_exit_lateral_m'."""
        return self._align_exit_lateral_m

    @align_exit_lateral_m.setter
    def align_exit_lateral_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'align_exit_lateral_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'align_exit_lateral_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._align_exit_lateral_m = value

    @builtins.property
    def active_max_vxy(self):
        """Message field 'active_max_vxy'."""
        return self._active_max_vxy

    @active_max_vxy.setter
    def active_max_vxy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'active_max_vxy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'active_max_vxy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._active_max_vxy = value

    @builtins.property
    def err_u_norm_filtered(self):
        """Message field 'err_u_norm_filtered'."""
        return self._err_u_norm_filtered

    @err_u_norm_filtered.setter
    def err_u_norm_filtered(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_u_norm_filtered' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'err_u_norm_filtered' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._err_u_norm_filtered = value

    @builtins.property
    def err_v_norm_filtered(self):
        """Message field 'err_v_norm_filtered'."""
        return self._err_v_norm_filtered

    @err_v_norm_filtered.setter
    def err_v_norm_filtered(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_v_norm_filtered' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'err_v_norm_filtered' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._err_v_norm_filtered = value

    @builtins.property
    def err_u_rate_norm_s(self):
        """Message field 'err_u_rate_norm_s'."""
        return self._err_u_rate_norm_s

    @err_u_rate_norm_s.setter
    def err_u_rate_norm_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_u_rate_norm_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'err_u_rate_norm_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._err_u_rate_norm_s = value

    @builtins.property
    def err_v_rate_norm_s(self):
        """Message field 'err_v_rate_norm_s'."""
        return self._err_v_rate_norm_s

    @err_v_rate_norm_s.setter
    def err_v_rate_norm_s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_v_rate_norm_s' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'err_v_rate_norm_s' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._err_v_rate_norm_s = value

    @builtins.property
    def lateral_error_valid(self):
        """Message field 'lateral_error_valid'."""
        return self._lateral_error_valid

    @lateral_error_valid.setter
    def lateral_error_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'lateral_error_valid' field must be of type 'bool'"
        self._lateral_error_valid = value

    @builtins.property
    def lateral_error_x_m(self):
        """Message field 'lateral_error_x_m'."""
        return self._lateral_error_x_m

    @lateral_error_x_m.setter
    def lateral_error_x_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lateral_error_x_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lateral_error_x_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lateral_error_x_m = value

    @builtins.property
    def lateral_error_y_m(self):
        """Message field 'lateral_error_y_m'."""
        return self._lateral_error_y_m

    @lateral_error_y_m.setter
    def lateral_error_y_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lateral_error_y_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lateral_error_y_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lateral_error_y_m = value

    @builtins.property
    def lateral_error_m(self):
        """Message field 'lateral_error_m'."""
        return self._lateral_error_m

    @lateral_error_m.setter
    def lateral_error_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lateral_error_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lateral_error_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lateral_error_m = value

    @builtins.property
    def lateral_error_rate_x_mps(self):
        """Message field 'lateral_error_rate_x_mps'."""
        return self._lateral_error_rate_x_mps

    @lateral_error_rate_x_mps.setter
    def lateral_error_rate_x_mps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lateral_error_rate_x_mps' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lateral_error_rate_x_mps' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lateral_error_rate_x_mps = value

    @builtins.property
    def lateral_error_rate_y_mps(self):
        """Message field 'lateral_error_rate_y_mps'."""
        return self._lateral_error_rate_y_mps

    @lateral_error_rate_y_mps.setter
    def lateral_error_rate_y_mps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lateral_error_rate_y_mps' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lateral_error_rate_y_mps' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lateral_error_rate_y_mps = value

    @builtins.property
    def z_target_height_m(self):
        """Message field 'z_target_height_m'."""
        return self._z_target_height_m

    @z_target_height_m.setter
    def z_target_height_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_target_height_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_target_height_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_target_height_m = value

    @builtins.property
    def z_error_m(self):
        """Message field 'z_error_m'."""
        return self._z_error_m

    @z_error_m.setter
    def z_error_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_error_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_error_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_error_m = value

    @builtins.property
    def xy_control_mode(self):
        """Message field 'xy_control_mode'."""
        return self._xy_control_mode

    @xy_control_mode.setter
    def xy_control_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'xy_control_mode' field must be of type 'str'"
        self._xy_control_mode = value

    @builtins.property
    def cmd_vx(self):
        """Message field 'cmd_vx'."""
        return self._cmd_vx

    @cmd_vx.setter
    def cmd_vx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_vx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_vx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_vx = value

    @builtins.property
    def cmd_vy(self):
        """Message field 'cmd_vy'."""
        return self._cmd_vy

    @cmd_vy.setter
    def cmd_vy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_vy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_vy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_vy = value

    @builtins.property
    def cmd_vz(self):
        """Message field 'cmd_vz'."""
        return self._cmd_vz

    @cmd_vz.setter
    def cmd_vz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_vz' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_vz' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_vz = value

    @builtins.property
    def cmd_yaw_rate(self):
        """Message field 'cmd_yaw_rate'."""
        return self._cmd_yaw_rate

    @cmd_yaw_rate.setter
    def cmd_yaw_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_yaw_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_yaw_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_yaw_rate = value
