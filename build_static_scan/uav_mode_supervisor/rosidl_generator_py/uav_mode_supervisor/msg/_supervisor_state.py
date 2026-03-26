# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uav_mode_supervisor:msg/SupervisorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SupervisorState(type):
    """Metaclass of message 'SupervisorState'."""

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
            module = import_type_support('uav_mode_supervisor')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uav_mode_supervisor.msg.SupervisorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__supervisor_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__supervisor_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__supervisor_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__supervisor_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__supervisor_state

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


class SupervisorState(metaclass=Metaclass_SupervisorState):
    """Message class 'SupervisorState'."""

    __slots__ = [
        '_header',
        '_owner',
        '_active_tracking_height_m',
        '_last_command',
        '_pending_command',
        '_command_in_progress',
        '_last_message',
        '_fusion_diagnostics_seen',
        '_fusion_initialized',
        '_fusion_relocalize_requested',
        '_fusion_ready',
        '_fusion_reason',
        '_visual_state_seen',
        '_visual_active',
        '_visual_target_detected',
        '_visual_phase',
        '_visual_committed',
        '_visual_capture_observed',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'owner': 'string',
        'active_tracking_height_m': 'float',
        'last_command': 'string',
        'pending_command': 'string',
        'command_in_progress': 'boolean',
        'last_message': 'string',
        'fusion_diagnostics_seen': 'boolean',
        'fusion_initialized': 'boolean',
        'fusion_relocalize_requested': 'boolean',
        'fusion_ready': 'boolean',
        'fusion_reason': 'string',
        'visual_state_seen': 'boolean',
        'visual_active': 'boolean',
        'visual_target_detected': 'boolean',
        'visual_phase': 'string',
        'visual_committed': 'boolean',
        'visual_capture_observed': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.owner = kwargs.get('owner', str())
        self.active_tracking_height_m = kwargs.get('active_tracking_height_m', float())
        self.last_command = kwargs.get('last_command', str())
        self.pending_command = kwargs.get('pending_command', str())
        self.command_in_progress = kwargs.get('command_in_progress', bool())
        self.last_message = kwargs.get('last_message', str())
        self.fusion_diagnostics_seen = kwargs.get('fusion_diagnostics_seen', bool())
        self.fusion_initialized = kwargs.get('fusion_initialized', bool())
        self.fusion_relocalize_requested = kwargs.get('fusion_relocalize_requested', bool())
        self.fusion_ready = kwargs.get('fusion_ready', bool())
        self.fusion_reason = kwargs.get('fusion_reason', str())
        self.visual_state_seen = kwargs.get('visual_state_seen', bool())
        self.visual_active = kwargs.get('visual_active', bool())
        self.visual_target_detected = kwargs.get('visual_target_detected', bool())
        self.visual_phase = kwargs.get('visual_phase', str())
        self.visual_committed = kwargs.get('visual_committed', bool())
        self.visual_capture_observed = kwargs.get('visual_capture_observed', bool())

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
        if self.owner != other.owner:
            return False
        if self.active_tracking_height_m != other.active_tracking_height_m:
            return False
        if self.last_command != other.last_command:
            return False
        if self.pending_command != other.pending_command:
            return False
        if self.command_in_progress != other.command_in_progress:
            return False
        if self.last_message != other.last_message:
            return False
        if self.fusion_diagnostics_seen != other.fusion_diagnostics_seen:
            return False
        if self.fusion_initialized != other.fusion_initialized:
            return False
        if self.fusion_relocalize_requested != other.fusion_relocalize_requested:
            return False
        if self.fusion_ready != other.fusion_ready:
            return False
        if self.fusion_reason != other.fusion_reason:
            return False
        if self.visual_state_seen != other.visual_state_seen:
            return False
        if self.visual_active != other.visual_active:
            return False
        if self.visual_target_detected != other.visual_target_detected:
            return False
        if self.visual_phase != other.visual_phase:
            return False
        if self.visual_committed != other.visual_committed:
            return False
        if self.visual_capture_observed != other.visual_capture_observed:
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
    def owner(self):
        """Message field 'owner'."""
        return self._owner

    @owner.setter
    def owner(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'owner' field must be of type 'str'"
        self._owner = value

    @builtins.property
    def active_tracking_height_m(self):
        """Message field 'active_tracking_height_m'."""
        return self._active_tracking_height_m

    @active_tracking_height_m.setter
    def active_tracking_height_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'active_tracking_height_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'active_tracking_height_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._active_tracking_height_m = value

    @builtins.property
    def last_command(self):
        """Message field 'last_command'."""
        return self._last_command

    @last_command.setter
    def last_command(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'last_command' field must be of type 'str'"
        self._last_command = value

    @builtins.property
    def pending_command(self):
        """Message field 'pending_command'."""
        return self._pending_command

    @pending_command.setter
    def pending_command(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'pending_command' field must be of type 'str'"
        self._pending_command = value

    @builtins.property
    def command_in_progress(self):
        """Message field 'command_in_progress'."""
        return self._command_in_progress

    @command_in_progress.setter
    def command_in_progress(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'command_in_progress' field must be of type 'bool'"
        self._command_in_progress = value

    @builtins.property
    def last_message(self):
        """Message field 'last_message'."""
        return self._last_message

    @last_message.setter
    def last_message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'last_message' field must be of type 'str'"
        self._last_message = value

    @builtins.property
    def fusion_diagnostics_seen(self):
        """Message field 'fusion_diagnostics_seen'."""
        return self._fusion_diagnostics_seen

    @fusion_diagnostics_seen.setter
    def fusion_diagnostics_seen(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fusion_diagnostics_seen' field must be of type 'bool'"
        self._fusion_diagnostics_seen = value

    @builtins.property
    def fusion_initialized(self):
        """Message field 'fusion_initialized'."""
        return self._fusion_initialized

    @fusion_initialized.setter
    def fusion_initialized(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fusion_initialized' field must be of type 'bool'"
        self._fusion_initialized = value

    @builtins.property
    def fusion_relocalize_requested(self):
        """Message field 'fusion_relocalize_requested'."""
        return self._fusion_relocalize_requested

    @fusion_relocalize_requested.setter
    def fusion_relocalize_requested(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fusion_relocalize_requested' field must be of type 'bool'"
        self._fusion_relocalize_requested = value

    @builtins.property
    def fusion_ready(self):
        """Message field 'fusion_ready'."""
        return self._fusion_ready

    @fusion_ready.setter
    def fusion_ready(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fusion_ready' field must be of type 'bool'"
        self._fusion_ready = value

    @builtins.property
    def fusion_reason(self):
        """Message field 'fusion_reason'."""
        return self._fusion_reason

    @fusion_reason.setter
    def fusion_reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'fusion_reason' field must be of type 'str'"
        self._fusion_reason = value

    @builtins.property
    def visual_state_seen(self):
        """Message field 'visual_state_seen'."""
        return self._visual_state_seen

    @visual_state_seen.setter
    def visual_state_seen(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'visual_state_seen' field must be of type 'bool'"
        self._visual_state_seen = value

    @builtins.property
    def visual_active(self):
        """Message field 'visual_active'."""
        return self._visual_active

    @visual_active.setter
    def visual_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'visual_active' field must be of type 'bool'"
        self._visual_active = value

    @builtins.property
    def visual_target_detected(self):
        """Message field 'visual_target_detected'."""
        return self._visual_target_detected

    @visual_target_detected.setter
    def visual_target_detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'visual_target_detected' field must be of type 'bool'"
        self._visual_target_detected = value

    @builtins.property
    def visual_phase(self):
        """Message field 'visual_phase'."""
        return self._visual_phase

    @visual_phase.setter
    def visual_phase(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'visual_phase' field must be of type 'str'"
        self._visual_phase = value

    @builtins.property
    def visual_committed(self):
        """Message field 'visual_committed'."""
        return self._visual_committed

    @visual_committed.setter
    def visual_committed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'visual_committed' field must be of type 'bool'"
        self._visual_committed = value

    @builtins.property
    def visual_capture_observed(self):
        """Message field 'visual_capture_observed'."""
        return self._visual_capture_observed

    @visual_capture_observed.setter
    def visual_capture_observed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'visual_capture_observed' field must be of type 'bool'"
        self._visual_capture_observed = value
