# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uav_mode_supervisor:srv/CommandSupervisor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CommandSupervisor_Request(type):
    """Metaclass of message 'CommandSupervisor_Request'."""

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
                'uav_mode_supervisor.srv.CommandSupervisor_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__command_supervisor__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__command_supervisor__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__command_supervisor__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__command_supervisor__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__command_supervisor__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommandSupervisor_Request(metaclass=Metaclass_CommandSupervisor_Request):
    """Message class 'CommandSupervisor_Request'."""

    __slots__ = [
        '_command',
        '_tracking_target_height_m',
    ]

    _fields_and_field_types = {
        'command': 'string',
        'tracking_target_height_m': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.command = kwargs.get('command', str())
        self.tracking_target_height_m = kwargs.get('tracking_target_height_m', float())

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
        if self.command != other.command:
            return False
        if self.tracking_target_height_m != other.tracking_target_height_m:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'command' field must be of type 'str'"
        self._command = value

    @builtins.property
    def tracking_target_height_m(self):
        """Message field 'tracking_target_height_m'."""
        return self._tracking_target_height_m

    @tracking_target_height_m.setter
    def tracking_target_height_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tracking_target_height_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tracking_target_height_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tracking_target_height_m = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CommandSupervisor_Response(type):
    """Metaclass of message 'CommandSupervisor_Response'."""

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
                'uav_mode_supervisor.srv.CommandSupervisor_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__command_supervisor__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__command_supervisor__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__command_supervisor__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__command_supervisor__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__command_supervisor__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommandSupervisor_Response(metaclass=Metaclass_CommandSupervisor_Response):
    """Message class 'CommandSupervisor_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_CommandSupervisor(type):
    """Metaclass of service 'CommandSupervisor'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uav_mode_supervisor')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uav_mode_supervisor.srv.CommandSupervisor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__command_supervisor

            from uav_mode_supervisor.srv import _command_supervisor
            if _command_supervisor.Metaclass_CommandSupervisor_Request._TYPE_SUPPORT is None:
                _command_supervisor.Metaclass_CommandSupervisor_Request.__import_type_support__()
            if _command_supervisor.Metaclass_CommandSupervisor_Response._TYPE_SUPPORT is None:
                _command_supervisor.Metaclass_CommandSupervisor_Response.__import_type_support__()


class CommandSupervisor(metaclass=Metaclass_CommandSupervisor):
    from uav_mode_supervisor.srv._command_supervisor import CommandSupervisor_Request as Request
    from uav_mode_supervisor.srv._command_supervisor import CommandSupervisor_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
