# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dracon_msgs:msg/JobAssignment.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_JobAssignment(type):
    """Metaclass of message 'JobAssignment'."""

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
            module = import_type_support('dracon_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dracon_msgs.msg.JobAssignment')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__job_assignment
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__job_assignment
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__job_assignment
            cls._TYPE_SUPPORT = module.type_support_msg__msg__job_assignment
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__job_assignment

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class JobAssignment(metaclass=Metaclass_JobAssignment):
    """Message class 'JobAssignment'."""

    __slots__ = [
        '_agent_id',
        '_job_id',
        '_job_x',
        '_job_y',
    ]

    _fields_and_field_types = {
        'agent_id': 'string',
        'job_id': 'string',
        'job_x': 'double',
        'job_y': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.agent_id = kwargs.get('agent_id', str())
        self.job_id = kwargs.get('job_id', str())
        self.job_x = kwargs.get('job_x', float())
        self.job_y = kwargs.get('job_y', float())

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
        if self.agent_id != other.agent_id:
            return False
        if self.job_id != other.job_id:
            return False
        if self.job_x != other.job_x:
            return False
        if self.job_y != other.job_y:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def agent_id(self):
        """Message field 'agent_id'."""
        return self._agent_id

    @agent_id.setter
    def agent_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'agent_id' field must be of type 'str'"
        self._agent_id = value

    @builtins.property
    def job_id(self):
        """Message field 'job_id'."""
        return self._job_id

    @job_id.setter
    def job_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'job_id' field must be of type 'str'"
        self._job_id = value

    @builtins.property
    def job_x(self):
        """Message field 'job_x'."""
        return self._job_x

    @job_x.setter
    def job_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'job_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'job_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._job_x = value

    @builtins.property
    def job_y(self):
        """Message field 'job_y'."""
        return self._job_y

    @job_y.setter
    def job_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'job_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'job_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._job_y = value
