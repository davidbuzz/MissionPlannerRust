# What IronPython gives a Mission Planner script beyond `Script` and `cs`, for RustPython.
#
# `Script.cs:35-53`: Mission Planner loads every assembly it has into the engine
# (`engine.Runtime.LoadAssembly`), which is what makes `import clr`, `clr.AddReference(...)`,
# `import MissionPlanner`, `import MAVLink`, `from MissionPlanner.Utilities import Locationwp` and
# `from System import Byte` work, and puts `MainV2`, `FlightPlanner`, `FlightData`, `Ports`,
# `MAV`, `cs`, `Script`, `mavutil` and `Joystick` in the scope. There is no CLR here, so this
# module answers to the names the shipped scripts reach - the modules `clr`, `System`,
# `MissionPlanner` (with `Utilities`, `MainV2` and `Comms`) and `MAVLink`, and the objects `MAV`,
# `MainV2`, `FlightPlanner`, `FlightData`, `Ports` and `Joystick` - each member with the C#'s
# semantics, the link's side of it through the host (`crates/mp-script/src/api.rs`,
# `ScriptHost`). Anything else under those names raises "<name> is not available to scripts in
# this version", so a script stops at the member it reached for and says which.
#
# Run by `engine.rs` before every script, in a namespace of its own that already holds
# `ENUMS` (`mavlink_enums.py`) and the engine's `_link`, `_script` and `_cs`.

import builtins
import re
import struct
import sys
import time

_NOT_AVAILABLE = ' is not available to scripts in this version'


def _unavailable(name):
    return RuntimeError(name + _NOT_AVAILABLE)


_Module = type(sys)


class _Namespace(_Module):
    """A .NET namespace or static class as IronPython shows it: the members ported, and every
    other name raising with its full name."""

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        raise _unavailable(self.__name__ + '.' + name)


def _namespace(name, **members):
    module = _Namespace(name)
    for key, value in members.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


# --- System ---------------------------------------------------------------------------------

class TimeoutException(Exception):
    """`System.TimeoutException`, which every waiting `MAVLinkInterface` member throws when the
    vehicle stays silent: "Timeout on read - setWP"."""


TimeoutException.__module__ = 'System'


class InvalidOperationException(Exception):
    """`System.InvalidOperationException`: what a closed port's `Write` throws."""


InvalidOperationException.__module__ = 'System'


def _call(function, *args):
    # The host's timeouts arrive as Python's TimeoutError; a script sees the C#'s type.
    try:
        return function(*args)
    except TimeoutError as timeout:
        raise TimeoutException(str(timeout)) from None


class Byte(int):
    """`System.Byte`: 0 to 255, and the .NET message outside that."""

    def __new__(cls, value=0):
        value = int(value)
        if not 0 <= value <= 255:
            raise OverflowError('Value was either too large or too small for an unsigned byte.')
        return int.__new__(cls, value)


class _Delegate(object):
    """A `Func[...]` or `Action[...]` made from a Python callable: calling it calls that."""

    def __init__(self, kind, types, function):
        self._kind = kind
        self._types = types
        self._function = function

    def __call__(self, *args):
        return self._function(*args)

    def Invoke(self, *args):
        return self._function(*args)


class _GenericDelegate(object):
    """`System.Func` and `System.Action`: `Func[MAVLink.MAVLinkMessage, bool](callable)`."""

    def __init__(self, kind):
        self._kind = kind

    def __getitem__(self, types):
        kind = self._kind
        return lambda function: _Delegate(kind, types, function)


class _ArrayOf(object):
    def __init__(self, element):
        self._element = element

    def __call__(self, items):
        # `Array[Byte]([...])`, a `byte[]`: the one array type the shipped scripts make.
        if self._element is Byte:
            return bytearray(Byte(item) for item in items)
        raise _unavailable('System.Array[' + getattr(self._element, '__name__', '?') + ']')


class _Array(object):
    """`System.Array`: `Array[Byte](...)`."""

    def __getitem__(self, element):
        return _ArrayOf(element)


System = _namespace(
    'System',
    Byte=Byte,
    Func=_GenericDelegate('Func'),
    Action=_GenericDelegate('Action'),
    Array=_Array(),
    TimeoutException=TimeoutException,
    InvalidOperationException=InvalidOperationException,
)


# --- clr ------------------------------------------------------------------------------------

def _add_reference(*names):
    # `clr.AddReference`: Mission Planner has already loaded every assembly into the engine
    # (`Script.cs:40-44`), so a reference adds nothing a script can then import; any name is
    # accepted, where IronPython refuses one that is not in the application's folder.
    return None


def _clear_profiler_data():
    # `clr.ClearProfilerData`: IronPython's profiler is off, so there is nothing to clear.
    return None


clr = _namespace('clr', AddReference=_add_reference, ClearProfilerData=_clear_profiler_data)


# --- Value types --------------------------------------------------------------------------------

def _float32(value):
    # `(float)value`: a .NET float holds what a 32-bit float holds.
    return struct.unpack('<f', struct.pack('<f', float(value)))[0]


def _uint16(value):
    value = int(value)
    if not 0 <= value <= 65535:
        raise OverflowError('Value was either too large or too small for a UInt16.')
    return value


def _byte(value):
    return int(Byte(value))


def _double(value):
    return float(value)


def _object(value):
    return value


class _Field(object):
    """A public field of a .NET struct as IronPython shows it. Read from an instance, the value;
    read from the type (`Locationwp.lat`), the field itself, whose `SetValue(item, value)` is how
    a script changes a struct it holds - IronPython refuses `item.lat = value` on a value type,
    which is why the shipped scripts all write `Locationwp.lat.SetValue(item, value)`."""

    def __init__(self, owner, name, convert):
        self._owner = owner
        self._name = name
        self._convert = convert

    def __get__(self, instance, owner=None):
        if instance is None:
            return self
        return instance._values[self._name]

    def __set__(self, instance, value):
        raise ValueError(
            "Attempt to update field '%s' on value type '%s'; value type fields cannot be "
            "directly modified" % (self._name, self._owner))

    def SetValue(self, instance, value):
        instance._values[self._name] = self._convert(value)

    def GetValue(self, instance):
        return instance._values[self._name]


class _Struct(object):
    """A .NET struct: its fields at their defaults, copied when handed on."""

    _layout = ()

    def __init__(self):
        object.__setattr__(self, '_values', dict((name, default) for name, _, default in self._layout))

    def __setattr__(self, name, value):
        field = getattr(type(self), name, None)
        if isinstance(field, _Field):
            field.__set__(self, value)
        raise AttributeError("'%s' object has no attribute '%s'" % (type(self).__name__, name))

    def _copy(self):
        other = type(self)()
        other._values.update(self._values)
        return other


def _struct(name, module, layout):
    cls = type(name, (_Struct,), {'_layout': tuple(layout)})
    cls.__module__ = module
    for field, convert, _ in layout:
        setattr(cls, field, _Field(name, field, convert))
    return cls


# `MissionPlanner.Utilities.Locationwp`. `// C#: ExtLibs/Utilities/locationwp.cs:11-211`
Locationwp = _struct('Locationwp', 'MissionPlanner.Utilities', [
    ('frame', _byte, 0),
    ('Tag', _object, None),
    ('id', _uint16, 0),
    ('p1', _float32, 0.0),
    ('p2', _float32, 0.0),
    ('p3', _float32, 0.0),
    ('p4', _float32, 0.0),
    ('lat', _double, 0.0),
    ('lng', _double, 0.0),
    ('alt', _float32, 0.0),
])


def _locationwp_set(self, lat, lng, alt, id):
    # `Set` on the struct's own copy, then that copy returned: the item a script calls it on
    # is changed as well, as `this` is in the C#. `frame` is always 3, GLOBAL_RELATIVE_ALT.
    # `// C#: ExtLibs/Utilities/locationwp.cs:13-22`
    self._values['lat'] = _double(lat)
    self._values['lng'] = _double(lng)
    self._values['alt'] = _float32(alt)
    self._values['id'] = _uint16(id)
    self._values['frame'] = 3
    return self._copy()


Locationwp.Set = _locationwp_set


# --- MAVLink ------------------------------------------------------------------------------------

class _EnumValue(int):
    """A member of a .NET enum: its number wherever a number is wanted (`int(...)`, a
    parameter), `.value__` for the number itself as IronPython spells it, its name as text."""

    def __new__(cls, enum, name, value):
        member = int.__new__(cls, value)
        member._enum = enum
        member._name = name
        return member

    @property
    def value__(self):
        return int(self)

    def __str__(self):
        return self._name

    def __repr__(self):
        return '<enum %s.%s: %d>' % (self._enum, self._name, int(self))

    def ToString(self):
        return self._name


class _Enum(object):
    """A .NET enum type: its members by name, and `_of(number)` for a number the vehicle sent."""

    def __init__(self, name, members):
        object.__setattr__(self, '_name', name)
        object.__setattr__(self, '_by_value', {})
        for member, value in members:
            item = _EnumValue(name, member, value)
            object.__setattr__(self, member, item)
            self._by_value.setdefault(value, item)

    def __getattr__(self, name):
        raise AttributeError("type object '%s' has no attribute '%s'" % (self._name, name))

    def _of(self, value):
        found = self._by_value.get(int(value))
        return found if found is not None else _EnumValue(self._name, str(int(value)), int(value))


_ENUMS = dict((name, _Enum(name, members)) for name, members in ENUMS.items())
MAV_CMD = _ENUMS['MAV_CMD']
MAV_FRAME = _ENUMS['MAV_FRAME']
MAV_MISSION_RESULT = _ENUMS['MAV_MISSION_RESULT']
MAV_MISSION_TYPE = _ENUMS['MAV_MISSION_TYPE']


class MAVLinkMessage(object):
    """`MAVLink.MAVLinkMessage`: named as a delegate's type (`Func[MAVLink.MAVLinkMessage,
    bool]`); no message reaches a script in this version (see `SubscribeToPacketType`)."""


MAVLinkMessage.__module__ = 'MAVLink'


# `MAVLink.mavlink_command_long_t`, which two scripts import.
# `// C#: ExtLibs/Mavlink/Mavlink.cs:19505-19600`
mavlink_command_long_t = _struct('mavlink_command_long_t', 'MAVLink', [
    ('param1', _float32, 0.0),
    ('param2', _float32, 0.0),
    ('param3', _float32, 0.0),
    ('param4', _float32, 0.0),
    ('param5', _float32, 0.0),
    ('param6', _float32, 0.0),
    ('param7', _float32, 0.0),
    ('command', _uint16, 0),
    ('target_system', _byte, 0),
    ('target_component', _byte, 0),
    ('confirmation', _byte, 0),
])

MAVLink = _namespace(
    'MAVLink',
    MAVLinkMessage=MAVLinkMessage,
    mavlink_command_long_t=mavlink_command_long_t,
    **_ENUMS)


# --- MAV: MainV2.comPort, the MAVLinkInterface ----------------------------------------------------

def _target():
    return _link.target()


def _bind(member, args, kwargs, names, defaults):
    # One C# overload's parameters from a call's positional and named arguments, the trailing
    # ones defaulted as the C# declares them; any other call is IronPython's TypeError.
    if len(args) > len(names):
        raise TypeError('%s() takes at most %d arguments (%d given)'
                        % (member, len(names), len(args)))
    bound = dict(zip(names, args))
    for name, value in kwargs.items():
        if name not in names or name in bound:
            raise TypeError("%s() got an unexpected keyword argument '%s'" % (member, name))
        bound[name] = value
    required = len(names) - len(defaults)
    for index, name in enumerate(names):
        if name not in bound:
            if index < required:
                raise TypeError('%s() takes at least %d arguments (%d given)'
                                % (member, required, len(args) + len(kwargs)))
            bound[name] = defaults[index - required]
    return tuple(bound[name] for name in names)


class _BaseStream(object):
    """`MAV.BaseStream`, the port: whether it is open, and `Write`."""

    @property
    def IsOpen(self):
        return _link.is_open()

    def Write(self, buffer, offset, count):
        # `ICommsSerial.Write(byte[] buffer, int offset, int count)`: the bytes as they are.
        data = bytes(bytearray(buffer)[int(offset):int(offset) + int(count)])
        if not _link.write_raw(data):
            raise InvalidOperationException('The port is closed.')

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        raise _unavailable('MAV.BaseStream.' + name)


class MAVLinkInterface(object):
    """`MAV`: `MainV2.comPort`, the members of `MAVLinkInterface` the shipped scripts reach.
    `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs`"""

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        raise _unavailable('MAV.' + name)

    @property
    def BaseStream(self):
        return _BASE_STREAM

    @property
    def sysidcurrent(self):
        # `// C#: MAVLinkInterface.cs:291-302`
        return _target()[0]

    @property
    def compidcurrent(self):
        # `// C#: MAVLinkInterface.cs:305-316`
        return _target()[1]

    # Each member takes the C#'s overloads IronPython would choose between, by their count and
    # types: the [Obsolete] ones on `MAV.sysid`, `MAV.compid` - the vehicle being flown, as
    # `sysidcurrent` and `compidcurrent` are - and the current ones on the vehicle a script names.
    # A call no overload takes is IronPython's TypeError.

    def setParam(self, *args, **kwargs):
        # `setParam(string[] paramnames, double value)`: each name until one is set.
        # `// C#: MAVLinkInterface.cs:1602-1620`
        # `setParam(string paramname, double value, bool force = false)` on `sysidcurrent`,
        # `compidcurrent`. `// C#: MAVLinkInterface.cs:1622-1626`
        # `setParam(byte sysid, byte compid, string paramname, double value, bool force = false)`.
        # `// C#: MAVLinkInterface.cs:1628-1631`
        if args and isinstance(args[0], (list, tuple)) and not kwargs and len(args) == 2:
            for name in args[0]:
                if self.setParam(name, args[1]):
                    return True
            return False
        if args and isinstance(args[0], str):
            sysid, compid = _target()
            name, value, force = _bind('setParam', args, kwargs,
                                       ('paramname', 'value', 'force'), (False,))
        else:
            sysid, compid, name, value, force = _bind(
                'setParam', args, kwargs, ('sysid', 'compid', 'paramname', 'value', 'force'),
                (False,))
        if not isinstance(name, str):
            raise TypeError('setParam() expected str for paramname, got ' + type(name).__name__)
        return _call(_link.set_param, _byte(sysid), _byte(compid), name, float(value),
                     bool(force))

    def doCommand(self, *args, **kwargs):
        # `doCommand(MAV_CMD actionid, float p1..p7, bool requireack = true)` on `MAV.sysid`,
        # `MAV.compid`, or `doCommand(byte sysid, byte compid, MAV_CMD actionid, float p1..p7,
        # bool requireack = true, Action uicallback = null)`.
        # `// C#: MAVLinkInterface.cs:2673-2685`
        names = ('actionid', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'requireack')
        if len(args) + len(kwargs) <= len(names) and 'sysid' not in kwargs:
            sysid, compid = _target()
            bound = _bind('doCommand', args, kwargs, names, (True,))
        else:
            bound = _bind('doCommand', args, kwargs, ('sysid', 'compid') + names + ('uicallback',),
                          (True, None))
            sysid, compid, bound, uicallback = bound[0], bound[1], bound[2:-1], bound[-1]
            if uicallback is not None:
                # Called while the vehicle says IN_PROGRESS; no shipped script passes one.
                raise _unavailable('MAV.doCommand(uicallback)')
        actionid, params, requireack = bound[0], bound[1:8], bound[8]
        return _call(_link.command, _byte(sysid), _byte(compid), int(actionid),
                     [_float32(p) for p in params], bool(requireack))

    def doARM(self, *args, **kwargs):
        # `doARM(bool armit, bool force = false)` on `MAV.sysid`, `MAV.compid`, or
        # `doARM(byte sysid, byte compid, bool armit, bool force = false)`: 2989 and 21196, the
        # magic numbers that make ArduPilot arm or disarm past its checks.
        # `// C#: MAVLinkInterface.cs:2621-2657`
        if len(args) + len(kwargs) <= 2 and 'sysid' not in kwargs:
            sysid, compid = _target()
            armit, force = _bind('doARM', args, kwargs, ('armit', 'force'), (False,))
        else:
            sysid, compid, armit, force = _bind('doARM', args, kwargs,
                                                ('sysid', 'compid', 'armit', 'force'), (False,))
        param1 = 1.0 if armit else 0.0
        param2 = (2989.0 if armit else 21196.0) if force else 0.0
        return _call(_link.command, _byte(sysid), _byte(compid),
                     int(MAV_CMD.COMPONENT_ARM_DISARM),
                     [param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0], True)

    def doReboot(self, bootloadermode=False, currentvehicle=True):
        # `// C#: MAVLinkInterface.cs:2553-2618`
        if not currentvehicle:
            # The heartbeat scan for a vehicle not yet known; no shipped script asks for it.
            raise _unavailable('MAV.doReboot(currentvehicle=False)')
        return _call(_link.reboot, bool(bootloadermode))

    def setWPTotal(self, wp_total, type=MAV_MISSION_TYPE.MISSION):
        # Only the [Obsolete] `setWPTotal(ushort wp_total, MAV_MISSION_TYPE type)`: the one on a
        # named vehicle is `setWPTotalAsync`, a Task. `// C#: MAVLinkInterface.cs:3752-3757`
        _call(_link.set_wp_total, *_target(), _uint16(wp_total), int(type))

    def setWP(self, *args, **kwargs):
        # `setWP(Locationwp loc, ushort index, MAV_FRAME frame, byte current = 0,
        # byte autocontinue = 1, bool use_int = false, MAV_MISSION_TYPE mission_type = MISSION)`
        # on `MAV.sysid`, `MAV.compid`, or the same after `byte sysid, byte compid`.
        # `// C#: MAVLinkInterface.cs:3974-3988, 3990-4058`
        names = ('loc', 'index', 'frame', 'current', 'autocontinue', 'use_int', 'mission_type')
        defaults = (0, 1, False, MAV_MISSION_TYPE.MISSION)
        if (args and isinstance(args[0], Locationwp)) or (not args and 'loc' in kwargs):
            sysid, compid = _target()
            bound = _bind('setWP', args, kwargs, names, defaults)
        else:
            bound = _bind('setWP', args, kwargs, ('sysid', 'compid') + names, defaults)
            sysid, compid, bound = bound[0], bound[1], bound[2:]
        loc, index, frame, current, autocontinue, use_int, mission_type = bound
        if not isinstance(loc, Locationwp):
            raise TypeError('setWP() expected Locationwp, got ' + type(loc).__name__)
        if use_int:
            # The MISSION_ITEM_INT form; no shipped script asks for it.
            raise _unavailable('MAV.setWP(use_int=True)')
        values = loc._values
        result = _call(_link.set_wp, _byte(sysid), _byte(compid), _uint16(index),
                       _byte(int(frame)), values['id'], _byte(current), _byte(autocontinue),
                       [values['p1'], values['p2'], values['p3'], values['p4']],
                       _float32(values['lat']), _float32(values['lng']), _float32(values['alt']),
                       _byte(int(mission_type)))
        return MAV_MISSION_RESULT._of(result)

    def setWPACK(self, *args, **kwargs):
        # `setWPACK(MAV_MISSION_TYPE type = MISSION)` on `MAV.sysid`, `MAV.compid`, or
        # `setWPACK(byte sysid, byte compid, MAV_MISSION_TYPE type = MISSION)`.
        # `// C#: MAVLinkInterface.cs:2433-2450`
        if len(args) + len(kwargs) <= 1 and 'sysid' not in kwargs:
            sysid, compid = _target()
            (kind,) = _bind('setWPACK', args, kwargs, ('type',), (MAV_MISSION_TYPE.MISSION,))
        else:
            sysid, compid, kind = _bind('setWPACK', args, kwargs, ('sysid', 'compid', 'type'),
                                        (MAV_MISSION_TYPE.MISSION,))
        _link.set_wp_ack(_byte(sysid), _byte(compid), int(kind))

    def setWPCurrent(self, *args):
        # Only `setWPCurrent(byte sysid, byte compid, ushort index)`: IronPython refuses any
        # other count of arguments, which is how `setWPCurrent(1)` in TAKEOFF.py and
        # PARACHUTE LANDING APPROACH.py ends those scripts under Mission Planner.
        # `// C#: MAVLinkInterface.cs:2452-2501`
        if len(args) != 3:
            raise TypeError('setWPCurrent() takes exactly 3 arguments (%d given)' % len(args))
        sysid, compid, index = args
        return _call(_link.set_wp_current, _byte(sysid), _byte(compid), _uint16(index))

    def getWP(self, *args, **kwargs):
        # `getWP(ushort index, MAV_MISSION_TYPE type = MISSION)` on `MAV.sysid`, `MAV.compid`,
        # or `getWP(byte sysid, byte compid, ushort index, MAV_MISSION_TYPE type = MISSION)`:
        # that one item of that list, asked for on its own.
        # `// C#: MAVLinkInterface.cs:3397-3407, 3413-3565`
        if len(args) + len(kwargs) <= 2 and 'sysid' not in kwargs:
            sysid, compid = _target()
            index, kind = _bind('getWP', args, kwargs, ('index', 'type'),
                                (MAV_MISSION_TYPE.MISSION,))
        else:
            sysid, compid, index, kind = _bind('getWP', args, kwargs,
                                               ('sysid', 'compid', 'index', 'type'),
                                               (MAV_MISSION_TYPE.MISSION,))
        wp = _call(_link.get_wp, _byte(sysid), _byte(compid), _uint16(index), int(kind))
        loc = Locationwp()
        names = ('id', 'p1', 'p2', 'p3', 'p4', 'lat', 'lng', 'alt', 'frame')
        for name, value in zip(names, wp):
            loc._values[name] = value
        return loc

    def setGuidedModeWP(self, *args, **kwargs):
        # `setGuidedModeWP(Locationwp gotohere, bool setguidedmode = true)` on `MAV.sysid`,
        # `MAV.compid`, or the same after `byte sysid, byte compid`.
        # `// C#: MAVLinkInterface.cs:4417-4461`
        if (args and isinstance(args[0], Locationwp)) or (not args and 'gotohere' in kwargs):
            sysid, compid = _target()
            gotohere, setguidedmode = _bind('setGuidedModeWP', args, kwargs,
                                            ('gotohere', 'setguidedmode'), (True,))
        else:
            sysid, compid, gotohere, setguidedmode = _bind(
                'setGuidedModeWP', args, kwargs,
                ('sysid', 'compid', 'gotohere', 'setguidedmode'), (True,))
        if not isinstance(gotohere, Locationwp):
            raise TypeError('setGuidedModeWP() expected Locationwp, got '
                            + type(gotohere).__name__)
        if gotohere.alt == 0 or gotohere.lat == 0 or gotohere.lng == 0:
            return
        sysid, compid = _byte(sysid), _byte(compid)
        try:
            # The struct is the C#'s own copy: the script's item keeps its id.
            gotohere = gotohere._copy()
            gotohere._values['id'] = int(MAV_CMD.WAYPOINT)
            if setguidedmode:
                # "fix for followme change": `MAVlist[sysid, compid].cs.mode`, and
                # `setMode(sysid, compid, "GUIDED")` - the vehicle named, not the one flown.
                mode = _link.cs_of(sysid, compid, 'mode')
                if str('' if mode is None else mode).upper() != 'GUIDED':
                    _link.set_mode_of(sysid, compid, 'GUIDED')
            if _link.cs_of(sysid, compid, 'firmware') == 'ArduPlane':
                ans = self.setWP(sysid, compid, gotohere, 0, gotohere.frame, 2)
                if ans != MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED:
                    raise Exception('Guided Mode Failed')
            else:
                values = gotohere._values
                _link.set_position_target(_byte(sysid), _byte(compid), values['frame'],
                                          values['lat'], values['lng'], float(values['alt']))
        except Exception:
            # `log.Error(ex)`: the C# swallows every failure here.
            pass

    def SubscribeToPacketType(self, *args):
        # `SubscribeToPacketType(MAVLINK_MSG_ID msgid, Func<MAVLinkMessage, bool> function,
        # byte sysid, byte compid, bool exclusive = false)`. The shipped example2 and example10
        # pass two arguments, written against an older signature, and IronPython refuses them;
        # a call with the four is a packet feed this version does not give scripts.
        # `// C#: MAVLinkInterface.cs:5567-5594`
        if len(args) < 4:
            raise TypeError(
                'SubscribeToPacketType() takes at least 4 arguments (%d given)' % len(args))
        raise _unavailable('MAV.SubscribeToPacketType')


_BASE_STREAM = _BaseStream()
MAV = MAVLinkInterface()


# --- MainV2, the screens, Ports, Joystick ---------------------------------------------------------

class _Screen(object):
    """`FlightPlanner` or `FlightData`: a WinForms screen; `BUT_read_Click` named, the rest not."""

    def __init__(self, name, members):
        object.__setattr__(self, '_name', name)
        for member in members:
            object.__setattr__(self, member, self._refusing(member))

    def _refusing(self, member):
        name = self._name + '.' + member

        def refuse(*args):
            raise _unavailable(name)
        return refuse

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        raise _unavailable(self._name + '.' + name)


# `FlightPlanner.BUT_read_Click(sender, e)` is public (`GCSViews/FlightPlanner.cs:607`), so
# `MainV2.instance.FlightPlanner.BUT_read_Click` resolves in IronPython; example7 then fails on
# its own `null`, a name Python does not have. Reading the vehicle's mission into the Plan
# screen from a script is not given in this version, so a call raises.
FlightPlanner = _Screen('FlightPlanner', ['BUT_read_Click'])
FlightData = _Screen('FlightData', [])

# `MainV2.Comports`, `Ports`: the links Mission Planner holds - here the one.
# `// C#: MainV2.cs:422`
Ports = [MAV]

# `MainV2.joystick`: null until a joystick is started from the Joystick setup, which is what a
# script sees here always - no shipped script reads it. `// C#: MainV2.cs:491`
Joystick = None


class _Speech(object):
    """`MainV2.speechEngine`, `ISpeech`: `SpeakAsync` with its checks and its rewording, the
    text handed to the host - speech itself is DELIVERABLES D15. `// C#: Utilities/Speech.cs`"""

    def __init__(self):
        # `speechEnable` from the "speechenable" setting, off when it is absent.
        # `// C#: MainV2.cs:1005-1006; Utilities/Speech.cs:15`
        self.speechEnable = bool(_link.speech_settings()[0])

    @property
    def IsReady(self):
        return True

    def SpeakAsync(self, text):
        # `// C#: Utilities/Speech.cs:65-80`
        if not _MAIN.speechEnabled():
            return
        if text is None or str(text).strip() == '':
            return
        text = str(text)
        text = re.sub(r'\bPreArm\b', 'Pre Arm', text, flags=re.IGNORECASE)
        text = re.sub(r'\bdist\b', 'distance', text, flags=re.IGNORECASE)
        text = re.sub(r'\bNAV\b', 'Navigation', text, flags=re.IGNORECASE)
        text = re.sub(r'\b([0-9]+)m\b', r'\1 meters', text, flags=re.IGNORECASE)
        text = re.sub(r'\b([0-9]+)ft\b', r'\1 feet', text, flags=re.IGNORECASE)
        text = re.sub(r'\b([0-9]+)\bbaud\b', r'\1 baudrate', text, flags=re.IGNORECASE)
        text = re.sub(r'\bq((?!u)[a-z]+)', r'q \1', text, flags=re.IGNORECASE)
        _link.speak(text)

    def SpeakAsyncCancelAll(self):
        return None


class _MainV2(object):
    """`MainV2`: the scope's `MainV2` is `MainV2.instance`, and `MissionPlanner.MainV2` the
    class; IronPython reaches the static members through either, so here they are one object.
    `// C#: MainV2.cs:401-582`"""

    def __init__(self):
        object.__setattr__(self, 'speechEngine', _Speech())
        # From the "speech_armed_only" setting, false when it is absent. `// C#: MainV2.cs:658`
        object.__setattr__(self, 'speech_armed_only', bool(_link.speech_settings()[1]))

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        raise _unavailable('MainV2.' + name)

    def __setattr__(self, name, value):
        if name == 'speechEnable':
            # `// C#: MainV2.cs:459-465`
            if self.speechEngine is not None:
                self.speechEngine.speechEnable = bool(value)
            return
        if name in ('speechEngine', 'speech_armed_only'):
            object.__setattr__(self, name, value)
            return
        raise _unavailable('MainV2.' + name)

    @property
    def instance(self):
        return self

    @property
    def comPort(self):
        return MAV

    @property
    def Comports(self):
        return Ports

    @property
    def joystick(self):
        return Joystick

    @property
    def FlightPlanner(self):
        return FlightPlanner

    @property
    def FlightData(self):
        return FlightData

    @property
    def speechEnable(self):
        return False if self.speechEngine is None else self.speechEngine.speechEnable

    def speechEnabled(self):
        # `// C#: MainV2.cs:469-481`
        if self.speechEngine is None or not self.speechEnable:
            return False
        if self.speech_armed_only:
            return bool(getattr(cs, 'armed', False))
        return True


_MAIN = _MainV2()
MainV2 = _MAIN


# --- MissionPlanner -----------------------------------------------------------------------------

MissionPlanner = _namespace('MissionPlanner', MainV2=_MAIN)
MissionPlanner.Utilities = _namespace('MissionPlanner.Utilities', Locationwp=Locationwp)
MissionPlanner.Comms = _namespace('MissionPlanner.Comms')


# --- cs: CurrentState's numbers as their .NET types ---------------------------------------------

# `CurrentState`'s members declared `double`, and those declared as an integer type, as
# `ExtLibs/ArduPilot/CurrentState.cs` declares them; every other number there is a `float`.
_CS_DOUBLES = frozenset((
    'lat', 'lng', 'vx', 'vy', 'vz', 'lat2', 'lng2', 'glide_ratio', 'battery_voltage',
    'battery_voltage2', 'battery_voltage3', 'battery_voltage4', 'battery_voltage5',
    'battery_voltage6', 'battery_voltage7', 'battery_voltage8', 'battery_voltage9', 'current',
    'current2', 'current3', 'current4', 'current5', 'current6', 'current7', 'current8',
    'current9', 'watts', 'battery_mahperkm', 'battery_kmleft', 'battery_usedmah',
    'battery_usedmah2', 'battery_usedmah3', 'battery_usedmah4', 'battery_usedmah5',
    'battery_usedmah6', 'battery_usedmah7', 'battery_usedmah8', 'battery_usedmah9', 'HomeAlt',
    'timesincelastshot', 'ahrs2_lat', 'ahrs2_lng',
))
_CS_INTEGERS = frozenset((
    'hilch1', 'hilch2', 'hilch3', 'hilch4', 'hilch5', 'hilch6', 'hilch7', 'hilch8',
    'lastautowp', 'rcoverridech1', 'rcoverridech2', 'rcoverridech3', 'rcoverridech4',
    'rcoverridech5', 'rcoverridech6', 'rcoverridech7', 'rcoverridech8', 'rcoverridech9',
    'rcoverridech10', 'rcoverridech11', 'rcoverridech12', 'rcoverridech13', 'rcoverridech14',
    'rcoverridech15', 'rcoverridech16', 'rcoverridech17', 'rcoverridech18', 'hygrotemp1',
    'hygrohumi1', 'hygrotemp2', 'hygrohumi2', 'tot', 'toh', 'battery_remaining',
    'battery_remaining2', 'battery_remaining3', 'battery_remaining4', 'battery_remaining5',
    'battery_remaining6', 'battery_remaining7', 'battery_remaining8', 'battery_remaining9',
    'KIndex', 'gen_runtime', 'gen_maint_time', 'efi_health', 'xpdr_mode_A_squawk_code',
    'xpdr_nic', 'xpdr_nacp', 'xpdr_board_temperature', 'fenceb_count', 'fenceb_status',
    'fenceb_type',
))


def _general(value, digits):
    # .NET Framework's "G" with its default precision: 15 digits for a Double, 7 for a Single.
    if value != value:
        return 'NaN'
    if value == float('inf'):
        return 'Infinity'
    if value == float('-inf'):
        return '-Infinity'
    if value == 0.0:
        return '0'
    return '%.*G' % (digits, value)


class _Double(float):
    """A `CurrentState` number declared `double`, as IronPython hands over a `System.Double`,
    whose `ToString()` is .NET Framework's "G15"."""

    def ToString(self):
        return _general(float(self), 15)


class _Single(float):
    """A `CurrentState` number declared `float`, a `System.Single`: its `ToString()` on .NET
    Framework 4.7.2, which Mission Planner targets, is "G7" of the single - example8's
    `cs.roll.ToString()` speaks seven significant digits at most."""

    def ToString(self):
        return _general(_float32(self), 7)


class _Integer(int):
    """A `CurrentState` number declared as an integer type: an `int` to IronPython."""

    def ToString(self):
        return str(int(self))


class _CurrentState(object):
    """`cs`: the engine's own, each number as its C# type has it."""

    def __getattr__(self, name):
        value = getattr(_cs, name)
        if type(value) is float:
            if name in _CS_DOUBLES:
                return _Double(value)
            if name in _CS_INTEGERS:
                return _Integer(int(round(value)))
            return _Single(value)
        return value


cs = _CurrentState()


# --- import, time.sleep ---------------------------------------------------------------------------

_ROOTS = ('System', 'MissionPlanner', 'MAVLink', 'clr')
_real_import = builtins.__import__


def _import(name, globals=None, locals=None, fromlist=(), level=0):
    # A .NET namespace this module does not give - `System.Windows.Forms`,
    # `System.Diagnostics` - stops the script there, named.
    if level == 0 and name not in sys.modules and name.split('.')[0] in _ROOTS:
        raise _unavailable(name)
    return _real_import(name, globals, locals, fromlist, level)


builtins.__import__ = _import


def _sleep(seconds):
    # The C#'s Abort button is `Thread.Abort`, which stops a script anywhere, `time.sleep`
    # included; here it stops one at its waits, and this makes `time.sleep` one of them.
    _script.Sleep(float(seconds) * 1000.0)


time.sleep = _sleep
