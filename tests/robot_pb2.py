# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: robot.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import common_pb2 as common__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='robot.proto',
  package='is.robot',
  syntax='proto3',
  serialized_pb=_b('\n\x0brobot.proto\x12\x08is.robot\x1a\x0c\x63ommon.proto\".\n\x0bRobotConfig\x12\x1f\n\x05speed\x18\x01 \x01(\x0b\x32\x10.is.common.Speed\".\n\rFinalPoseTask\x12\x1d\n\x04goal\x18\x01 \x01(\x0b\x32\x0f.is.common.Pose\"2\n\x08PathTask\x12&\n\tpositions\x18\x01 \x03(\x0b\x32\x13.is.common.Position\"Z\n\x0eTrajectoryTask\x12&\n\tpositions\x18\x01 \x03(\x0b\x32\x13.is.common.Position\x12 \n\x06speeds\x18\x02 \x03(\x0b\x32\x10.is.common.Speed\"\xd6\x01\n\tRobotTask\x12\'\n\x04pose\x18\x01 \x01(\x0b\x32\x17.is.robot.FinalPoseTaskH\x00\x12\"\n\x04path\x18\x02 \x01(\x0b\x32\x12.is.robot.PathTaskH\x00\x12.\n\ntrajectory\x18\x03 \x01(\x0b\x32\x18.is.robot.TrajectoryTaskH\x00\x12\x15\n\rallowed_error\x18\x0e \x01(\x02\x12-\n\x08sampling\x18\x0f \x01(\x0b\x32\x1b.is.common.SamplingSettingsB\x06\n\x04Task\"\xad\x01\n\x17RobotControllerProgress\x12\'\n\rcurrent_speed\x18\x01 \x01(\x0b\x32\x10.is.common.Speed\x12%\n\x0c\x63urrent_pose\x18\x02 \x01(\x0b\x32\x0f.is.common.Pose\x12%\n\x0c\x64\x65sired_pose\x18\x03 \x01(\x0b\x32\x0f.is.common.Pose\x12\r\n\x05\x65rror\x18\x04 \x01(\x02\x12\x0c\n\x04\x64one\x18\x05 \x01(\x08\x42\x10\n\x0c\x63om.is.robotP\x01\x62\x06proto3')
  ,
  dependencies=[common__pb2.DESCRIPTOR,])




_ROBOTCONFIG = _descriptor.Descriptor(
  name='RobotConfig',
  full_name='is.robot.RobotConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='speed', full_name='is.robot.RobotConfig.speed', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=39,
  serialized_end=85,
)


_FINALPOSETASK = _descriptor.Descriptor(
  name='FinalPoseTask',
  full_name='is.robot.FinalPoseTask',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='goal', full_name='is.robot.FinalPoseTask.goal', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=87,
  serialized_end=133,
)


_PATHTASK = _descriptor.Descriptor(
  name='PathTask',
  full_name='is.robot.PathTask',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='positions', full_name='is.robot.PathTask.positions', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=135,
  serialized_end=185,
)


_TRAJECTORYTASK = _descriptor.Descriptor(
  name='TrajectoryTask',
  full_name='is.robot.TrajectoryTask',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='positions', full_name='is.robot.TrajectoryTask.positions', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='speeds', full_name='is.robot.TrajectoryTask.speeds', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=187,
  serialized_end=277,
)


_ROBOTTASK = _descriptor.Descriptor(
  name='RobotTask',
  full_name='is.robot.RobotTask',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='pose', full_name='is.robot.RobotTask.pose', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='path', full_name='is.robot.RobotTask.path', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trajectory', full_name='is.robot.RobotTask.trajectory', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='allowed_error', full_name='is.robot.RobotTask.allowed_error', index=3,
      number=14, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sampling', full_name='is.robot.RobotTask.sampling', index=4,
      number=15, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='Task', full_name='is.robot.RobotTask.Task',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=280,
  serialized_end=494,
)


_ROBOTCONTROLLERPROGRESS = _descriptor.Descriptor(
  name='RobotControllerProgress',
  full_name='is.robot.RobotControllerProgress',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='current_speed', full_name='is.robot.RobotControllerProgress.current_speed', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='current_pose', full_name='is.robot.RobotControllerProgress.current_pose', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='desired_pose', full_name='is.robot.RobotControllerProgress.desired_pose', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='error', full_name='is.robot.RobotControllerProgress.error', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='done', full_name='is.robot.RobotControllerProgress.done', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=497,
  serialized_end=670,
)

_ROBOTCONFIG.fields_by_name['speed'].message_type = common__pb2._SPEED
_FINALPOSETASK.fields_by_name['goal'].message_type = common__pb2._POSE
_PATHTASK.fields_by_name['positions'].message_type = common__pb2._POSITION
_TRAJECTORYTASK.fields_by_name['positions'].message_type = common__pb2._POSITION
_TRAJECTORYTASK.fields_by_name['speeds'].message_type = common__pb2._SPEED
_ROBOTTASK.fields_by_name['pose'].message_type = _FINALPOSETASK
_ROBOTTASK.fields_by_name['path'].message_type = _PATHTASK
_ROBOTTASK.fields_by_name['trajectory'].message_type = _TRAJECTORYTASK
_ROBOTTASK.fields_by_name['sampling'].message_type = common__pb2._SAMPLINGSETTINGS
_ROBOTTASK.oneofs_by_name['Task'].fields.append(
  _ROBOTTASK.fields_by_name['pose'])
_ROBOTTASK.fields_by_name['pose'].containing_oneof = _ROBOTTASK.oneofs_by_name['Task']
_ROBOTTASK.oneofs_by_name['Task'].fields.append(
  _ROBOTTASK.fields_by_name['path'])
_ROBOTTASK.fields_by_name['path'].containing_oneof = _ROBOTTASK.oneofs_by_name['Task']
_ROBOTTASK.oneofs_by_name['Task'].fields.append(
  _ROBOTTASK.fields_by_name['trajectory'])
_ROBOTTASK.fields_by_name['trajectory'].containing_oneof = _ROBOTTASK.oneofs_by_name['Task']
_ROBOTCONTROLLERPROGRESS.fields_by_name['current_speed'].message_type = common__pb2._SPEED
_ROBOTCONTROLLERPROGRESS.fields_by_name['current_pose'].message_type = common__pb2._POSE
_ROBOTCONTROLLERPROGRESS.fields_by_name['desired_pose'].message_type = common__pb2._POSE
DESCRIPTOR.message_types_by_name['RobotConfig'] = _ROBOTCONFIG
DESCRIPTOR.message_types_by_name['FinalPoseTask'] = _FINALPOSETASK
DESCRIPTOR.message_types_by_name['PathTask'] = _PATHTASK
DESCRIPTOR.message_types_by_name['TrajectoryTask'] = _TRAJECTORYTASK
DESCRIPTOR.message_types_by_name['RobotTask'] = _ROBOTTASK
DESCRIPTOR.message_types_by_name['RobotControllerProgress'] = _ROBOTCONTROLLERPROGRESS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RobotConfig = _reflection.GeneratedProtocolMessageType('RobotConfig', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTCONFIG,
  __module__ = 'robot_pb2'
  # @@protoc_insertion_point(class_scope:is.robot.RobotConfig)
  ))
_sym_db.RegisterMessage(RobotConfig)

FinalPoseTask = _reflection.GeneratedProtocolMessageType('FinalPoseTask', (_message.Message,), dict(
  DESCRIPTOR = _FINALPOSETASK,
  __module__ = 'robot_pb2'
  # @@protoc_insertion_point(class_scope:is.robot.FinalPoseTask)
  ))
_sym_db.RegisterMessage(FinalPoseTask)

PathTask = _reflection.GeneratedProtocolMessageType('PathTask', (_message.Message,), dict(
  DESCRIPTOR = _PATHTASK,
  __module__ = 'robot_pb2'
  # @@protoc_insertion_point(class_scope:is.robot.PathTask)
  ))
_sym_db.RegisterMessage(PathTask)

TrajectoryTask = _reflection.GeneratedProtocolMessageType('TrajectoryTask', (_message.Message,), dict(
  DESCRIPTOR = _TRAJECTORYTASK,
  __module__ = 'robot_pb2'
  # @@protoc_insertion_point(class_scope:is.robot.TrajectoryTask)
  ))
_sym_db.RegisterMessage(TrajectoryTask)

RobotTask = _reflection.GeneratedProtocolMessageType('RobotTask', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTTASK,
  __module__ = 'robot_pb2'
  # @@protoc_insertion_point(class_scope:is.robot.RobotTask)
  ))
_sym_db.RegisterMessage(RobotTask)

RobotControllerProgress = _reflection.GeneratedProtocolMessageType('RobotControllerProgress', (_message.Message,), dict(
  DESCRIPTOR = _ROBOTCONTROLLERPROGRESS,
  __module__ = 'robot_pb2'
  # @@protoc_insertion_point(class_scope:is.robot.RobotControllerProgress)
  ))
_sym_db.RegisterMessage(RobotControllerProgress)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\014com.is.robotP\001'))
# @@protoc_insertion_point(module_scope)
