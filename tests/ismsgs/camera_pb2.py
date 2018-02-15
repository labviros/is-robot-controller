# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: camera.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import common_pb2 as common__pb2
import image_pb2 as image__pb2
from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='camera.proto',
  package='is.vision',
  syntax='proto3',
  serialized_pb=_b('\n\x0c\x63\x61mera.proto\x12\tis.vision\x1a\x0c\x63ommon.proto\x1a\x0bimage.proto\x1a\x1fgoogle/protobuf/timestamp.proto\"1\n\rCameraSetting\x12\x11\n\tautomatic\x18\x01 \x01(\x08\x12\r\n\x05ratio\x18\x02 \x01(\x02\"\xc9\x04\n\x0e\x43\x61meraSettings\x12,\n\nbrightness\x18\x01 \x01(\x0b\x32\x18.is.vision.CameraSetting\x12*\n\x08\x65xposure\x18\x02 \x01(\x0b\x32\x18.is.vision.CameraSetting\x12\'\n\x05\x66ocus\x18\x03 \x01(\x0b\x32\x18.is.vision.CameraSetting\x12&\n\x04gain\x18\x04 \x01(\x0b\x32\x18.is.vision.CameraSetting\x12\'\n\x05gamma\x18\x05 \x01(\x0b\x32\x18.is.vision.CameraSetting\x12%\n\x03hue\x18\x06 \x01(\x0b\x32\x18.is.vision.CameraSetting\x12&\n\x04iris\x18\x07 \x01(\x0b\x32\x18.is.vision.CameraSetting\x12,\n\nsaturation\x18\x08 \x01(\x0b\x32\x18.is.vision.CameraSetting\x12+\n\tsharpness\x18\t \x01(\x0b\x32\x18.is.vision.CameraSetting\x12)\n\x07shutter\x18\n \x01(\x0b\x32\x18.is.vision.CameraSetting\x12\x32\n\x10white_balance_bu\x18\x0b \x01(\x0b\x32\x18.is.vision.CameraSetting\x12\x32\n\x10white_balance_rv\x18\x0c \x01(\x0b\x32\x18.is.vision.CameraSetting\x12&\n\x04zoom\x18\r \x01(\x0b\x32\x18.is.vision.CameraSetting\"\x91\x01\n\x0c\x43\x61meraConfig\x12-\n\x08sampling\x18\x01 \x01(\x0b\x32\x1b.is.common.SamplingSettings\x12\'\n\x05image\x18\x02 \x01(\x0b\x32\x18.is.vision.ImageSettings\x12)\n\x06\x63\x61mera\x18\x03 \x01(\x0b\x32\x19.is.vision.CameraSettings\"\x8c\x02\n\x11\x43\x61meraCalibration\x12\n\n\x02id\x18\x01 \x01(\t\x12\x31\n\rcalibrated_at\x18\x02 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12\r\n\x05\x65rror\x18\x03 \x01(\x01\x12)\n\nresolution\x18\x04 \x01(\x0b\x32\x15.is.vision.Resolution\x12$\n\tintrinsic\x18\x05 \x01(\x0b\x32\x11.is.common.Tensor\x12%\n\ndistortion\x18\x06 \x01(\x0b\x32\x11.is.common.Tensor\x12\x31\n\textrinsic\x18\x07 \x01(\x0b\x32\x1e.is.vision.FrameTransformation\"N\n\x13\x46rameTransformation\x12\x0c\n\x04\x66rom\x18\x01 \x01(\t\x12\n\n\x02to\x18\x02 \x01(\t\x12\x1d\n\x02tf\x18\x03 \x01(\x0b\x32\x11.is.common.Tensor*]\n\x12\x43\x61meraConfigFields\x12\x07\n\x03\x41LL\x10\x00\x12\x15\n\x11SAMPLING_SETTINGS\x10\x01\x12\x12\n\x0eIMAGE_SETTINGS\x10\x02\x12\x13\n\x0f\x43\x41MERA_SETTINGS\x10\x03\x42\x11\n\rcom.is.visionP\x01\x62\x06proto3')
  ,
  dependencies=[common__pb2.DESCRIPTOR,image__pb2.DESCRIPTOR,google_dot_protobuf_dot_timestamp__pb2.DESCRIPTOR,])

_CAMERACONFIGFIELDS = _descriptor.EnumDescriptor(
  name='CameraConfigFields',
  full_name='is.vision.CameraConfigFields',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='ALL', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SAMPLING_SETTINGS', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IMAGE_SETTINGS', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CAMERA_SETTINGS', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1225,
  serialized_end=1318,
)
_sym_db.RegisterEnumDescriptor(_CAMERACONFIGFIELDS)

CameraConfigFields = enum_type_wrapper.EnumTypeWrapper(_CAMERACONFIGFIELDS)
ALL = 0
SAMPLING_SETTINGS = 1
IMAGE_SETTINGS = 2
CAMERA_SETTINGS = 3



_CAMERASETTING = _descriptor.Descriptor(
  name='CameraSetting',
  full_name='is.vision.CameraSetting',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='automatic', full_name='is.vision.CameraSetting.automatic', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ratio', full_name='is.vision.CameraSetting.ratio', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_end=136,
)


_CAMERASETTINGS = _descriptor.Descriptor(
  name='CameraSettings',
  full_name='is.vision.CameraSettings',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='brightness', full_name='is.vision.CameraSettings.brightness', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='exposure', full_name='is.vision.CameraSettings.exposure', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='focus', full_name='is.vision.CameraSettings.focus', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gain', full_name='is.vision.CameraSettings.gain', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gamma', full_name='is.vision.CameraSettings.gamma', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='hue', full_name='is.vision.CameraSettings.hue', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='iris', full_name='is.vision.CameraSettings.iris', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='saturation', full_name='is.vision.CameraSettings.saturation', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sharpness', full_name='is.vision.CameraSettings.sharpness', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='shutter', full_name='is.vision.CameraSettings.shutter', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='white_balance_bu', full_name='is.vision.CameraSettings.white_balance_bu', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='white_balance_rv', full_name='is.vision.CameraSettings.white_balance_rv', index=11,
      number=12, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='zoom', full_name='is.vision.CameraSettings.zoom', index=12,
      number=13, type=11, cpp_type=10, label=1,
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
  serialized_start=139,
  serialized_end=724,
)


_CAMERACONFIG = _descriptor.Descriptor(
  name='CameraConfig',
  full_name='is.vision.CameraConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sampling', full_name='is.vision.CameraConfig.sampling', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='image', full_name='is.vision.CameraConfig.image', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='camera', full_name='is.vision.CameraConfig.camera', index=2,
      number=3, type=11, cpp_type=10, label=1,
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
  serialized_start=727,
  serialized_end=872,
)


_CAMERACALIBRATION = _descriptor.Descriptor(
  name='CameraCalibration',
  full_name='is.vision.CameraCalibration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='is.vision.CameraCalibration.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='calibrated_at', full_name='is.vision.CameraCalibration.calibrated_at', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='error', full_name='is.vision.CameraCalibration.error', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='resolution', full_name='is.vision.CameraCalibration.resolution', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='intrinsic', full_name='is.vision.CameraCalibration.intrinsic', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distortion', full_name='is.vision.CameraCalibration.distortion', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='extrinsic', full_name='is.vision.CameraCalibration.extrinsic', index=6,
      number=7, type=11, cpp_type=10, label=1,
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
  serialized_start=875,
  serialized_end=1143,
)


_FRAMETRANSFORMATION = _descriptor.Descriptor(
  name='FrameTransformation',
  full_name='is.vision.FrameTransformation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='from', full_name='is.vision.FrameTransformation.from', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='to', full_name='is.vision.FrameTransformation.to', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tf', full_name='is.vision.FrameTransformation.tf', index=2,
      number=3, type=11, cpp_type=10, label=1,
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
  serialized_start=1145,
  serialized_end=1223,
)

_CAMERASETTINGS.fields_by_name['brightness'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['exposure'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['focus'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['gain'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['gamma'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['hue'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['iris'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['saturation'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['sharpness'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['shutter'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['white_balance_bu'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['white_balance_rv'].message_type = _CAMERASETTING
_CAMERASETTINGS.fields_by_name['zoom'].message_type = _CAMERASETTING
_CAMERACONFIG.fields_by_name['sampling'].message_type = common__pb2._SAMPLINGSETTINGS
_CAMERACONFIG.fields_by_name['image'].message_type = image__pb2._IMAGESETTINGS
_CAMERACONFIG.fields_by_name['camera'].message_type = _CAMERASETTINGS
_CAMERACALIBRATION.fields_by_name['calibrated_at'].message_type = google_dot_protobuf_dot_timestamp__pb2._TIMESTAMP
_CAMERACALIBRATION.fields_by_name['resolution'].message_type = image__pb2._RESOLUTION
_CAMERACALIBRATION.fields_by_name['intrinsic'].message_type = common__pb2._TENSOR
_CAMERACALIBRATION.fields_by_name['distortion'].message_type = common__pb2._TENSOR
_CAMERACALIBRATION.fields_by_name['extrinsic'].message_type = _FRAMETRANSFORMATION
_FRAMETRANSFORMATION.fields_by_name['tf'].message_type = common__pb2._TENSOR
DESCRIPTOR.message_types_by_name['CameraSetting'] = _CAMERASETTING
DESCRIPTOR.message_types_by_name['CameraSettings'] = _CAMERASETTINGS
DESCRIPTOR.message_types_by_name['CameraConfig'] = _CAMERACONFIG
DESCRIPTOR.message_types_by_name['CameraCalibration'] = _CAMERACALIBRATION
DESCRIPTOR.message_types_by_name['FrameTransformation'] = _FRAMETRANSFORMATION
DESCRIPTOR.enum_types_by_name['CameraConfigFields'] = _CAMERACONFIGFIELDS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CameraSetting = _reflection.GeneratedProtocolMessageType('CameraSetting', (_message.Message,), dict(
  DESCRIPTOR = _CAMERASETTING,
  __module__ = 'camera_pb2'
  # @@protoc_insertion_point(class_scope:is.vision.CameraSetting)
  ))
_sym_db.RegisterMessage(CameraSetting)

CameraSettings = _reflection.GeneratedProtocolMessageType('CameraSettings', (_message.Message,), dict(
  DESCRIPTOR = _CAMERASETTINGS,
  __module__ = 'camera_pb2'
  # @@protoc_insertion_point(class_scope:is.vision.CameraSettings)
  ))
_sym_db.RegisterMessage(CameraSettings)

CameraConfig = _reflection.GeneratedProtocolMessageType('CameraConfig', (_message.Message,), dict(
  DESCRIPTOR = _CAMERACONFIG,
  __module__ = 'camera_pb2'
  # @@protoc_insertion_point(class_scope:is.vision.CameraConfig)
  ))
_sym_db.RegisterMessage(CameraConfig)

CameraCalibration = _reflection.GeneratedProtocolMessageType('CameraCalibration', (_message.Message,), dict(
  DESCRIPTOR = _CAMERACALIBRATION,
  __module__ = 'camera_pb2'
  # @@protoc_insertion_point(class_scope:is.vision.CameraCalibration)
  ))
_sym_db.RegisterMessage(CameraCalibration)

FrameTransformation = _reflection.GeneratedProtocolMessageType('FrameTransformation', (_message.Message,), dict(
  DESCRIPTOR = _FRAMETRANSFORMATION,
  __module__ = 'camera_pb2'
  # @@protoc_insertion_point(class_scope:is.vision.FrameTransformation)
  ))
_sym_db.RegisterMessage(FrameTransformation)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\rcom.is.visionP\001'))
# @@protoc_insertion_point(module_scope)