# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: te3002b.proto
# Protobuf Python Version: 5.26.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\rte3002b.proto\x12\rTE3002BSimPkg\x1a\x1bgoogle/protobuf/empty.proto\"I\n\nImageFrame\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x0c\x12\r\n\x05width\x18\x02 \x01(\r\x12\x0e\n\x06height\x18\x03 \x01(\r\x12\x0e\n\x06\x66ormat\x18\x04 \x01(\t\"*\n\x07Vector3\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01z\x18\x03 \x01(\x01\"^\n\x0b\x43ommandData\x12&\n\x06linear\x18\x01 \x01(\x0b\x32\x16.TE3002BSimPkg.Vector3\x12\'\n\x07\x61ngular\x18\x02 \x01(\x0b\x32\x16.TE3002BSimPkg.Vector32\x96\x01\n\nTE3002BSim\x12\x44\n\rGetImageFrame\x12\x16.google.protobuf.Empty\x1a\x19.TE3002BSimPkg.ImageFrame\"\x00\x12\x42\n\nSetCommand\x12\x1a.TE3002BSimPkg.CommandData\x1a\x16.google.protobuf.Empty\"\x00\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'te3002b_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_IMAGEFRAME']._serialized_start=61
  _globals['_IMAGEFRAME']._serialized_end=134
  _globals['_VECTOR3']._serialized_start=136
  _globals['_VECTOR3']._serialized_end=178
  _globals['_COMMANDDATA']._serialized_start=180
  _globals['_COMMANDDATA']._serialized_end=274
  _globals['_TE3002BSIM']._serialized_start=277
  _globals['_TE3002BSIM']._serialized_end=427
# @@protoc_insertion_point(module_scope)
