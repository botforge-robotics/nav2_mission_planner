// Generated by ros_to_dart_converter
// Do not edit manually

// ignore_for_file: non_constant_identifier_names

import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

import 'package:std_msgs/msg.dart' as std_msgs;

class Image extends RosMessage<Image> {
  std_msgs.Header header;
  int height;
  int width;
  String encoding;
  int is_bigendian;
  int step;
  List<int> data;

  static Image $prototype = Image();

  // Constructor
  Image(
      {std_msgs.Header? header,
      int? height,
      int? width,
      String? encoding,
      int? is_bigendian,
      int? step,
      List<int>? data})
      : header = header ?? std_msgs.Header(),
        height = height ?? 0,
        width = width ?? 0,
        encoding = encoding ?? '',
        is_bigendian = is_bigendian ?? 0,
        step = step ?? 0,
        data = data ?? const [];

  @override
  String get fullType => 'sensor_msgs/msg/Image';

  @override
  String get messageDefinition => '''std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data''';

  @override
  int getMessageSize() {
    return (header.getMessageSize() +
            4 +
            4 +
            4 +
            encoding.length +
            1 +
            4 +
            4 +
            (data.length * 1))
        .toInt();
  }

  @override
  Map<String, dynamic> toJson() => {
        'header': header.toJson(),
        'height': height,
        'width': width,
        'encoding': encoding,
        'is_bigendian': is_bigendian,
        'step': step,
        'data': data
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  Image fromJson(Map<String, dynamic> jsonMap) {
    return Image(
        header: std_msgs.Header()
            .fromJson(jsonMap['header'] as Map<String, dynamic>),
        height: jsonMap['height'] as int,
        width: jsonMap['width'] as int,
        encoding: jsonMap['encoding'] as String,
        is_bigendian: jsonMap['is_bigendian'] as int,
        step: jsonMap['step'] as int,
        data: (jsonMap['data'] as List).cast<int>());
  }
}
