// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

import 'package:sensor_msgs/msg.dart' as sensor_msgs;
import 'package:std_msgs/msg.dart' as std_msgs;

class PointCloud2Update extends RosMessage<PointCloud2Update> {
  std_msgs.Header header;
  int type;
  sensor_msgs.PointCloud2 points;

  static PointCloud2Update $prototype = PointCloud2Update();

  // Constructor
  PointCloud2Update(
      {std_msgs.Header? header, int? type, sensor_msgs.PointCloud2? points})
      : header = header ?? std_msgs.Header(),
        type = type ?? 0,
        points = points ?? sensor_msgs.PointCloud2();

  @override
  String get fullType => 'map_msgs/msg/PointCloud2Update';

  @override
  String get messageDefinition => '''std_msgs/Header header
uint32 type
sensor_msgs/PointCloud2 points''';

  @override
  int getMessageSize() {
    return (header.getMessageSize() + 4 + points.getMessageSize()).toInt();
  }

  @override
  Map<String, dynamic> toJson() =>
      {'header': header.toJson(), 'type': type, 'points': points.toJson()};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  PointCloud2Update fromJson(Map<String, dynamic> jsonMap) {
    return PointCloud2Update(
        header: std_msgs.Header()
            .fromJson(jsonMap['header'] as Map<String, dynamic>),
        type: jsonMap['type'] as int,
        points: sensor_msgs.PointCloud2()
            .fromJson(jsonMap['points'] as Map<String, dynamic>));
  }
}
