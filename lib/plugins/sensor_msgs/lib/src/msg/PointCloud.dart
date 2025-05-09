// Generated by ros_to_dart_converter
// Do not edit manually

import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

import 'package:geometry_msgs/msg.dart' as geometry_msgs;
import 'package:std_msgs/msg.dart' as std_msgs;
import 'ChannelFloat32.dart';

class PointCloud extends RosMessage<PointCloud> {
  std_msgs.Header header;
  List<geometry_msgs.Point32> points;
  List<ChannelFloat32> channels;

  static PointCloud $prototype = PointCloud();

  // Constructor
  PointCloud(
      {std_msgs.Header? header,
      List<geometry_msgs.Point32>? points,
      List<ChannelFloat32>? channels})
      : header = header ?? std_msgs.Header(),
        points = points ?? const [],
        channels = channels ?? const [];

  @override
  String get fullType => 'sensor_msgs/msg/PointCloud';

  @override
  String get messageDefinition => '''std_msgs/Header header
geometry_msgs/Point32[] points
ChannelFloat32[] channels''';

  @override
  int getMessageSize() {
    return (header.getMessageSize() +
            4 +
            points.fold(0, (sum, item) => sum + item.getMessageSize()) +
            4 +
            channels.fold(0, (sum, item) => sum + item.getMessageSize()))
        .toInt();
  }

  @override
  Map<String, dynamic> toJson() => {
        'header': header.toJson(),
        'points': points.map((item) => item.toJson()).toList(),
        'channels': channels.map((item) => item.toJson()).toList()
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  PointCloud fromJson(Map<String, dynamic> jsonMap) {
    return PointCloud(
        header: std_msgs.Header()
            .fromJson(jsonMap['header'] as Map<String, dynamic>),
        points: (jsonMap['points'] as List)
            .map((item) =>
                geometry_msgs.Point32().fromJson(item as Map<String, dynamic>))
            .toList(),
        channels: (jsonMap['channels'] as List)
            .map((item) =>
                ChannelFloat32().fromJson(item as Map<String, dynamic>))
            .toList());
  }
}
