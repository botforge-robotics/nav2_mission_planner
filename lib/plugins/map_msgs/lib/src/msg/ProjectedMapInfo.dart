// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class ProjectedMapInfo extends RosMessage<ProjectedMapInfo> {
  String frame_id;
  double x;
  double y;
  double width;
  double height;
  double min_z;
  double max_z;

  static ProjectedMapInfo $prototype = ProjectedMapInfo();

  // Constructor
  ProjectedMapInfo(
      {String? frame_id,
      double? x,
      double? y,
      double? width,
      double? height,
      double? min_z,
      double? max_z})
      : frame_id = frame_id ?? '',
        x = x ?? 0.0,
        y = y ?? 0.0,
        width = width ?? 0.0,
        height = height ?? 0.0,
        min_z = min_z ?? 0.0,
        max_z = max_z ?? 0.0;

  @override
  String get fullType => 'map_msgs/msg/ProjectedMapInfo';

  @override
  String get messageDefinition => '''string frame_id
float64 x
float64 y
float64 width
float64 height
float64 min_z
float64 max_z''';

  @override
  int getMessageSize() {
    return (4 + frame_id.length + 8 + 8 + 8 + 8 + 8 + 8).toInt();
  }

  @override
  Map<String, dynamic> toJson() => {
        'frame_id': frame_id,
        'x': x,
        'y': y,
        'width': width,
        'height': height,
        'min_z': min_z,
        'max_z': max_z
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ProjectedMapInfo fromJson(Map<String, dynamic> jsonMap) {
    return ProjectedMapInfo(
        frame_id: jsonMap['frame_id'] as String,
        x: jsonMap['x'] as double,
        y: jsonMap['y'] as double,
        width: jsonMap['width'] as double,
        height: jsonMap['height'] as double,
        min_z: jsonMap['min_z'] as double,
        max_z: jsonMap['max_z'] as double);
  }
}
