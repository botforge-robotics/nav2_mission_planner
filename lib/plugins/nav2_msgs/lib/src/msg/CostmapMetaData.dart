// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

import 'package:builtin_interfaces/msg.dart' as builtin_interfaces;
import 'package:geometry_msgs/msg.dart' as geometry_msgs;

class CostmapMetaData extends RosMessage<CostmapMetaData> {
  builtin_interfaces.Time map_load_time;
  builtin_interfaces.Time update_time;
  String layer;
  double resolution;
  int size_x;
  int size_y;
  geometry_msgs.Pose origin;

  static CostmapMetaData $prototype = CostmapMetaData();

  // Constructor
  CostmapMetaData(
      {builtin_interfaces.Time? map_load_time,
      builtin_interfaces.Time? update_time,
      String? layer,
      double? resolution,
      int? size_x,
      int? size_y,
      geometry_msgs.Pose? origin})
      : map_load_time = map_load_time ?? builtin_interfaces.Time(),
        update_time = update_time ?? builtin_interfaces.Time(),
        layer = layer ?? '',
        resolution = resolution ?? 0.0,
        size_x = size_x ?? 0,
        size_y = size_y ?? 0,
        origin = origin ?? geometry_msgs.Pose();

  @override
  String get fullType => 'nav2_msgs/msg/CostmapMetaData';

  @override
  String get messageDefinition => '''builtin_interfaces/Time map_load_time
builtin_interfaces/Time update_time
string layer
float32 resolution
uint32 size_x
uint32 size_y
geometry_msgs/Pose origin''';

  @override
  int getMessageSize() {
    return (map_load_time.getMessageSize() +
            update_time.getMessageSize() +
            4 +
            layer.length +
            4 +
            4 +
            4 +
            origin.getMessageSize())
        .toInt();
  }

  @override
  Map<String, dynamic> toJson() => {
        'map_load_time': map_load_time.toJson(),
        'update_time': update_time.toJson(),
        'layer': layer,
        'resolution': resolution,
        'size_x': size_x,
        'size_y': size_y,
        'origin': origin.toJson()
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  CostmapMetaData fromJson(Map<String, dynamic> jsonMap) {
    return CostmapMetaData(
        map_load_time: builtin_interfaces.Time()
            .fromJson(jsonMap['map_load_time'] as Map<String, dynamic>),
        update_time: builtin_interfaces.Time()
            .fromJson(jsonMap['update_time'] as Map<String, dynamic>),
        layer: jsonMap['layer'] as String,
        resolution: jsonMap['resolution'] as double,
        size_x: jsonMap['size_x'] as int,
        size_y: jsonMap['size_y'] as int,
        origin: geometry_msgs.Pose()
            .fromJson(jsonMap['origin'] as Map<String, dynamic>));
  }
}
