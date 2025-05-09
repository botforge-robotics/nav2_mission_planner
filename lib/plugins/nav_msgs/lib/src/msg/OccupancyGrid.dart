// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

import 'package:std_msgs/msg.dart' as std_msgs;
import 'MapMetaData.dart';

class OccupancyGrid extends RosMessage<OccupancyGrid> {
  std_msgs.Header header;
  MapMetaData info;
  List<int> data;

  static OccupancyGrid $prototype = OccupancyGrid();

  // Constructor
  OccupancyGrid({std_msgs.Header? header, MapMetaData? info, List<int>? data})
      : header = header ?? std_msgs.Header(),
        info = info ?? MapMetaData(),
        data = data ?? const [];

  @override
  String get fullType => 'nav_msgs/msg/OccupancyGrid';

  @override
  String get messageDefinition => '''std_msgs/Header header
MapMetaData info
int8[] data''';

  @override
  int getMessageSize() {
    return (header.getMessageSize() +
            info.getMessageSize() +
            4 +
            (data.length * 1))
        .toInt();
  }

  @override
  Map<String, dynamic> toJson() =>
      {'header': header.toJson(), 'info': info.toJson(), 'data': data};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  OccupancyGrid fromJson(Map<String, dynamic> jsonMap) {
    return OccupancyGrid(
        header: std_msgs.Header()
            .fromJson(jsonMap['header'] as Map<String, dynamic>),
        info: MapMetaData().fromJson(jsonMap['info'] as Map<String, dynamic>),
        data: (jsonMap['data'] as List).cast<int>());
  }
}
