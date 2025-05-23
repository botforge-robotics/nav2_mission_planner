// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';
import 'package:geometry_msgs/msg.dart' as geometry_msgs;

class TFMessage extends RosMessage<TFMessage> {
  late List<geometry_msgs.TransformStamped> transforms;

  static TFMessage $prototype = TFMessage();

  // Constructor
  TFMessage({List<geometry_msgs.TransformStamped>? transforms})
      : transforms = transforms ?? const [];

  @override
  String get fullType => 'tf2_msgs/msg/TFMessage';

  @override
  String get messageDefinition =>
      '''geometry_msgs/TransformStamped[] transforms''';

  @override
  int getMessageSize() {
    return 4 +
        transforms.fold<int>(0, (sum, item) => sum + (item.getMessageSize()));
  }

  @override
  Map<String, dynamic> toJson() =>
      {'transforms': transforms.map((item) => item.toJson()).toList()};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  TFMessage fromJson(Map<String, dynamic> jsonMap) {
    return TFMessage(
        transforms: (jsonMap['transforms'] as List<dynamic>)
            .map((item) => geometry_msgs.TransformStamped()
                .fromJson(item as Map<String, dynamic>))
            .toList());
  }
}
