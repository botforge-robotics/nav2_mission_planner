// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class FrameGraphRequest extends RosMessage<FrameGraphRequest> {
  static FrameGraphRequest $prototype = FrameGraphRequest();

  FrameGraphRequest();

  @override
  String get fullType => 'tf2_msgs/srv/FrameGraph_Request';

  @override
  String get messageDefinition => '''''';

  @override
  int getMessageSize() {
    return 0;
  }

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  FrameGraphRequest fromJson(Map<String, dynamic> jsonMap) {
    return FrameGraphRequest();
  }
}

class FrameGraphResponse extends RosMessage<FrameGraphResponse> {
  late String frame_yaml;

  static FrameGraphResponse $prototype = FrameGraphResponse();

  FrameGraphResponse({String? frame_yaml}) : frame_yaml = frame_yaml ?? '';

  @override
  String get fullType => 'tf2_msgs/srv/FrameGraph_Response';

  @override
  String get messageDefinition => '''string frame_yaml''';

  @override
  int getMessageSize() {
    return 4 + frame_yaml.length;
  }

  @override
  Map<String, dynamic> toJson() => {'frame_yaml': frame_yaml};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  FrameGraphResponse fromJson(Map<String, dynamic> jsonMap) {
    return FrameGraphResponse(frame_yaml: jsonMap['frame_yaml'] as String);
  }
}

class FrameGraph
    extends RosServiceMessage<FrameGraphRequest, FrameGraphResponse> {
  @override
  FrameGraphRequest get request => FrameGraphRequest();

  @override
  FrameGraphResponse get response => FrameGraphResponse();

  @override
  String get fullType => 'tf2_msgs/srv/FrameGraph';

  @override
  String get messageDefinition => '''

---
string frame_yaml''';
}
