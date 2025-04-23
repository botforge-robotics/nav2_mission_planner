import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class GetTopicsRequest extends RosMessage<GetTopicsRequest> {
  static GetTopicsRequest $prototype = GetTopicsRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/GetTopics_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetTopicsRequest fromJson(Map<String, dynamic> jsonMap) => GetTopicsRequest();
}

class GetTopicsResponse extends RosMessage<GetTopicsResponse> {
  List<String> topics;

  static GetTopicsResponse $prototype = GetTopicsResponse();

  GetTopicsResponse({this.topics = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/GetTopics_Response';

  @override
  String get messageDefinition => 'string[] topics';

  @override
  int getMessageSize() =>
      topics.fold(0, (sum, topic) => sum + topic.length) + (4 * topics.length);

  @override
  Map<String, dynamic> toJson() => {'topics': topics};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetTopicsResponse fromJson(Map<String, dynamic> jsonMap) => GetTopicsResponse(
        topics: List<String>.from(jsonMap['topics']),
      );
}

class GetTopics extends RosServiceMessage<GetTopicsRequest, GetTopicsResponse> {
  @override
  GetTopicsRequest get request => GetTopicsRequest();

  @override
  GetTopicsResponse get response => GetTopicsResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/GetTopics';

  @override
  String get messageDefinition => '''---
string[] topics''';
}
