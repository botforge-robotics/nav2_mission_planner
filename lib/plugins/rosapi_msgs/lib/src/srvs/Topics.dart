import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class TopicsRequest extends RosMessage<TopicsRequest> {
  static TopicsRequest $prototype = TopicsRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/Topics_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  TopicsRequest fromJson(Map<String, dynamic> jsonMap) => TopicsRequest();
}

class TopicsResponse extends RosMessage<TopicsResponse> {
  List<String> topics;

  static TopicsResponse $prototype = TopicsResponse();

  TopicsResponse({this.topics = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/Topics_Response';

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
  TopicsResponse fromJson(Map<String, dynamic> jsonMap) => TopicsResponse(
        topics: List<String>.from(jsonMap['topics']),
      );
}

class Topics extends RosServiceMessage<TopicsRequest, TopicsResponse> {
  @override
  TopicsRequest get request => TopicsRequest();

  @override
  TopicsResponse get response => TopicsResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/Topics';

  @override
  String get messageDefinition => '''---
string[] topics''';
}
