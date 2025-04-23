import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class TopicsAndRawTypesRequest extends RosMessage<TopicsAndRawTypesRequest> {
  static TopicsAndRawTypesRequest $prototype = TopicsAndRawTypesRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/TopicsAndRawTypes_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  TopicsAndRawTypesRequest fromJson(Map<String, dynamic> jsonMap) =>
      TopicsAndRawTypesRequest();
}

class TopicsAndRawTypesResponse extends RosMessage<TopicsAndRawTypesResponse> {
  List<String> topics;
  List<String> types;

  static TopicsAndRawTypesResponse $prototype = TopicsAndRawTypesResponse();

  TopicsAndRawTypesResponse({this.topics = const [], this.types = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/TopicsAndRawTypes_Response';

  @override
  String get messageDefinition => 'string[] topics\nstring[] types';

  @override
  int getMessageSize() =>
      (topics.fold(0, (sum, topic) => sum + topic.length) +
          (4 * topics.length)) +
      (types.fold(0, (sum, type) => sum + type.length) + (4 * types.length));

  @override
  Map<String, dynamic> toJson() => {'topics': topics, 'types': types};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  TopicsAndRawTypesResponse fromJson(Map<String, dynamic> jsonMap) =>
      TopicsAndRawTypesResponse(
        topics: List<String>.from(jsonMap['topics']),
        types: List<String>.from(jsonMap['types']),
      );
}

class TopicsAndRawTypes extends RosServiceMessage<TopicsAndRawTypesRequest,
    TopicsAndRawTypesResponse> {
  @override
  TopicsAndRawTypesRequest get request => TopicsAndRawTypesRequest();

  @override
  TopicsAndRawTypesResponse get response => TopicsAndRawTypesResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/TopicsAndRawTypes';

  @override
  String get messageDefinition => '''---
string[] topics
string[] types''';
}
