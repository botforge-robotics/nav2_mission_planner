import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class TopicsForTypeRequest extends RosMessage<TopicsForTypeRequest> {
  String type;

  static TopicsForTypeRequest $prototype = TopicsForTypeRequest();

  TopicsForTypeRequest({this.type = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/TopicsForType_Request';

  @override
  String get messageDefinition => 'string type';

  @override
  int getMessageSize() => type.length;

  @override
  Map<String, dynamic> toJson() => {'type': type};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  TopicsForTypeRequest fromJson(Map<String, dynamic> jsonMap) =>
      TopicsForTypeRequest(
        type: jsonMap['type'] as String,
      );
}

class TopicsForTypeResponse extends RosMessage<TopicsForTypeResponse> {
  List<String> topics;

  static TopicsForTypeResponse $prototype = TopicsForTypeResponse();

  TopicsForTypeResponse({this.topics = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/TopicsForType_Response';

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
  TopicsForTypeResponse fromJson(Map<String, dynamic> jsonMap) =>
      TopicsForTypeResponse(
        topics: List<String>.from(jsonMap['topics']),
      );
}

class TopicsForType
    extends RosServiceMessage<TopicsForTypeRequest, TopicsForTypeResponse> {
  @override
  TopicsForTypeRequest get request => TopicsForTypeRequest();

  @override
  TopicsForTypeResponse get response => TopicsForTypeResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/TopicsForType';

  @override
  String get messageDefinition => '''string type
---
string[] topics''';
}
