import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class TopicTypeRequest extends RosMessage<TopicTypeRequest> {
  String topic;

  static TopicTypeRequest $prototype = TopicTypeRequest();

  TopicTypeRequest({this.topic = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/TopicType_Request';

  @override
  String get messageDefinition => 'string topic';

  @override
  int getMessageSize() => topic.length;

  @override
  Map<String, dynamic> toJson() => {'topic': topic};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  TopicTypeRequest fromJson(Map<String, dynamic> jsonMap) => TopicTypeRequest(
        topic: jsonMap['topic'] as String,
      );
}

class TopicTypeResponse extends RosMessage<TopicTypeResponse> {
  String type;

  static TopicTypeResponse $prototype = TopicTypeResponse();

  TopicTypeResponse({this.type = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/TopicType_Response';

  @override
  String get messageDefinition => 'string type';

  @override
  int getMessageSize() => type.length;

  @override
  Map<String, dynamic> toJson() => {'type': type};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  TopicTypeResponse fromJson(Map<String, dynamic> jsonMap) => TopicTypeResponse(
        type: jsonMap['type'] as String,
      );
}

class TopicType extends RosServiceMessage<TopicTypeRequest, TopicTypeResponse> {
  @override
  TopicTypeRequest get request => TopicTypeRequest();

  @override
  TopicTypeResponse get response => TopicTypeResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/TopicType';

  @override
  String get messageDefinition => '''string topic
---
string type''';
}
