import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class SubscribersRequest extends RosMessage<SubscribersRequest> {
  String topic;

  static SubscribersRequest $prototype = SubscribersRequest();

  SubscribersRequest({this.topic = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/Subscribers_Request';

  @override
  String get messageDefinition => 'string topic';

  @override
  int getMessageSize() => topic.length;

  @override
  Map<String, dynamic> toJson() => {'topic': topic};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  SubscribersRequest fromJson(Map<String, dynamic> jsonMap) =>
      SubscribersRequest(
        topic: jsonMap['topic'] as String,
      );
}

class SubscribersResponse extends RosMessage<SubscribersResponse> {
  List<String> subscribers;

  static SubscribersResponse $prototype = SubscribersResponse();

  SubscribersResponse({this.subscribers = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/Subscribers_Response';

  @override
  String get messageDefinition => 'string[] subscribers';

  @override
  int getMessageSize() =>
      subscribers.fold(0, (sum, sub) => sum + sub.length) +
      (4 * subscribers.length);

  @override
  Map<String, dynamic> toJson() => {'subscribers': subscribers};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  SubscribersResponse fromJson(Map<String, dynamic> jsonMap) =>
      SubscribersResponse(
        subscribers: List<String>.from(jsonMap['subscribers']),
      );
}

class Subscribers
    extends RosServiceMessage<SubscribersRequest, SubscribersResponse> {
  @override
  SubscribersRequest get request => SubscribersRequest();

  @override
  SubscribersResponse get response => SubscribersResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/Subscribers';

  @override
  String get messageDefinition => '''string topic
---
string[] subscribers''';
}
