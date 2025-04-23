import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class MessageDetailsRequest extends RosMessage<MessageDetailsRequest> {
  String type;

  static MessageDetailsRequest $prototype = MessageDetailsRequest();

  MessageDetailsRequest({this.type = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/MessageDetails_Request';

  @override
  String get messageDefinition => 'string type';

  @override
  int getMessageSize() => type.length;

  @override
  Map<String, dynamic> toJson() => {'type': type};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  MessageDetailsRequest fromJson(Map<String, dynamic> jsonMap) =>
      MessageDetailsRequest(
        type: jsonMap['type'] as String,
      );
}

class MessageDetailsResponse extends RosMessage<MessageDetailsResponse> {
  String message_definition;

  static MessageDetailsResponse $prototype = MessageDetailsResponse();

  MessageDetailsResponse({this.message_definition = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/MessageDetails_Response';

  @override
  String get messageDefinition => 'string message_definition';

  @override
  int getMessageSize() => message_definition.length;

  @override
  Map<String, dynamic> toJson() => {'message_definition': message_definition};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  MessageDetailsResponse fromJson(Map<String, dynamic> jsonMap) =>
      MessageDetailsResponse(
        message_definition: jsonMap['message_definition'] as String,
      );
}

class MessageDetails
    extends RosServiceMessage<MessageDetailsRequest, MessageDetailsResponse> {
  @override
  MessageDetailsRequest get request => MessageDetailsRequest();

  @override
  MessageDetailsResponse get response => MessageDetailsResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/MessageDetails';

  @override
  String get messageDefinition => '''string type
---
string message_definition''';
}
