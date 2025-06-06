// Generated by ros_to_dart_converter
// Do not edit manually

import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class TriggerRequest extends RosMessage<TriggerRequest> {
  static TriggerRequest $prototype = TriggerRequest();

  // Constructor
  TriggerRequest();

  @override
  String get fullType => 'std_srvs/srv/Trigger_Request';

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
  TriggerRequest fromJson(Map<String, dynamic> jsonMap) {
    return TriggerRequest();
  }
}

class TriggerResponse extends RosMessage<TriggerResponse> {
  bool success;
  String message;

  static TriggerResponse $prototype = TriggerResponse();

  // Constructor
  TriggerResponse({this.success = false, this.message = ''});

  @override
  String get fullType => 'std_srvs/srv/Trigger_Response';

  @override
  String get messageDefinition => '''bool success
string message''';

  @override
  int getMessageSize() {
    return (1 + 4 + message.length).toInt();
  }

  @override
  Map<String, dynamic> toJson() => {'success': success, 'message': message};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  TriggerResponse fromJson(Map<String, dynamic> jsonMap) {
    return TriggerResponse(
        success: jsonMap['success'] as bool,
        message: jsonMap['message'] as String);
  }
}

class Trigger extends RosServiceMessage<TriggerRequest, TriggerResponse> {
  @override
  TriggerRequest get request => TriggerRequest();

  @override
  TriggerResponse get response => TriggerResponse();

  @override
  String get fullType => 'std_srvs/srv/Trigger';

  @override
  String get messageDefinition => '''
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages''';
}
