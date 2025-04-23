import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class SetParamRequest extends RosMessage<SetParamRequest> {
  String name;
  String value;

  static SetParamRequest $prototype = SetParamRequest();

  SetParamRequest({this.name = '', this.value = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/SetParam_Request';

  @override
  String get messageDefinition => 'string name\nstring value';

  @override
  int getMessageSize() => name.length + value.length;

  @override
  Map<String, dynamic> toJson() => {'name': name, 'value': value};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  SetParamRequest fromJson(Map<String, dynamic> jsonMap) => SetParamRequest(
        name: jsonMap['name'] as String,
        value: jsonMap['value'] as String,
      );
}

class SetParamResponse extends RosMessage<SetParamResponse> {
  bool success;

  static SetParamResponse $prototype = SetParamResponse();

  SetParamResponse({this.success = false});

  @override
  String get fullType => 'rosapi_msgs/srv/SetParam_Response';

  @override
  String get messageDefinition => 'bool success';

  @override
  int getMessageSize() => 1;

  @override
  Map<String, dynamic> toJson() => {'success': success};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  SetParamResponse fromJson(Map<String, dynamic> jsonMap) => SetParamResponse(
        success: jsonMap['success'] as bool,
      );
}

class SetParam extends RosServiceMessage<SetParamRequest, SetParamResponse> {
  @override
  SetParamRequest get request => SetParamRequest();

  @override
  SetParamResponse get response => SetParamResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/SetParam';

  @override
  String get messageDefinition => '''string name
string value
---
bool success''';
}
