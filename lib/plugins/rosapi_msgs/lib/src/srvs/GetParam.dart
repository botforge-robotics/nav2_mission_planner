import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class GetParamRequest extends RosMessage<GetParamRequest> {
  String name;
  String default_value;

  static GetParamRequest $prototype = GetParamRequest();

  GetParamRequest({this.name = '', this.default_value = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/GetParam_Request';

  @override
  String get messageDefinition => 'string name\nstring default_value';

  @override
  int getMessageSize() => name.length + default_value.length;

  @override
  Map<String, dynamic> toJson() => {
        'name': name,
        'default_value': default_value,
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetParamRequest fromJson(Map<String, dynamic> jsonMap) => GetParamRequest(
        name: jsonMap['name'] as String,
        default_value: jsonMap['default_value'] as String,
      );
}

class GetParamResponse extends RosMessage<GetParamResponse> {
  String value;

  static GetParamResponse $prototype = GetParamResponse();

  GetParamResponse({this.value = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/GetParam_Response';

  @override
  String get messageDefinition => 'string value';

  @override
  int getMessageSize() => value.length;

  @override
  Map<String, dynamic> toJson() => {'value': value};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetParamResponse fromJson(Map<String, dynamic> jsonMap) => GetParamResponse(
        value: jsonMap['value'] as String,
      );
}

class GetParam extends RosServiceMessage<GetParamRequest, GetParamResponse> {
  @override
  GetParamRequest get request => GetParamRequest();

  @override
  GetParamResponse get response => GetParamResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/GetParam';

  @override
  String get messageDefinition => '''string name
string default_value
---
string value''';
}
