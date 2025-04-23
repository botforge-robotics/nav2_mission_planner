import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class HasParamRequest extends RosMessage<HasParamRequest> {
  String name;

  static HasParamRequest $prototype = HasParamRequest();

  HasParamRequest({this.name = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/HasParam_Request';

  @override
  String get messageDefinition => 'string name';

  @override
  int getMessageSize() => name.length;

  @override
  Map<String, dynamic> toJson() => {'name': name};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  HasParamRequest fromJson(Map<String, dynamic> jsonMap) => HasParamRequest(
        name: jsonMap['name'] as String,
      );
}

class HasParamResponse extends RosMessage<HasParamResponse> {
  bool exists;

  static HasParamResponse $prototype = HasParamResponse();

  HasParamResponse({this.exists = false});

  @override
  String get fullType => 'rosapi_msgs/srv/HasParam_Response';

  @override
  String get messageDefinition => 'bool exists';

  @override
  int getMessageSize() => 1;

  @override
  Map<String, dynamic> toJson() => {'exists': exists};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  HasParamResponse fromJson(Map<String, dynamic> jsonMap) => HasParamResponse(
        exists: jsonMap['exists'] as bool,
      );
}

class HasParam extends RosServiceMessage<HasParamRequest, HasParamResponse> {
  @override
  HasParamRequest get request => HasParamRequest();

  @override
  HasParamResponse get response => HasParamResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/HasParam';

  @override
  String get messageDefinition => '''string name
---
bool exists''';
}
