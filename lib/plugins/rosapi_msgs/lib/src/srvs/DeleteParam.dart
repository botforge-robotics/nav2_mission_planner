import 'package:ros2_msg_utils/ros2_msg_utils.dart';
import 'dart:convert';

class DeleteParamRequest extends RosMessage<DeleteParamRequest> {
  String name;

  static DeleteParamRequest $prototype = DeleteParamRequest();

  DeleteParamRequest({this.name = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/DeleteParam_Request';

  @override
  String get messageDefinition => 'string name';

  @override
  int getMessageSize() => name.length;

  @override
  Map<String, dynamic> toJson() => {'name': name};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  DeleteParamRequest fromJson(Map<String, dynamic> jsonMap) =>
      DeleteParamRequest(name: jsonMap['name'] as String);
}

class DeleteParamResponse extends RosMessage<DeleteParamResponse> {
  static DeleteParamResponse $prototype = DeleteParamResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/DeleteParam_Response';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  DeleteParamResponse fromJson(Map<String, dynamic> jsonMap) =>
      DeleteParamResponse();
}

class DeleteParam
    extends RosServiceMessage<DeleteParamRequest, DeleteParamResponse> {
  @override
  DeleteParamRequest get request => DeleteParamRequest();

  @override
  DeleteParamResponse get response => DeleteParamResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/DeleteParam';

  @override
  String get messageDefinition => '''string name
---
''';
}
