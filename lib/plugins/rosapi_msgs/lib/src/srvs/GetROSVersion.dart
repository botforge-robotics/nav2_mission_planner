import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class GetROSVersionRequest extends RosMessage<GetROSVersionRequest> {
  static GetROSVersionRequest $prototype = GetROSVersionRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/GetROSVersion_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetROSVersionRequest fromJson(Map<String, dynamic> jsonMap) =>
      GetROSVersionRequest();
}

class GetROSVersionResponse extends RosMessage<GetROSVersionResponse> {
  String version;

  static GetROSVersionResponse $prototype = GetROSVersionResponse();

  GetROSVersionResponse({this.version = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/GetROSVersion_Response';

  @override
  String get messageDefinition => 'string version';

  @override
  int getMessageSize() => version.length;

  @override
  Map<String, dynamic> toJson() => {'version': version};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetROSVersionResponse fromJson(Map<String, dynamic> jsonMap) =>
      GetROSVersionResponse(
        version: jsonMap['version'] as String,
      );
}

class GetROSVersion
    extends RosServiceMessage<GetROSVersionRequest, GetROSVersionResponse> {
  @override
  GetROSVersionRequest get request => GetROSVersionRequest();

  @override
  GetROSVersionResponse get response => GetROSVersionResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/GetROSVersion';

  @override
  String get messageDefinition => '''---
string version''';
}
