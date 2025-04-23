import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class GetParamNamesRequest extends RosMessage<GetParamNamesRequest> {
  static GetParamNamesRequest $prototype = GetParamNamesRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/GetParamNames_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetParamNamesRequest fromJson(Map<String, dynamic> jsonMap) =>
      GetParamNamesRequest();
}

class GetParamNamesResponse extends RosMessage<GetParamNamesResponse> {
  List<String> names;

  static GetParamNamesResponse $prototype = GetParamNamesResponse();

  GetParamNamesResponse({this.names = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/GetParamNames_Response';

  @override
  String get messageDefinition => 'string[] names';

  @override
  int getMessageSize() =>
      names.fold(0, (sum, name) => sum + name.length) + (4 * names.length);

  @override
  Map<String, dynamic> toJson() => {'names': names};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetParamNamesResponse fromJson(Map<String, dynamic> jsonMap) =>
      GetParamNamesResponse(
        names: List<String>.from(jsonMap['names']),
      );
}

class GetParamNames
    extends RosServiceMessage<GetParamNamesRequest, GetParamNamesResponse> {
  @override
  GetParamNamesRequest get request => GetParamNamesRequest();

  @override
  GetParamNamesResponse get response => GetParamNamesResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/GetParamNames';

  @override
  String get messageDefinition => '''---
string[] names''';
}
