import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class GetServicesRequest extends RosMessage<GetServicesRequest> {
  static GetServicesRequest $prototype = GetServicesRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/GetServices_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetServicesRequest fromJson(Map<String, dynamic> jsonMap) =>
      GetServicesRequest();
}

class GetServicesResponse extends RosMessage<GetServicesResponse> {
  List<String> services;

  static GetServicesResponse $prototype = GetServicesResponse();

  GetServicesResponse({this.services = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/GetServices_Response';

  @override
  String get messageDefinition => 'string[] services';

  @override
  int getMessageSize() =>
      services.fold(0, (sum, service) => sum + service.length) +
      (4 * services.length);

  @override
  Map<String, dynamic> toJson() => {'services': services};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetServicesResponse fromJson(Map<String, dynamic> jsonMap) =>
      GetServicesResponse(
        services: List<String>.from(jsonMap['services']),
      );
}

class GetServices
    extends RosServiceMessage<GetServicesRequest, GetServicesResponse> {
  @override
  GetServicesRequest get request => GetServicesRequest();

  @override
  GetServicesResponse get response => GetServicesResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/GetServices';

  @override
  String get messageDefinition => '''---
string[] services''';
}
