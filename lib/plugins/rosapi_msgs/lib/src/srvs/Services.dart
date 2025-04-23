import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class ServicesRequest extends RosMessage<ServicesRequest> {
  static ServicesRequest $prototype = ServicesRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/Services_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServicesRequest fromJson(Map<String, dynamic> jsonMap) => ServicesRequest();
}

class ServicesResponse extends RosMessage<ServicesResponse> {
  List<String> services;

  static ServicesResponse $prototype = ServicesResponse();

  ServicesResponse({this.services = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/Services_Response';

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
  ServicesResponse fromJson(Map<String, dynamic> jsonMap) => ServicesResponse(
        services: List<String>.from(jsonMap['services']),
      );
}

class Services extends RosServiceMessage<ServicesRequest, ServicesResponse> {
  @override
  ServicesRequest get request => ServicesRequest();

  @override
  ServicesResponse get response => ServicesResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/Services';

  @override
  String get messageDefinition => '''---
string[] services''';
}
