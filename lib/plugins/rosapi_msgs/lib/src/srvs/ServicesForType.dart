import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class ServicesForTypeRequest extends RosMessage<ServicesForTypeRequest> {
  String type;

  static ServicesForTypeRequest $prototype = ServicesForTypeRequest();

  ServicesForTypeRequest({this.type = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServicesForType_Request';

  @override
  String get messageDefinition => 'string type';

  @override
  int getMessageSize() => type.length;

  @override
  Map<String, dynamic> toJson() => {'type': type};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServicesForTypeRequest fromJson(Map<String, dynamic> jsonMap) =>
      ServicesForTypeRequest(
        type: jsonMap['type'] as String,
      );
}

class ServicesForTypeResponse extends RosMessage<ServicesForTypeResponse> {
  List<String> services;

  static ServicesForTypeResponse $prototype = ServicesForTypeResponse();

  ServicesForTypeResponse({this.services = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/ServicesForType_Response';

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
  ServicesForTypeResponse fromJson(Map<String, dynamic> jsonMap) =>
      ServicesForTypeResponse(
        services: List<String>.from(jsonMap['services']),
      );
}

class ServicesForType
    extends RosServiceMessage<ServicesForTypeRequest, ServicesForTypeResponse> {
  @override
  ServicesForTypeRequest get request => ServicesForTypeRequest();

  @override
  ServicesForTypeResponse get response => ServicesForTypeResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/ServicesForType';

  @override
  String get messageDefinition => '''string type
---
string[] services''';
}
