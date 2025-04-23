import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class ServiceTypeRequest extends RosMessage<ServiceTypeRequest> {
  String service;

  static ServiceTypeRequest $prototype = ServiceTypeRequest();

  ServiceTypeRequest({this.service = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceType_Request';

  @override
  String get messageDefinition => 'string service';

  @override
  int getMessageSize() => service.length;

  @override
  Map<String, dynamic> toJson() => {'service': service};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceTypeRequest fromJson(Map<String, dynamic> jsonMap) =>
      ServiceTypeRequest(
        service: jsonMap['service'] as String,
      );
}

class ServiceTypeResponse extends RosMessage<ServiceTypeResponse> {
  String type;

  static ServiceTypeResponse $prototype = ServiceTypeResponse();

  ServiceTypeResponse({this.type = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceType_Response';

  @override
  String get messageDefinition => 'string type';

  @override
  int getMessageSize() => type.length;

  @override
  Map<String, dynamic> toJson() => {'type': type};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceTypeResponse fromJson(Map<String, dynamic> jsonMap) =>
      ServiceTypeResponse(
        type: jsonMap['type'] as String,
      );
}

class ServiceType
    extends RosServiceMessage<ServiceTypeRequest, ServiceTypeResponse> {
  @override
  ServiceTypeRequest get request => ServiceTypeRequest();

  @override
  ServiceTypeResponse get response => ServiceTypeResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceType';

  @override
  String get messageDefinition => '''string service
---
string type''';
}
