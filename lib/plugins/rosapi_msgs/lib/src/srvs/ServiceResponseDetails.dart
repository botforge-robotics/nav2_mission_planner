import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class ServiceResponseDetailsRequest
    extends RosMessage<ServiceResponseDetailsRequest> {
  String service;

  static ServiceResponseDetailsRequest $prototype =
      ServiceResponseDetailsRequest();

  ServiceResponseDetailsRequest({this.service = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceResponseDetails_Request';

  @override
  String get messageDefinition => 'string service';

  @override
  int getMessageSize() => service.length;

  @override
  Map<String, dynamic> toJson() => {'service': service};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceResponseDetailsRequest fromJson(Map<String, dynamic> jsonMap) =>
      ServiceResponseDetailsRequest(
        service: jsonMap['service'] as String,
      );
}

class ServiceResponseDetailsResponse
    extends RosMessage<ServiceResponseDetailsResponse> {
  String details;

  static ServiceResponseDetailsResponse $prototype =
      ServiceResponseDetailsResponse();

  ServiceResponseDetailsResponse({this.details = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceResponseDetails_Response';

  @override
  String get messageDefinition => 'string details';

  @override
  int getMessageSize() => details.length;

  @override
  Map<String, dynamic> toJson() => {'details': details};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceResponseDetailsResponse fromJson(Map<String, dynamic> jsonMap) =>
      ServiceResponseDetailsResponse(
        details: jsonMap['details'] as String,
      );
}

class ServiceResponseDetails extends RosServiceMessage<
    ServiceResponseDetailsRequest, ServiceResponseDetailsResponse> {
  @override
  ServiceResponseDetailsRequest get request => ServiceResponseDetailsRequest();

  @override
  ServiceResponseDetailsResponse get response =>
      ServiceResponseDetailsResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceResponseDetails';

  @override
  String get messageDefinition => '''string service
---
string details''';
}
