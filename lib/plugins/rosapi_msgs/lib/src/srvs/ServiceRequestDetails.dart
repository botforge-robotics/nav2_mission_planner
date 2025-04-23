import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class ServiceRequestDetailsRequest
    extends RosMessage<ServiceRequestDetailsRequest> {
  String service;

  static ServiceRequestDetailsRequest $prototype =
      ServiceRequestDetailsRequest();

  ServiceRequestDetailsRequest({this.service = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceRequestDetails_Request';

  @override
  String get messageDefinition => 'string service';

  @override
  int getMessageSize() => service.length;

  @override
  Map<String, dynamic> toJson() => {'service': service};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceRequestDetailsRequest fromJson(Map<String, dynamic> jsonMap) =>
      ServiceRequestDetailsRequest(
        service: jsonMap['service'] as String,
      );
}

class ServiceRequestDetailsResponse
    extends RosMessage<ServiceRequestDetailsResponse> {
  String details;

  static ServiceRequestDetailsResponse $prototype =
      ServiceRequestDetailsResponse();

  ServiceRequestDetailsResponse({this.details = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceRequestDetails_Response';

  @override
  String get messageDefinition => 'string details';

  @override
  int getMessageSize() => details.length;

  @override
  Map<String, dynamic> toJson() => {'details': details};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceRequestDetailsResponse fromJson(Map<String, dynamic> jsonMap) =>
      ServiceRequestDetailsResponse(
        details: jsonMap['details'] as String,
      );
}

class ServiceRequestDetails extends RosServiceMessage<
    ServiceRequestDetailsRequest, ServiceRequestDetailsResponse> {
  @override
  ServiceRequestDetailsRequest get request => ServiceRequestDetailsRequest();

  @override
  ServiceRequestDetailsResponse get response => ServiceRequestDetailsResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceRequestDetails';

  @override
  String get messageDefinition => '''string service
---
string details''';
}
