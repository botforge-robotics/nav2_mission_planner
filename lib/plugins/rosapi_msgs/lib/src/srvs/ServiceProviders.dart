import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class ServiceProvidersRequest extends RosMessage<ServiceProvidersRequest> {
  String service;

  static ServiceProvidersRequest $prototype = ServiceProvidersRequest();

  ServiceProvidersRequest({this.service = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceProviders_Request';

  @override
  String get messageDefinition => 'string service';

  @override
  int getMessageSize() => service.length;

  @override
  Map<String, dynamic> toJson() => {'service': service};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceProvidersRequest fromJson(Map<String, dynamic> jsonMap) =>
      ServiceProvidersRequest(
        service: jsonMap['service'] as String,
      );
}

class ServiceProvidersResponse extends RosMessage<ServiceProvidersResponse> {
  List<String> providers;

  static ServiceProvidersResponse $prototype = ServiceProvidersResponse();

  ServiceProvidersResponse({this.providers = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceProviders_Response';

  @override
  String get messageDefinition => 'string[] providers';

  @override
  int getMessageSize() =>
      providers.fold(0, (sum, provider) => sum + provider.length) +
      (4 * providers.length);

  @override
  Map<String, dynamic> toJson() => {'providers': providers};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceProvidersResponse fromJson(Map<String, dynamic> jsonMap) =>
      ServiceProvidersResponse(
        providers: List<String>.from(jsonMap['providers']),
      );
}

class ServiceProviders extends RosServiceMessage<ServiceProvidersRequest,
    ServiceProvidersResponse> {
  @override
  ServiceProvidersRequest get request => ServiceProvidersRequest();

  @override
  ServiceProvidersResponse get response => ServiceProvidersResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceProviders';

  @override
  String get messageDefinition => '''string service
---
string[] providers''';
}
