import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class ServiceNodeRequest extends RosMessage<ServiceNodeRequest> {
  String node;

  static ServiceNodeRequest $prototype = ServiceNodeRequest();

  ServiceNodeRequest({this.node = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceNode_Request';

  @override
  String get messageDefinition => 'string node';

  @override
  int getMessageSize() => node.length;

  @override
  Map<String, dynamic> toJson() => {'node': node};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ServiceNodeRequest fromJson(Map<String, dynamic> jsonMap) =>
      ServiceNodeRequest(
        node: jsonMap['node'] as String,
      );
}

class ServiceNodeResponse extends RosMessage<ServiceNodeResponse> {
  List<String> services;

  static ServiceNodeResponse $prototype = ServiceNodeResponse();

  ServiceNodeResponse({this.services = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceNode_Response';

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
  ServiceNodeResponse fromJson(Map<String, dynamic> jsonMap) =>
      ServiceNodeResponse(
        services: List<String>.from(jsonMap['services']),
      );
}

class ServiceNode
    extends RosServiceMessage<ServiceNodeRequest, ServiceNodeResponse> {
  @override
  ServiceNodeRequest get request => ServiceNodeRequest();

  @override
  ServiceNodeResponse get response => ServiceNodeResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/ServiceNode';

  @override
  String get messageDefinition => '''string node
---
string[] services''';
}
