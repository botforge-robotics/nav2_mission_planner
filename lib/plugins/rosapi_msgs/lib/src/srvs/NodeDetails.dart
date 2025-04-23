import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class NodeDetailsRequest extends RosMessage<NodeDetailsRequest> {
  String node;

  static NodeDetailsRequest $prototype = NodeDetailsRequest();

  NodeDetailsRequest({this.node = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/NodeDetails_Request';

  @override
  String get messageDefinition => 'string node';

  @override
  int getMessageSize() => node.length;

  @override
  Map<String, dynamic> toJson() => {'node': node};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  NodeDetailsRequest fromJson(Map<String, dynamic> jsonMap) =>
      NodeDetailsRequest(
        node: jsonMap['node'] as String,
      );
}

class NodeDetailsResponse extends RosMessage<NodeDetailsResponse> {
  String details;

  static NodeDetailsResponse $prototype = NodeDetailsResponse();

  NodeDetailsResponse({this.details = ''});

  @override
  String get fullType => 'rosapi_msgs/srv/NodeDetails_Response';

  @override
  String get messageDefinition => 'string details';

  @override
  int getMessageSize() => details.length;

  @override
  Map<String, dynamic> toJson() => {'details': details};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  NodeDetailsResponse fromJson(Map<String, dynamic> jsonMap) =>
      NodeDetailsResponse(
        details: jsonMap['details'] as String,
      );
}

class NodeDetails
    extends RosServiceMessage<NodeDetailsRequest, NodeDetailsResponse> {
  @override
  NodeDetailsRequest get request => NodeDetailsRequest();

  @override
  NodeDetailsResponse get response => NodeDetailsResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/NodeDetails';

  @override
  String get messageDefinition => '''string node
---
string details''';
}
