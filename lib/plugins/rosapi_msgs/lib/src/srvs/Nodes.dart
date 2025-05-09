import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class NodesRequest extends RosMessage<NodesRequest> {
  static NodesRequest $prototype = NodesRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/Nodes_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  NodesRequest fromJson(Map<String, dynamic> jsonMap) => NodesRequest();
}

class NodesResponse extends RosMessage<NodesResponse> {
  List<String> nodes;

  static NodesResponse $prototype = NodesResponse();

  NodesResponse({this.nodes = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/Nodes_Response';

  @override
  String get messageDefinition => 'string[] nodes';

  @override
  int getMessageSize() =>
      nodes.fold(0, (sum, node) => sum + node.length) + (4 * nodes.length);

  @override
  Map<String, dynamic> toJson() => {'nodes': nodes};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  NodesResponse fromJson(Map<String, dynamic> jsonMap) => NodesResponse(
        nodes: List<String>.from(jsonMap['nodes']),
      );
}

class Nodes extends RosServiceMessage<NodesRequest, NodesResponse> {
  @override
  NodesRequest get request => NodesRequest();

  @override
  NodesResponse get response => NodesResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/Nodes';

  @override
  String get messageDefinition => '''---
string[] nodes''';
}
