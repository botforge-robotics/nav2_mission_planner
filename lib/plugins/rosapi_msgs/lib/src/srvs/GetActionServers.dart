import 'package:ros2_msg_utils/ros2_msg_utils.dart';
import 'dart:convert';

class GetActionServersRequest extends RosMessage<GetActionServersRequest> {
  static GetActionServersRequest $prototype = GetActionServersRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/GetActionServers_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetActionServersRequest fromJson(Map<String, dynamic> jsonMap) =>
      GetActionServersRequest();
}

class GetActionServersResponse extends RosMessage<GetActionServersResponse> {
  List<String> action_servers;

  static GetActionServersResponse $prototype = GetActionServersResponse();

  GetActionServersResponse({this.action_servers = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/GetActionServers_Response';

  @override
  String get messageDefinition => 'string[] action_servers';

  @override
  int getMessageSize() =>
      action_servers.fold(0, (sum, server) => sum + server.length) +
      (4 * action_servers.length);

  @override
  Map<String, dynamic> toJson() => {'action_servers': action_servers};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetActionServersResponse fromJson(Map<String, dynamic> jsonMap) =>
      GetActionServersResponse(
        action_servers: List<String>.from(jsonMap['action_servers']),
      );
}

class GetActionServers extends RosServiceMessage<GetActionServersRequest,
    GetActionServersResponse> {
  @override
  GetActionServersRequest get request => GetActionServersRequest();

  @override
  GetActionServersResponse get response => GetActionServersResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/GetActionServers';

  @override
  String get messageDefinition => '''---
string[] action_servers''';
}
