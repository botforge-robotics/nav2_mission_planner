import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class InterfacesRequest extends RosMessage<InterfacesRequest> {
  static InterfacesRequest $prototype = InterfacesRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/Interfaces_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  InterfacesRequest fromJson(Map<String, dynamic> jsonMap) =>
      InterfacesRequest();
}

class InterfacesResponse extends RosMessage<InterfacesResponse> {
  List<String> interfaces;

  static InterfacesResponse $prototype = InterfacesResponse();

  InterfacesResponse({this.interfaces = const []});

  @override
  String get fullType => 'rosapi_msgs/srv/Interfaces_Response';

  @override
  String get messageDefinition => 'string[] interfaces';

  @override
  int getMessageSize() =>
      interfaces.fold(0, (sum, iface) => sum + iface.length) +
      (4 * interfaces.length);

  @override
  Map<String, dynamic> toJson() => {'interfaces': interfaces};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  InterfacesResponse fromJson(Map<String, dynamic> jsonMap) =>
      InterfacesResponse(
        interfaces: List<String>.from(jsonMap['interfaces']),
      );
}

class Interfaces
    extends RosServiceMessage<InterfacesRequest, InterfacesResponse> {
  @override
  InterfacesRequest get request => InterfacesRequest();

  @override
  InterfacesResponse get response => InterfacesResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/Interfaces';

  @override
  String get messageDefinition => '''---
string[] interfaces''';
}
