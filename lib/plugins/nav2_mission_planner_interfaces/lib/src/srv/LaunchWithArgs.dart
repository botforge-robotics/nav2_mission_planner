// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class LaunchWithArgsRequest extends RosMessage<LaunchWithArgsRequest> {
  late String package;
  late String launch_file;
  late String arguments;

  static LaunchWithArgsRequest $prototype = LaunchWithArgsRequest();

  LaunchWithArgsRequest(
      {String? package, String? launch_file, String? arguments})
      : package = package ?? '',
        launch_file = launch_file ?? '',
        arguments = arguments ?? '';

  @override
  String get fullType =>
      'nav2_mission_planner_interfaces/srv/LaunchWithArgs_Request';

  @override
  String get messageDefinition => '''string package
string launch_file
string arguments''';

  @override
  int getMessageSize() {
    return 4 + package.length + 4 + launch_file.length + 4 + arguments.length;
  }

  @override
  Map<String, dynamic> toJson() =>
      {'package': package, 'launch_file': launch_file, 'arguments': arguments};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  LaunchWithArgsRequest fromJson(Map<String, dynamic> jsonMap) {
    return LaunchWithArgsRequest(
        package: jsonMap['package'] as String,
        launch_file: jsonMap['launch_file'] as String,
        arguments: jsonMap['arguments'] as String);
  }
}

class LaunchWithArgsResponse extends RosMessage<LaunchWithArgsResponse> {
  late bool success;
  late String message;
  late String unique_id;

  static LaunchWithArgsResponse $prototype = LaunchWithArgsResponse();

  LaunchWithArgsResponse({bool? success, String? message, String? unique_id})
      : success = success ?? false,
        message = message ?? '',
        unique_id = unique_id ?? '';

  @override
  String get fullType =>
      'nav2_mission_planner_interfaces/srv/LaunchWithArgs_Response';

  @override
  String get messageDefinition => '''bool success
string message
string unique_id''';

  @override
  int getMessageSize() {
    return 1 + 4 + message.length + 4 + unique_id.length;
  }

  @override
  Map<String, dynamic> toJson() =>
      {'success': success, 'message': message, 'unique_id': unique_id};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  LaunchWithArgsResponse fromJson(Map<String, dynamic> jsonMap) {
    return LaunchWithArgsResponse(
        success: jsonMap['success'] as bool,
        message: jsonMap['message'] as String,
        unique_id: jsonMap['unique_id'] as String);
  }
}

class LaunchWithArgs
    extends RosServiceMessage<LaunchWithArgsRequest, LaunchWithArgsResponse> {
  @override
  LaunchWithArgsRequest get request => LaunchWithArgsRequest();

  @override
  LaunchWithArgsResponse get response => LaunchWithArgsResponse();

  @override
  String get fullType => 'nav2_mission_planner_interfaces/srv/LaunchWithArgs';

  @override
  String get messageDefinition => '''
string package
string launch_file
string arguments
---
bool success
string message
string unique_id''';
}
