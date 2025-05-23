// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';
import 'package:std_msgs/msg.dart' as std_msgs;
import 'package:actionlib_msgs/msg.dart' as actionlib_msgs;
import 'package:builtin_interfaces/msg.dart' as builtin_interfaces;
import 'package:geometry_msgs/msg.dart' as geometry_msgs;

class BackUpGoal extends RosMessage<BackUpGoal> {
  late geometry_msgs.Point target;
  late double speed;
  late builtin_interfaces.Duration time_allowance;

  static BackUpGoal $prototype = BackUpGoal();

  BackUpGoal(
      {geometry_msgs.Point? target,
      double? speed,
      builtin_interfaces.Duration? time_allowance})
      : target = target ?? geometry_msgs.Point(),
        speed = speed ?? 0.0,
        time_allowance = time_allowance ?? builtin_interfaces.Duration();

  @override
  String get fullType => 'nav2_msgs/action/BackUp_Goal';

  @override
  String get messageDefinition => '''geometry_msgs/Point target
float32 speed
builtin_interfaces/Duration time_allowance''';

  @override
  int getMessageSize() {
    return (target.getMessageSize()) + 4 + (time_allowance.getMessageSize());
  }

  @override
  Map<String, dynamic> toJson() => {
        'target': target.toJson(),
        'speed': speed,
        'time_allowance': time_allowance.toJson()
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  BackUpGoal fromJson(Map<String, dynamic> jsonMap) {
    return BackUpGoal(
        target: geometry_msgs.Point()
            .fromJson(jsonMap['target'] as Map<String, dynamic>),
        speed: jsonMap['speed'] as double,
        time_allowance: builtin_interfaces.Duration()
            .fromJson(jsonMap['time_allowance'] as Map<String, dynamic>));
  }
}

class BackUpResult extends RosMessage<BackUpResult> {
  late builtin_interfaces.Duration total_elapsed_time;
  late int error_code;

  static BackUpResult $prototype = BackUpResult();

  BackUpResult(
      {builtin_interfaces.Duration? total_elapsed_time, int? error_code})
      : total_elapsed_time =
            total_elapsed_time ?? builtin_interfaces.Duration(),
        error_code = error_code ?? 0;

  @override
  String get fullType => 'nav2_msgs/action/BackUp_Result';

  @override
  String get messageDefinition =>
      '''builtin_interfaces/Duration total_elapsed_time
uint16 error_code''';

  @override
  int getMessageSize() {
    return (total_elapsed_time.getMessageSize()) + 2;
  }

  @override
  Map<String, dynamic> toJson() => {
        'total_elapsed_time': total_elapsed_time.toJson(),
        'error_code': error_code
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  BackUpResult fromJson(Map<String, dynamic> jsonMap) {
    return BackUpResult(
        total_elapsed_time: builtin_interfaces.Duration()
            .fromJson(jsonMap['total_elapsed_time'] as Map<String, dynamic>),
        error_code: jsonMap['error_code'] as int);
  }
}

class BackUpFeedback extends RosMessage<BackUpFeedback> {
  late double distance_traveled;

  static BackUpFeedback $prototype = BackUpFeedback();

  BackUpFeedback({double? distance_traveled})
      : distance_traveled = distance_traveled ?? 0.0;

  @override
  String get fullType => 'nav2_msgs/action/BackUp_Feedback';

  @override
  String get messageDefinition => '''float32 distance_traveled''';

  @override
  int getMessageSize() {
    return 4;
  }

  @override
  Map<String, dynamic> toJson() => {'distance_traveled': distance_traveled};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  BackUpFeedback fromJson(Map<String, dynamic> jsonMap) {
    return BackUpFeedback(
        distance_traveled: jsonMap['distance_traveled'] as double);
  }
}

class BackUpActionGoal extends RosActionGoal<BackUpGoal, BackUpActionGoal> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalID goal_id;
  @override
  late BackUpGoal goal;

  static BackUpActionGoal $prototype = BackUpActionGoal();

  BackUpActionGoal({
    std_msgs.Header? header,
    actionlib_msgs.GoalID? goal_id,
    BackUpGoal? goal,
  })  : header = header ?? std_msgs.Header(),
        goal_id = goal_id ?? actionlib_msgs.GoalID(),
        goal = goal ?? BackUpGoal();

  @override
  String get fullType => 'nav2_msgs/action/BackUp_ActionGoal';

  @override
  String get messageDefinition => '''geometry_msgs/Point target
float32 speed
builtin_interfaces/Duration time_allowance''';

  @override
  int getMessageSize() {
    return header.getMessageSize() +
        goal_id.getMessageSize() +
        goal.getMessageSize();
  }

  @override
  Map<String, dynamic> toJson() => {
        'header': header.toJson(),
        'goal_id': goal_id.toJson(),
        'goal': goal.toJson(),
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  BackUpActionGoal fromJson(Map<String, dynamic> jsonMap) {
    return BackUpActionGoal(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      goal_id: actionlib_msgs.GoalID()
          .fromJson(jsonMap['goal_id'] as Map<String, dynamic>),
      goal: BackUpGoal().fromJson(jsonMap['goal'] as Map<String, dynamic>),
    );
  }
}

class BackUpActionResult
    extends RosActionResult<BackUpResult, BackUpActionResult> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalStatus status;
  @override
  late BackUpResult result;

  static BackUpActionResult $prototype = BackUpActionResult();

  BackUpActionResult({
    std_msgs.Header? header,
    actionlib_msgs.GoalStatus? status,
    BackUpResult? result,
  })  : header = header ?? std_msgs.Header(),
        status = status ?? actionlib_msgs.GoalStatus(),
        result = result ?? BackUpResult();

  @override
  String get fullType => 'nav2_msgs/action/BackUp_ActionResult';

  @override
  String get messageDefinition =>
      '''builtin_interfaces/Duration total_elapsed_time
uint16 error_code''';

  @override
  int getMessageSize() {
    return header.getMessageSize() +
        status.getMessageSize() +
        result.getMessageSize();
  }

  @override
  Map<String, dynamic> toJson() => {
        'header': header.toJson(),
        'status': status.toJson(),
        'result': result.toJson(),
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  BackUpActionResult fromJson(Map<String, dynamic> jsonMap) {
    return BackUpActionResult(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      status: actionlib_msgs.GoalStatus()
          .fromJson(jsonMap['status'] as Map<String, dynamic>),
      result:
          BackUpResult().fromJson(jsonMap['result'] as Map<String, dynamic>),
    );
  }
}

class BackUpActionFeedback
    extends RosActionFeedback<BackUpFeedback, BackUpActionFeedback> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalStatus status;
  @override
  late BackUpFeedback feedback;

  static BackUpActionFeedback $prototype = BackUpActionFeedback();

  BackUpActionFeedback({
    std_msgs.Header? header,
    actionlib_msgs.GoalStatus? status,
    BackUpFeedback? feedback,
  })  : header = header ?? std_msgs.Header(),
        status = status ?? actionlib_msgs.GoalStatus(),
        feedback = feedback ?? BackUpFeedback();

  @override
  String get fullType => 'nav2_msgs/action/BackUp_ActionFeedback';

  @override
  String get messageDefinition => '''float32 distance_traveled''';

  @override
  int getMessageSize() {
    return header.getMessageSize() +
        status.getMessageSize() +
        feedback.getMessageSize();
  }

  @override
  Map<String, dynamic> toJson() => {
        'header': header.toJson(),
        'status': status.toJson(),
        'feedback': feedback.toJson(),
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  BackUpActionFeedback fromJson(Map<String, dynamic> jsonMap) {
    return BackUpActionFeedback(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      status: actionlib_msgs.GoalStatus()
          .fromJson(jsonMap['status'] as Map<String, dynamic>),
      feedback: BackUpFeedback()
          .fromJson(jsonMap['feedback'] as Map<String, dynamic>),
    );
  }
}

class BackUp extends RosActionMessage<BackUpGoal, BackUpActionGoal,
    BackUpFeedback, BackUpActionFeedback, BackUpResult, BackUpActionResult> {
  @override
  BackUpActionGoal get actionGoal => BackUpActionGoal();

  @override
  BackUpActionFeedback get actionFeedback => BackUpActionFeedback();

  @override
  BackUpActionResult get actionResult => BackUpActionResult();

  @override
  BackUpGoal get goal => BackUpGoal();

  @override
  BackUpFeedback get feedback => BackUpFeedback();

  @override
  BackUpResult get result => BackUpResult();

  @override
  String get fullType => 'nav2_msgs/action/BackUp';

  @override
  String get messageDefinition => '''
Goal:
geometry_msgs/Point target
float32 speed
builtin_interfaces/Duration time_allowance
---
Result:
builtin_interfaces/Duration total_elapsed_time
uint16 error_code
---
Feedback:
float32 distance_traveled''';
}
