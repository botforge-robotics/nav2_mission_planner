// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';
import 'package:std_msgs/msg.dart' as std_msgs;
import 'package:actionlib_msgs/msg.dart' as actionlib_msgs;
import 'package:builtin_interfaces/msg.dart' as builtin_interfaces;
import 'package:nav_msgs/msg.dart' as nav_msgs;

class SmoothPathGoal extends RosMessage<SmoothPathGoal> {
  late nav_msgs.Path path;
  late String smoother_id;
  late builtin_interfaces.Duration max_smoothing_duration;
  late bool check_for_collisions;

  static SmoothPathGoal $prototype = SmoothPathGoal();

  SmoothPathGoal(
      {nav_msgs.Path? path,
      String? smoother_id,
      builtin_interfaces.Duration? max_smoothing_duration,
      bool? check_for_collisions})
      : path = path ?? nav_msgs.Path(),
        smoother_id = smoother_id ?? '',
        max_smoothing_duration =
            max_smoothing_duration ?? builtin_interfaces.Duration(),
        check_for_collisions = check_for_collisions ?? false;

  @override
  String get fullType => 'nav2_msgs/action/SmoothPath_Goal';

  @override
  String get messageDefinition => '''nav_msgs/Path path
string smoother_id
builtin_interfaces/Duration max_smoothing_duration
bool check_for_collisions''';

  @override
  int getMessageSize() {
    return (path.getMessageSize()) +
        4 +
        smoother_id.length +
        (max_smoothing_duration.getMessageSize()) +
        1;
  }

  @override
  Map<String, dynamic> toJson() => {
        'path': path.toJson(),
        'smoother_id': smoother_id,
        'max_smoothing_duration': max_smoothing_duration.toJson(),
        'check_for_collisions': check_for_collisions
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  SmoothPathGoal fromJson(Map<String, dynamic> jsonMap) {
    return SmoothPathGoal(
        path: nav_msgs.Path().fromJson(jsonMap['path'] as Map<String, dynamic>),
        smoother_id: jsonMap['smoother_id'] as String,
        max_smoothing_duration: builtin_interfaces.Duration().fromJson(
            jsonMap['max_smoothing_duration'] as Map<String, dynamic>),
        check_for_collisions: jsonMap['check_for_collisions'] as bool);
  }
}

class SmoothPathResult extends RosMessage<SmoothPathResult> {
  late nav_msgs.Path path;
  late builtin_interfaces.Duration smoothing_duration;
  late bool was_completed;
  late int error_code;

  static SmoothPathResult $prototype = SmoothPathResult();

  SmoothPathResult(
      {nav_msgs.Path? path,
      builtin_interfaces.Duration? smoothing_duration,
      bool? was_completed,
      int? error_code})
      : path = path ?? nav_msgs.Path(),
        smoothing_duration =
            smoothing_duration ?? builtin_interfaces.Duration(),
        was_completed = was_completed ?? false,
        error_code = error_code ?? 0;

  @override
  String get fullType => 'nav2_msgs/action/SmoothPath_Result';

  @override
  String get messageDefinition => '''nav_msgs/Path path
builtin_interfaces/Duration smoothing_duration
bool was_completed
uint16 error_code''';

  @override
  int getMessageSize() {
    return (path.getMessageSize()) +
        (smoothing_duration.getMessageSize()) +
        1 +
        2;
  }

  @override
  Map<String, dynamic> toJson() => {
        'path': path.toJson(),
        'smoothing_duration': smoothing_duration.toJson(),
        'was_completed': was_completed,
        'error_code': error_code
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  SmoothPathResult fromJson(Map<String, dynamic> jsonMap) {
    return SmoothPathResult(
        path: nav_msgs.Path().fromJson(jsonMap['path'] as Map<String, dynamic>),
        smoothing_duration: builtin_interfaces.Duration()
            .fromJson(jsonMap['smoothing_duration'] as Map<String, dynamic>),
        was_completed: jsonMap['was_completed'] as bool,
        error_code: jsonMap['error_code'] as int);
  }
}

class SmoothPathFeedback extends RosMessage<SmoothPathFeedback> {
  static SmoothPathFeedback $prototype = SmoothPathFeedback();

  SmoothPathFeedback();

  @override
  String get fullType => 'nav2_msgs/action/SmoothPath_Feedback';

  @override
  String get messageDefinition => '''''';

  @override
  int getMessageSize() {
    return 0;
  }

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  SmoothPathFeedback fromJson(Map<String, dynamic> jsonMap) {
    return SmoothPathFeedback();
  }
}

class SmoothPathActionGoal
    extends RosActionGoal<SmoothPathGoal, SmoothPathActionGoal> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalID goal_id;
  @override
  late SmoothPathGoal goal;

  static SmoothPathActionGoal $prototype = SmoothPathActionGoal();

  SmoothPathActionGoal({
    std_msgs.Header? header,
    actionlib_msgs.GoalID? goal_id,
    SmoothPathGoal? goal,
  })  : header = header ?? std_msgs.Header(),
        goal_id = goal_id ?? actionlib_msgs.GoalID(),
        goal = goal ?? SmoothPathGoal();

  @override
  String get fullType => 'nav2_msgs/action/SmoothPath_ActionGoal';

  @override
  String get messageDefinition => '''nav_msgs/Path path
string smoother_id
builtin_interfaces/Duration max_smoothing_duration
bool check_for_collisions''';

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
  SmoothPathActionGoal fromJson(Map<String, dynamic> jsonMap) {
    return SmoothPathActionGoal(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      goal_id: actionlib_msgs.GoalID()
          .fromJson(jsonMap['goal_id'] as Map<String, dynamic>),
      goal: SmoothPathGoal().fromJson(jsonMap['goal'] as Map<String, dynamic>),
    );
  }
}

class SmoothPathActionResult
    extends RosActionResult<SmoothPathResult, SmoothPathActionResult> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalStatus status;
  @override
  late SmoothPathResult result;

  static SmoothPathActionResult $prototype = SmoothPathActionResult();

  SmoothPathActionResult({
    std_msgs.Header? header,
    actionlib_msgs.GoalStatus? status,
    SmoothPathResult? result,
  })  : header = header ?? std_msgs.Header(),
        status = status ?? actionlib_msgs.GoalStatus(),
        result = result ?? SmoothPathResult();

  @override
  String get fullType => 'nav2_msgs/action/SmoothPath_ActionResult';

  @override
  String get messageDefinition => '''nav_msgs/Path path
builtin_interfaces/Duration smoothing_duration
bool was_completed
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
  SmoothPathActionResult fromJson(Map<String, dynamic> jsonMap) {
    return SmoothPathActionResult(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      status: actionlib_msgs.GoalStatus()
          .fromJson(jsonMap['status'] as Map<String, dynamic>),
      result: SmoothPathResult()
          .fromJson(jsonMap['result'] as Map<String, dynamic>),
    );
  }
}

class SmoothPathActionFeedback
    extends RosActionFeedback<SmoothPathFeedback, SmoothPathActionFeedback> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalStatus status;
  @override
  late SmoothPathFeedback feedback;

  static SmoothPathActionFeedback $prototype = SmoothPathActionFeedback();

  SmoothPathActionFeedback({
    std_msgs.Header? header,
    actionlib_msgs.GoalStatus? status,
    SmoothPathFeedback? feedback,
  })  : header = header ?? std_msgs.Header(),
        status = status ?? actionlib_msgs.GoalStatus(),
        feedback = feedback ?? SmoothPathFeedback();

  @override
  String get fullType => 'nav2_msgs/action/SmoothPath_ActionFeedback';

  @override
  String get messageDefinition => '''''';

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
  SmoothPathActionFeedback fromJson(Map<String, dynamic> jsonMap) {
    return SmoothPathActionFeedback(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      status: actionlib_msgs.GoalStatus()
          .fromJson(jsonMap['status'] as Map<String, dynamic>),
      feedback: SmoothPathFeedback()
          .fromJson(jsonMap['feedback'] as Map<String, dynamic>),
    );
  }
}

class SmoothPath extends RosActionMessage<
    SmoothPathGoal,
    SmoothPathActionGoal,
    SmoothPathFeedback,
    SmoothPathActionFeedback,
    SmoothPathResult,
    SmoothPathActionResult> {
  @override
  SmoothPathActionGoal get actionGoal => SmoothPathActionGoal();

  @override
  SmoothPathActionFeedback get actionFeedback => SmoothPathActionFeedback();

  @override
  SmoothPathActionResult get actionResult => SmoothPathActionResult();

  @override
  SmoothPathGoal get goal => SmoothPathGoal();

  @override
  SmoothPathFeedback get feedback => SmoothPathFeedback();

  @override
  SmoothPathResult get result => SmoothPathResult();

  @override
  String get fullType => 'nav2_msgs/action/SmoothPath';

  @override
  String get messageDefinition => '''
Goal:
nav_msgs/Path path
string smoother_id
builtin_interfaces/Duration max_smoothing_duration
bool check_for_collisions
---
Result:
nav_msgs/Path path
builtin_interfaces/Duration smoothing_duration
bool was_completed
uint16 error_code
---
Feedback:
''';
}
