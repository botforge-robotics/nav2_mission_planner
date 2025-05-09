// Generated by ros_to_dart_converter
// Do not edit manually
// ignore_for_file: non_constant_identifier_names
import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';
import 'package:std_msgs/msg.dart' as std_msgs;
import 'package:actionlib_msgs/msg.dart' as actionlib_msgs;
import 'package:builtin_interfaces/msg.dart' as builtin_interfaces;
import 'package:geometry_msgs/msg.dart' as geometry_msgs;
import 'package:nav_msgs/msg.dart' as nav_msgs;

class ComputePathThroughPosesGoal
    extends RosMessage<ComputePathThroughPosesGoal> {
  late List<geometry_msgs.PoseStamped> goals;
  late geometry_msgs.PoseStamped start;
  late String planner_id;
  late bool use_start;

  static ComputePathThroughPosesGoal $prototype = ComputePathThroughPosesGoal();

  ComputePathThroughPosesGoal(
      {List<geometry_msgs.PoseStamped>? goals,
      geometry_msgs.PoseStamped? start,
      String? planner_id,
      bool? use_start})
      : goals = goals ?? const [],
        start = start ?? geometry_msgs.PoseStamped(),
        planner_id = planner_id ?? '',
        use_start = use_start ?? false;

  @override
  String get fullType => 'nav2_msgs/action/ComputePathThroughPoses_Goal';

  @override
  String get messageDefinition => '''geometry_msgs/PoseStamped[] goals
geometry_msgs/PoseStamped start
string planner_id
bool use_start''';

  @override
  int getMessageSize() {
    return 4 +
        goals.fold<int>(0, (sum, item) => sum + (item.getMessageSize())) +
        (start.getMessageSize()) +
        4 +
        planner_id.length +
        1;
  }

  @override
  Map<String, dynamic> toJson() => {
        'goals': goals.map((item) => item.toJson()).toList(),
        'start': start.toJson(),
        'planner_id': planner_id,
        'use_start': use_start
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ComputePathThroughPosesGoal fromJson(Map<String, dynamic> jsonMap) {
    return ComputePathThroughPosesGoal(
        goals: (jsonMap['goals'] as List<dynamic>)
            .map((item) => geometry_msgs.PoseStamped()
                .fromJson(item as Map<String, dynamic>))
            .toList(),
        start: geometry_msgs.PoseStamped()
            .fromJson(jsonMap['start'] as Map<String, dynamic>),
        planner_id: jsonMap['planner_id'] as String,
        use_start: jsonMap['use_start'] as bool);
  }
}

class ComputePathThroughPosesResult
    extends RosMessage<ComputePathThroughPosesResult> {
  late nav_msgs.Path path;
  late builtin_interfaces.Duration planning_time;
  late int error_code;

  static ComputePathThroughPosesResult $prototype =
      ComputePathThroughPosesResult();

  ComputePathThroughPosesResult(
      {nav_msgs.Path? path,
      builtin_interfaces.Duration? planning_time,
      int? error_code})
      : path = path ?? nav_msgs.Path(),
        planning_time = planning_time ?? builtin_interfaces.Duration(),
        error_code = error_code ?? 0;

  @override
  String get fullType => 'nav2_msgs/action/ComputePathThroughPoses_Result';

  @override
  String get messageDefinition => '''nav_msgs/Path path
builtin_interfaces/Duration planning_time
uint16 error_code''';

  @override
  int getMessageSize() {
    return (path.getMessageSize()) + (planning_time.getMessageSize()) + 2;
  }

  @override
  Map<String, dynamic> toJson() => {
        'path': path.toJson(),
        'planning_time': planning_time.toJson(),
        'error_code': error_code
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  ComputePathThroughPosesResult fromJson(Map<String, dynamic> jsonMap) {
    return ComputePathThroughPosesResult(
        path: nav_msgs.Path().fromJson(jsonMap['path'] as Map<String, dynamic>),
        planning_time: builtin_interfaces.Duration()
            .fromJson(jsonMap['planning_time'] as Map<String, dynamic>),
        error_code: jsonMap['error_code'] as int);
  }
}

class ComputePathThroughPosesFeedback
    extends RosMessage<ComputePathThroughPosesFeedback> {
  static ComputePathThroughPosesFeedback $prototype =
      ComputePathThroughPosesFeedback();

  ComputePathThroughPosesFeedback();

  @override
  String get fullType => 'nav2_msgs/action/ComputePathThroughPoses_Feedback';

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
  ComputePathThroughPosesFeedback fromJson(Map<String, dynamic> jsonMap) {
    return ComputePathThroughPosesFeedback();
  }
}

class ComputePathThroughPosesActionGoal extends RosActionGoal<
    ComputePathThroughPosesGoal, ComputePathThroughPosesActionGoal> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalID goal_id;
  @override
  late ComputePathThroughPosesGoal goal;

  static ComputePathThroughPosesActionGoal $prototype =
      ComputePathThroughPosesActionGoal();

  ComputePathThroughPosesActionGoal({
    std_msgs.Header? header,
    actionlib_msgs.GoalID? goal_id,
    ComputePathThroughPosesGoal? goal,
  })  : header = header ?? std_msgs.Header(),
        goal_id = goal_id ?? actionlib_msgs.GoalID(),
        goal = goal ?? ComputePathThroughPosesGoal();

  @override
  String get fullType => 'nav2_msgs/action/ComputePathThroughPoses_ActionGoal';

  @override
  String get messageDefinition => '''geometry_msgs/PoseStamped[] goals
geometry_msgs/PoseStamped start
string planner_id
bool use_start''';

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
  ComputePathThroughPosesActionGoal fromJson(Map<String, dynamic> jsonMap) {
    return ComputePathThroughPosesActionGoal(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      goal_id: actionlib_msgs.GoalID()
          .fromJson(jsonMap['goal_id'] as Map<String, dynamic>),
      goal: ComputePathThroughPosesGoal()
          .fromJson(jsonMap['goal'] as Map<String, dynamic>),
    );
  }
}

class ComputePathThroughPosesActionResult extends RosActionResult<
    ComputePathThroughPosesResult, ComputePathThroughPosesActionResult> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalStatus status;
  @override
  late ComputePathThroughPosesResult result;

  static ComputePathThroughPosesActionResult $prototype =
      ComputePathThroughPosesActionResult();

  ComputePathThroughPosesActionResult({
    std_msgs.Header? header,
    actionlib_msgs.GoalStatus? status,
    ComputePathThroughPosesResult? result,
  })  : header = header ?? std_msgs.Header(),
        status = status ?? actionlib_msgs.GoalStatus(),
        result = result ?? ComputePathThroughPosesResult();

  @override
  String get fullType =>
      'nav2_msgs/action/ComputePathThroughPoses_ActionResult';

  @override
  String get messageDefinition => '''nav_msgs/Path path
builtin_interfaces/Duration planning_time
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
  ComputePathThroughPosesActionResult fromJson(Map<String, dynamic> jsonMap) {
    return ComputePathThroughPosesActionResult(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      status: actionlib_msgs.GoalStatus()
          .fromJson(jsonMap['status'] as Map<String, dynamic>),
      result: ComputePathThroughPosesResult()
          .fromJson(jsonMap['result'] as Map<String, dynamic>),
    );
  }
}

class ComputePathThroughPosesActionFeedback extends RosActionFeedback<
    ComputePathThroughPosesFeedback, ComputePathThroughPosesActionFeedback> {
  @override
  late std_msgs.Header header;
  @override
  late actionlib_msgs.GoalStatus status;
  @override
  late ComputePathThroughPosesFeedback feedback;

  static ComputePathThroughPosesActionFeedback $prototype =
      ComputePathThroughPosesActionFeedback();

  ComputePathThroughPosesActionFeedback({
    std_msgs.Header? header,
    actionlib_msgs.GoalStatus? status,
    ComputePathThroughPosesFeedback? feedback,
  })  : header = header ?? std_msgs.Header(),
        status = status ?? actionlib_msgs.GoalStatus(),
        feedback = feedback ?? ComputePathThroughPosesFeedback();

  @override
  String get fullType =>
      'nav2_msgs/action/ComputePathThroughPoses_ActionFeedback';

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
  ComputePathThroughPosesActionFeedback fromJson(Map<String, dynamic> jsonMap) {
    return ComputePathThroughPosesActionFeedback(
      header:
          std_msgs.Header().fromJson(jsonMap['header'] as Map<String, dynamic>),
      status: actionlib_msgs.GoalStatus()
          .fromJson(jsonMap['status'] as Map<String, dynamic>),
      feedback: ComputePathThroughPosesFeedback()
          .fromJson(jsonMap['feedback'] as Map<String, dynamic>),
    );
  }
}

class ComputePathThroughPoses extends RosActionMessage<
    ComputePathThroughPosesGoal,
    ComputePathThroughPosesActionGoal,
    ComputePathThroughPosesFeedback,
    ComputePathThroughPosesActionFeedback,
    ComputePathThroughPosesResult,
    ComputePathThroughPosesActionResult> {
  @override
  ComputePathThroughPosesActionGoal get actionGoal =>
      ComputePathThroughPosesActionGoal();

  @override
  ComputePathThroughPosesActionFeedback get actionFeedback =>
      ComputePathThroughPosesActionFeedback();

  @override
  ComputePathThroughPosesActionResult get actionResult =>
      ComputePathThroughPosesActionResult();

  @override
  ComputePathThroughPosesGoal get goal => ComputePathThroughPosesGoal();

  @override
  ComputePathThroughPosesFeedback get feedback =>
      ComputePathThroughPosesFeedback();

  @override
  ComputePathThroughPosesResult get result => ComputePathThroughPosesResult();

  @override
  String get fullType => 'nav2_msgs/action/ComputePathThroughPoses';

  @override
  String get messageDefinition => '''
Goal:
geometry_msgs/PoseStamped[] goals
geometry_msgs/PoseStamped start
string planner_id
bool use_start
---
Result:
nav_msgs/Path path
builtin_interfaces/Duration planning_time
uint16 error_code
---
Feedback:
''';
}
