import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros2_api/ros2_api.dart';
import 'package:nav2_msgs/action.dart';
import '../providers/connection_provider.dart';
import 'package:geometry_msgs/msg.dart' as geometry_msgs;
import 'package:std_msgs/msg.dart' as std_msgs;
import 'package:builtin_interfaces/msg.dart' as builtin_interfaces;

class GoalService {
  late ActionClient<
      NavigateToPose,
      NavigateToPoseGoal,
      NavigateToPoseActionGoal,
      NavigateToPoseFeedback,
      NavigateToPoseActionFeedback,
      NavigateToPoseResult,
      NavigateToPoseActionResult> _client;

  void initialize({
    required BuildContext context,
  }) {
    _client = ActionClient(
      ros2: Provider.of<ConnectionProvider>(context, listen: false).ros2Client!,
      actionName: '/navigate_to_pose',
      actionType: NavigateToPose().fullType,
      actionMessage: NavigateToPose(),
    );
  }

  Future<void> sendGoal({
    required double x,
    required double y,
    required geometry_msgs.Quaternion orientation,
    required String frameId,
    Function(NavigateToPoseFeedback)? feedbackHandler,
  }) async {
    final goal = NavigateToPoseGoal()
      ..pose = geometry_msgs.PoseStamped(
        header: std_msgs.Header(
          stamp: builtin_interfaces.Time(
            sec: DateTime.now().millisecondsSinceEpoch ~/ 1000,
          ),
          frame_id: frameId,
        ),
        pose: geometry_msgs.Pose(
          position: geometry_msgs.Point(x: x, y: y, z: 0.0),
          orientation: orientation,
        ),
      );

    try {
      await _client.sendGoal(
        goal,
        onFeedback: (feedback) {
          feedbackHandler?.call(feedback);
        },
      );
    } catch (e) {
      print('Failed to send goal: $e');
    }
  }

  void cancelCurrentGoal(BuildContext context) {
    try {
      _client.cancelGoal();
    } catch (e) {
      print('Error canceling goal: $e');
    }
  }
}
