import 'dart:async';

import 'package:flutter_test/flutter_test.dart';
import 'package:ros2_api/ros2_api.dart';
import 'package:action_tutorials_interfaces/action.dart'; // Import the action definitions

final ros2 =
    Ros2(url: 'ws://localhost:9090'); // Update with your ROS bridge URL

void main() {
  group('Action Client Tests', () {
    setUp(() async {
      ros2.connect();
      await Future.delayed(const Duration(seconds: 2)); // Wait for connection
    });

    tearDown(() async {
      await ros2.close();
      await Future.delayed(const Duration(seconds: 1)); // Cleanup delay
    });

    test('Sends goal to /fibonacci action and preempts after 2 feedbacks',
        () async {
      // Arrange
      final actionClient = ActionClient<
          Fibonacci,
          FibonacciGoal,
          FibonacciActionGoal,
          FibonacciFeedback,
          FibonacciActionFeedback,
          FibonacciResult,
          FibonacciActionResult>(
        ros2: ros2,
        actionName: '/fibonacci',
        actionType: Fibonacci().fullType,
        actionMessage: Fibonacci(),
      );

      final goal = FibonacciGoal(order: 5); // Example goal
      int feedbackCount = 0; // Counter for feedback messages

      // Act
      try {
        final result = await actionClient.sendGoal(goal,
            onFeedback: (FibonacciFeedback feedback) {
          feedbackCount++;
          print('Received feedback: ${feedback.partial_sequence}');

          // Preempt the goal after receiving 2 feedbacks
          if (feedbackCount >= 2) {
            actionClient.cancelGoal();
            print('Goal preempted after receiving 2 feedbacks.');
          }
        });

        // Assert
        expect(result.sequence, isNotEmpty);
        print('Received result: ${result.sequence}');
      } catch (e) {
        fail('Action call failed: $e');
      } finally {
        actionClient.dispose(); // Cleanup
      }
    }, timeout: const Timeout(Duration(seconds: 60)));
  });
}
