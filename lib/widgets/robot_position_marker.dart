import 'package:flutter/material.dart';
import 'dart:math' as math;

class RobotPositionMarker extends StatelessWidget {
  final double x;
  final double y;
  final double theta;
  final Color color;
  final double size;

  const RobotPositionMarker({
    Key? key,
    required this.x,
    required this.y,
    required this.theta,
    required this.color,
    this.size = 24.0,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Transform.rotate(
      angle: theta + math.pi / 2,
      child: Icon(
        Icons.navigation,
        color: color,
        size: size,
      ),
    );
  }
}
