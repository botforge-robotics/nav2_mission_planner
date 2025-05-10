import 'package:flutter/material.dart';
import 'dart:ui' as ui;

class ArrowPainter extends CustomPainter {
  final Color color;

  ArrowPainter({required this.color});

  @override
  void paint(Canvas canvas, Size size) {
    final Paint paint = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3.0;

    final double width = size.width;
    final double height = size.height;
    final double centerX = width / 2;
    final double shaftLength = height * 0.5;
    final double arrowHeadSize = width * 0.25;

    // Draw pivot dot at bottom center
    final Paint dotPaint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;

    canvas.drawCircle(
        Offset(centerX, height), // Bottom center position
        3.0, // Dot radius
        dotPaint);

    // Draw shaft
    canvas.drawLine(
        Offset(centerX, height), Offset(centerX, height - shaftLength), paint);

    // Draw arrowhead
    final ui.Path arrowHead = ui.Path();
    final Offset tipPoint =
        Offset(centerX, height - shaftLength - arrowHeadSize);
    final Offset leftCorner =
        Offset(centerX - arrowHeadSize, height - shaftLength);
    final Offset rightCorner =
        Offset(centerX + arrowHeadSize, height - shaftLength);

    arrowHead.moveTo(tipPoint.dx, tipPoint.dy);
    arrowHead.lineTo(leftCorner.dx, leftCorner.dy);
    arrowHead.lineTo(rightCorner.dx, rightCorner.dy);
    arrowHead.lineTo(tipPoint.dx, tipPoint.dy);

    final Paint headPaint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;

    canvas.drawPath(arrowHead, headPaint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return oldDelegate is ArrowPainter && oldDelegate.color != color;
  }
}
