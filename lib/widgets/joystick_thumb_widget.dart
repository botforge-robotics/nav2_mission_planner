import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import 'package:geometry_msgs/msg.dart';
import 'package:ros2_api/ros2_api.dart';
import '../providers/settings_provider.dart';

class JoystickThumbWidget extends StatefulWidget {
  final Color modeColor;
  const JoystickThumbWidget({super.key, required this.modeColor});

  @override
  State<JoystickThumbWidget> createState() => _JoystickThumbWidgetState();
}

class _JoystickThumbWidgetState extends State<JoystickThumbWidget> {
  Offset _position = Offset.zero;
  bool _isActive = false;
  Publisher<Twist>? _publisher;
  SettingsProvider? _settingsProvider;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _settingsProvider = Provider.of<SettingsProvider>(context, listen: false);
  }

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _settingsProvider?.addListener(_setupPublisher);
    });
    _setupPublisher();
  }

  void _setupPublisher() {
    final connection = context.read<ConnectionProvider>();
    final settings = context.read<SettingsProvider>();
    if (connection.ros2Client != null && connection.isConnected) {
      _publisher = Publisher<Twist>(
        name: settings.cmdVelTopic,
        type: Twist().fullType,
        ros2: connection.ros2Client!,
      );
    }
  }

  void _updatePosition(Offset localPosition, Size size) {
    final center = size.center(Offset.zero);
    var newPosition = localPosition - center;
    final maxExtent = size.width / 2.5;

    // Limit to circular bounds
    if (newPosition.distance > maxExtent) {
      newPosition = newPosition * (maxExtent / newPosition.distance);
    }

    setState(() {
      _position = newPosition;
      _isActive = true;
    });

    // Normalize values between -1 and 1
    final normalizedX = -(newPosition.dx / maxExtent).clamp(-1.0, 1.0);
    final normalizedY = -(newPosition.dy / maxExtent).clamp(-1.0, 1.0);

    final settings = context.read<SettingsProvider>();
    final linear = normalizedY * settings.linearVelocity;
    final angular = normalizedX * settings.angularVelocity;

    // Publish velocity
    if (_publisher != null) {
      final twist = Twist(
        linear: Vector3(x: linear, y: 0.0, z: 0.0),
        angular: Vector3(x: 0.0, y: 0.0, z: angular),
      );
      _publisher!.publish(twist);
    }
  }

  void _stopMovement() {
    setState(() {
      _position = Offset.zero;
      _isActive = false;
    });

    // Send zero velocity
    if (_publisher != null) {
      final twist = Twist(
        linear: Vector3(x: 0.0, y: 0.0, z: 0.0),
        angular: Vector3(x: 0.0, y: 0.0, z: 0.0),
      );
      _publisher!.publish(twist);
    }
  }

  @override
  void dispose() {
    _settingsProvider?.removeListener(_setupPublisher);
    _publisher?.shutdown();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, constraints) {
        final baseSize = constraints.maxWidth;
        return GestureDetector(
          onPanStart: (details) =>
              _updatePosition(details.localPosition, constraints.biggest),
          onPanUpdate: (details) =>
              _updatePosition(details.localPosition, constraints.biggest),
          onPanEnd: (_) => _stopMovement(),
          child: Container(
            decoration: BoxDecoration(
              color: Colors.black54,
              shape: BoxShape.circle,
              border: Border.all(
                color: widget.modeColor,
                width: 2,
              ),
              boxShadow: [
                BoxShadow(
                  color: widget.modeColor.withOpacity(0.3),
                  blurRadius: 8,
                  spreadRadius: 1,
                )
              ],
            ),
            child: Stack(
              children: [
                // Crosshair guides
                Center(
                  child: Container(
                    width: baseSize * 0.8,
                    height: 1,
                    color: widget.modeColor.withOpacity(0.3),
                  ),
                ),
                Center(
                  child: Container(
                    width: 1,
                    height: baseSize * 0.8,
                    color: widget.modeColor.withOpacity(0.3),
                  ),
                ),

                // Direction markers
                Positioned(
                  top: 10,
                  left: 0,
                  right: 0,
                  child: Center(
                    child: Icon(
                      Icons.keyboard_arrow_up,
                      color: widget.modeColor.withOpacity(0.5),
                      size: 24,
                    ),
                  ),
                ),
                Positioned(
                  bottom: 10,
                  left: 0,
                  right: 0,
                  child: Center(
                    child: Icon(
                      Icons.keyboard_arrow_down,
                      color: widget.modeColor.withOpacity(0.5),
                      size: 24,
                    ),
                  ),
                ),
                Positioned(
                  left: 10,
                  top: 0,
                  bottom: 0,
                  child: Center(
                    child: Icon(
                      Icons.keyboard_arrow_left,
                      color: widget.modeColor.withOpacity(0.5),
                      size: 24,
                    ),
                  ),
                ),
                Positioned(
                  right: 10,
                  top: 0,
                  bottom: 0,
                  child: Center(
                    child: Icon(
                      Icons.keyboard_arrow_right,
                      color: widget.modeColor.withOpacity(0.5),
                      size: 24,
                    ),
                  ),
                ),

                // Thumb control
                Center(
                  child: AnimatedContainer(
                    duration: const Duration(milliseconds: 100),
                    transform: Matrix4.translationValues(
                      _position.dx,
                      _position.dy,
                      0,
                    ),
                    child: Container(
                      width: 28,
                      height: 28,
                      decoration: BoxDecoration(
                        color: _isActive ? widget.modeColor : Colors.white,
                        shape: BoxShape.circle,
                        border: Border.all(
                          color: Colors.white,
                          width: 2,
                        ),
                        boxShadow: _isActive
                            ? [
                                BoxShadow(
                                  color: widget.modeColor.withOpacity(0.6),
                                  blurRadius: 12,
                                  spreadRadius: 2,
                                )
                              ]
                            : null,
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
