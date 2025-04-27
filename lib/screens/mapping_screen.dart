import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../services/launch_service.dart';
import '../widgets/joystick_thumb_widget.dart';
import '../widgets/occupancy_grid_viewer.dart';

class MappingScreen extends StatefulWidget {
  final Color modeColor;
  const MappingScreen({super.key, required this.modeColor});

  @override
  State<MappingScreen> createState() => _MappingScreenState();
}

class _MappingScreenState extends State<MappingScreen> {
  double _scale = 1.0;
  double _previousScale = 1.0;
  Offset _offset = Offset.zero;
  Offset _previousOffset = Offset.zero;

  @override
  Widget build(BuildContext context) {
    final launchManager = Provider.of<LaunchManager>(context);
    final isMappingActive = launchManager.activeLaunches.isNotEmpty;

    return Consumer<ConnectionProvider>(
      builder: (context, connection, _) {
        return Stack(
          children: [
            // Background
            Positioned.fill(
              child: Container(color: Colors.black),
            ),

            // Occupancy Grid Map Display
            if (isMappingActive)
              Positioned.fill(
                child: GestureDetector(
                  onScaleStart: (details) {
                    _previousScale = _scale;
                    _previousOffset = _offset;
                  },
                  onScaleUpdate: (details) {
                    setState(() {
                      _scale = (_previousScale * details.scale).clamp(0.5, 5.0);

                      // Calculate translation for single finger pan
                      if (details.pointerCount == 1) {
                        final delta =
                            details.focalPoint - details.localFocalPoint;
                        _offset = delta;
                      }
                    });
                  },
                  onScaleEnd: (_) {
                    _previousScale = _scale;
                  },
                  child: Transform.translate(
                    offset: _offset,
                    child: OccupancyGridViewer(
                      topic: '/map',
                      enabled: true,
                      scale: _scale,
                      appModeColor: widget.modeColor,
                      onScaleChanged: (newScale) {
                        setState(() {
                          _scale = newScale;
                        });
                      },
                    ),
                  ),
                ),
              )
            else
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    begin: Alignment.topLeft,
                    end: Alignment.bottomRight,
                    colors: [
                      widget.modeColor.withOpacity(0.1),
                      widget.modeColor.withOpacity(0.05),
                    ],
                  ),
                ),
                child: Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Container(
                        padding: const EdgeInsets.all(20),
                        decoration: BoxDecoration(
                          color: widget.modeColor.withOpacity(0.2),
                          shape: BoxShape.circle,
                        ),
                        child: Icon(
                          Icons.map,
                          size: 80,
                          color: widget.modeColor,
                        ),
                      ),
                      const SizedBox(height: 20),
                      Text(
                        'Mapping Mode',
                        style: TextStyle(
                          fontSize: 32,
                          fontWeight: FontWeight.bold,
                          color: widget.modeColor,
                          letterSpacing: 1.2,
                        ),
                      ),
                      const SizedBox(height: 12),
                      Text(
                        'Start mapping to visualize the environment',
                        style: TextStyle(
                          fontSize: 16,
                          color: widget.modeColor.withOpacity(0.8),
                          fontStyle: FontStyle.italic,
                        ),
                      ),
                    ],
                  ),
                ),
              ),

            // Joystick Control (Same position as TeleopScreen)
            if (isMappingActive)
              Positioned(
                bottom: 60,
                right: 40,
                child: Container(
                  width: 150,
                  height: 150,
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.3),
                    shape: BoxShape.circle,
                  ),
                  child: JoystickThumbWidget(modeColor: widget.modeColor),
                ),
              ),

            // Status overlay for map information (when active)
            if (isMappingActive)
              Positioned(
                top: 16,
                left: 16,
                child: Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.6),
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Map Active',
                        style: TextStyle(
                          color: widget.modeColor,
                          fontWeight: FontWeight.bold,
                          fontSize: 16,
                        ),
                      ),
                      const SizedBox(height: 4),
                      Text(
                        'Zoom: ${_scale.toStringAsFixed(1)}x',
                        style: const TextStyle(
                          color: Colors.white,
                          fontSize: 12,
                        ),
                      ),
                      const Text(
                        'Pinch to zoom • Drag to pan',
                        style: TextStyle(
                          color: Colors.white70,
                          fontSize: 12,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
          ],
        );
      },
    );
  }
}
