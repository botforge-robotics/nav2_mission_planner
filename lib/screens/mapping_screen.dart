import 'package:flutter/material.dart';
import 'package:nav2_mission_planner/providers/settings_provider.dart';
import 'package:nav2_mission_planner/widgets/save_map_dialog.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../services/launch_service.dart';
import '../widgets/joystick_thumb_widget.dart';
import '../widgets/occupancy_grid_viewer.dart';
import '../widgets/image_viwer.dart';

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
  bool _isMappingStarted = false;
  bool _isMappingActive = false;

  // Add this - don't even create the OccupancyGridViewer until we're ready
  Widget? _mapWidget;

  @override
  Widget build(BuildContext context) {
    final settings = Provider.of<SettingsProvider>(context);
    return Consumer<ConnectionProvider>(
      builder: (context, connection, _) {
        return Stack(
          children: [
            // Background
            Positioned.fill(
              child: Container(color: Colors.black),
            ),

            // Occupancy Grid Map Display - only create it when started
            if (_mapWidget != null)
              Positioned.fill(child: _mapWidget!)
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
                          fontSize: 28,
                          fontWeight: FontWeight.bold,
                          color: widget.modeColor,
                          letterSpacing: 1.2,
                        ),
                      ),
                      const SizedBox(height: 12),
                      ElevatedButton(
                        onPressed: () async {
                          final launchManager = Provider.of<LaunchManager>(
                              context,
                              listen: false);
                          final success =
                              await launchManager.startMapping(context);
                          if (success) {
                            // Create map widget only when we're ready to start
                            setState(() {
                              _isMappingStarted = true;
                              _isMappingActive = true;

                              // Create the widget now that mapping is started
                              _mapWidget = GestureDetector(
                                onScaleStart: (details) {
                                  _previousScale = _scale;
                                  _previousOffset = _offset;
                                },
                                onScaleUpdate: (details) {
                                  setState(() {
                                    _scale = (_previousScale * details.scale)
                                        .clamp(0.5, 5.0);
                                    if (details.pointerCount == 1) {
                                      final delta = details.focalPoint -
                                          details.localFocalPoint;
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
                              );
                            });
                          }
                        },
                        style: ElevatedButton.styleFrom(
                          backgroundColor: widget.modeColor.withOpacity(0.2),
                          padding: const EdgeInsets.symmetric(
                              horizontal: 25, vertical: 15),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(20),
                            side: BorderSide(color: widget.modeColor, width: 2),
                          ),
                        ),
                        child: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(Icons.play_arrow,
                                color: widget.modeColor, size: 28),
                            const SizedBox(width: 10),
                            Text(
                              'Start Mapping',
                              style: TextStyle(
                                fontSize: 20,
                                color: widget.modeColor,
                                fontWeight: FontWeight.bold,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              ),

            // Joystick Control
            if (_isMappingStarted && _isMappingActive)
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

            // Status overlay
            if (_isMappingStarted && _isMappingActive)
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
            if (_isMappingStarted && _isMappingActive)
              // mapping_screen.dart
              Positioned(
                top: 65,
                right: 16,
                child: FloatingActionButton(
                  mini: true,
                  backgroundColor: widget.modeColor,
                  onPressed: () async {
                    final result = await showDialog<Map<String, dynamic>>(
                      context: context,
                      builder: (context) => MapSaveDialog(
                        screenSize: MediaQuery.of(context).size,
                        modeColor: widget.modeColor,
                      ),
                    );

                    if (result != null) {
                      final String mapName = result['mapName'];
                      final bool stopMapping = result['stopMapping'];

                      // Show saving overlay
                      showDialog(
                        context: context,
                        barrierDismissible: false,
                        builder: (context) => AlertDialog(
                          backgroundColor: Colors.transparent,
                          elevation: 0,
                          content: Center(
                            child: Column(
                              mainAxisSize: MainAxisSize.min,
                              children: [
                                CircularProgressIndicator(
                                    color: widget.modeColor),
                                const SizedBox(height: 16),
                                Text(
                                  'Saving Map...',
                                  style: TextStyle(
                                    color: Colors.white,
                                    fontSize: 16,
                                  ),
                                ),
                              ],
                            ),
                          ),
                        ),
                      );

                      final launchManager =
                          Provider.of<LaunchManager>(context, listen: false);
                      final success =
                          await launchManager.saveMap(context, mapName);

                      // Dismiss saving overlay
                      if (mounted) Navigator.pop(context);

                      if (success) {
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: Text('Map saved as $mapName'),
                            backgroundColor: Colors.green,
                          ),
                        );

                        // Only stop mapping if requested AND save was successful
                        if (stopMapping) {
                          for (final entry
                              in launchManager.activeLaunches.entries) {
                            try {
                              await launchManager.stopLaunch(
                                  context, entry.key);
                            } catch (e) {
                              ScaffoldMessenger.of(context).showSnackBar(
                                SnackBar(
                                  content: Text('Error stopping mapping: $e'),
                                  backgroundColor: Colors.red,
                                ),
                              );
                            }
                          }

                          setState(() {
                            _isMappingStarted = false;
                            _isMappingActive = false;
                            _mapWidget = null;
                          });
                        }
                      } else {
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: Text('Failed to save map $mapName'),
                            backgroundColor: Colors.red,
                          ),
                        );
                      }
                    }
                  },
                  child: const Icon(Icons.save, color: Colors.white),
                ),
              ),
            if (settings.cameraEnabled &&
                settings.cameraImageTopic.isNotEmpty &&
                _isMappingStarted &&
                _isMappingActive)
              Positioned(
                bottom: 20,
                left: 20,
                child: SizedBox(
                  width: MediaQuery.of(context).size.width * 0.25,
                  height: MediaQuery.of(context).size.width * 0.25 * (9 / 16),
                  child: Container(
                    decoration: BoxDecoration(
                      color: Colors.black,
                      borderRadius: BorderRadius.circular(8),
                      border: Border.all(color: widget.modeColor, width: 2),
                    ),
                    child: ClipRRect(
                      borderRadius: BorderRadius.circular(6),
                      child: ImageViewer(
                        topic: settings.cameraImageTopic,
                        enabled: settings.cameraEnabled,
                        hideTopic: true,
                      ),
                    ),
                  ),
                ),
              ),
          ],
        );
      },
    );
  }

  @override
  void dispose() {
    _mapWidget = null;
    super.dispose();
  }
}
