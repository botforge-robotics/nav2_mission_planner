import 'dart:typed_data';
import 'dart:ui' as ui;
import 'dart:async';
import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:nav2_mission_planner/helpers/conversions.dart';
import 'package:nav2_mission_planner/providers/settings_provider.dart';
import 'package:nav2_mission_planner/services/goal_service.dart';
import 'package:provider/provider.dart';
import 'package:ros2_api/ros2_api.dart';
import '../providers/connection_provider.dart';
import 'package:nav_msgs/msg.dart' as nav_msgs;
import 'robot_position_marker.dart';
import 'package:nav2_mission_planner/providers/nav_tool_provider.dart';
import 'package:nav2_mission_planner/services/pose_estimation_service.dart';
import 'dart:ui' as ui show Path;
import 'Arrow_painter.dart';
import 'Simple_rotation_slider.dart';
import 'package:geometry_msgs/msg.dart' as geometry_msgs;
import '../widgets/nav_bottom_bar.dart';

class OccupancyGridViewer extends StatefulWidget {
  final bool enabled;
  final String topic;
  final double scale;
  final Function(double)? onScaleChanged;
  final Color appModeColor;
  final bool showMarkers;

  const OccupancyGridViewer(
      {super.key,
      required this.enabled,
      required this.topic,
      this.scale = 1.0,
      this.onScaleChanged,
      required this.appModeColor,
      this.showMarkers = true});

  @override
  State<OccupancyGridViewer> createState() => _OccupancyGridViewerState();
}

class _OccupancyGridViewerState extends State<OccupancyGridViewer> {
  Subscriber<nav_msgs.OccupancyGrid>? _subscriber;
  ui.Image? _mapImage;
  double _mapResolution = 0.05;
  int _mapWidth = 0;
  int _mapHeight = 0;
  bool _isLoading = false;
  String _statusMessage = 'Waiting for map data...';
  bool _hasError = false;
  double _initialMapFitScale = 1.0;
  bool _hasCalculatedInitialScale = false;
  DateTime _lastUpdateTime = DateTime.now();

  // Controller for the interactive viewer
  final TransformationController _transformationController =
      TransformationController();

  // Add these variables for better zoom tracking
  double _currentScale = 1.0;
  bool _isFirstLoad = true;

  // Add to the class state
  Subscriber<dynamic>? _odomSubscriber;
  double _robotX = 0.0;
  double _robotY = 0.0;
  double _robotTheta = 0.0;

  // Add these variables to store map origin information
  double _mapOriginX = 0.0;
  double _mapOriginY = 0.0;
  double _mapOriginTheta = 0.0;

  // Add to class properties
  bool _poseEstimationMode = false;
  bool _goalMode = false;
  double _markerX = 0.0;
  double _markerY = 0.0;
  double _markerTheta = 0.0;
  bool _isDraggingMarker = false;

  // Add to class state variables
  bool _showRotationSlider = false;
  Offset _rotationSliderCenter = Offset.zero;

  GoalService _goalService = GoalService();

  // Add this variable to track current odometry topic
  String? _currentOdomTopic;
  String? _currentOdomType;

  // Add this reference
  late SettingsProvider _settingsProvider;

  bool _showGoalBar = false;
  bool _showPoseMarker = false;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // Store the provider reference here
    _settingsProvider = Provider.of<SettingsProvider>(context, listen: false);

    final navToolProvider = Provider.of<NavToolProvider>(context);
    if (navToolProvider.poseEstimationEnabled != _poseEstimationMode) {
      setState(() {
        _showRotationSlider = false;
        _showPoseMarker = false;
        _showGoalBar = false;
        _poseEstimationMode = navToolProvider.poseEstimationEnabled;
        _goalMode = false;
        if (!_poseEstimationMode) {
          _markerX = 0.0;
          _markerY = 0.0;
          _markerTheta = 0.0;
        }
      });
    }
    if (navToolProvider.goalMode != _goalMode) {
      setState(() {
        _showRotationSlider = false;
        _showPoseMarker = false;
        _showGoalBar = false;
        _poseEstimationMode = false;
        _goalMode = navToolProvider.goalMode;
        if (!_goalMode) {
          _markerX = 0.0;
          _markerY = 0.0;
          _markerTheta = 0.0;
        }
      });
    }
  }

  @override
  void initState() {
    super.initState();
    // Use stored provider reference
    _settingsProvider = Provider.of<SettingsProvider>(context, listen: false);
    _settingsProvider.addListener(_subscribeToOdometry);
    // Start with identity matrix (no transformations)
    _transformationController.value = Matrix4.identity();
    _subscribeToTopic();
    _subscribeToOdometry();
    _goalService.initialize(context: context);
  }

  @override
  void dispose() {
    // Use stored provider reference instead of Provider.of
    _settingsProvider.removeListener(_subscribeToOdometry);
    _unsubscribe();
    _odomSubscriber?.shutdown();
    _transformationController.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(OccupancyGridViewer oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.topic != widget.topic ||
        oldWidget.enabled != widget.enabled ||
        oldWidget.appModeColor != widget.appModeColor ||
        oldWidget.showMarkers != widget.showMarkers) {
      _unsubscribe();
      _subscribeToTopic();
      _subscribeToOdometry();
    }
  }

  void _subscribeToTopic() {
    if (!widget.enabled) return;

    final connection = Provider.of<ConnectionProvider>(context, listen: false);
    if (connection.ros2Client == null) {
      setState(() {
        _statusMessage = 'No ROS2 connection available';
        _hasError = true;
      });
      return;
    }

    try {
      _subscriber = Subscriber<nav_msgs.OccupancyGrid>(
        name: widget.topic,
        type: nav_msgs.OccupancyGrid().fullType,
        ros2: connection.ros2Client!,
        callback: _processMapMessage,
        prototype: nav_msgs.OccupancyGrid(),
      );
      //print('Subscribed to ${widget.topic}');
      setState(() {
        _statusMessage = 'Subscribed to ${widget.topic}, waiting for data...';
      });
    } catch (e) {
      setState(() {
        _statusMessage = 'Failed to subscribe: $e';
        _hasError = true;
      });
      //print('Error subscribing to ${widget.topic}: $e');
    }
  }

  void _unsubscribe() {
    try {
      _subscriber?.shutdown();
      _subscriber = null;
      //print('Unsubscribed from ${widget.topic}');
    } catch (e) {
      //print('Error unsubscribing: $e');
    }
  }

  void _subscribeToOdometry() {
    final connection = Provider.of<ConnectionProvider>(context, listen: false);
    final settings = Provider.of<SettingsProvider>(context, listen: false);

    // Determine which topic and type to use
    final (String topic, String type) = widget.showMarkers
        ? (settings.navigationOdomTopic, settings.navigationOdomTopicType)
        : (settings.mappingOdomTopic, settings.mappingOdomTopicType);

    // Only resubscribe if topic or type changed
    if (topic == _currentOdomTopic && type == _currentOdomType) return;
    _currentOdomTopic = topic;
    _currentOdomType = type;

    // Unsubscribe from old topic
    if (_odomSubscriber != null) {
      _odomSubscriber?.shutdown();
      _odomSubscriber = null;
    }

    if (connection.ros2Client == null) return;

    try {
      // Create subscriber based on message type
      if (type == 'nav_msgs/msg/Odometry') {
        _odomSubscriber = Subscriber<nav_msgs.Odometry>(
          name: topic,
          type: nav_msgs.Odometry().fullType,
          ros2: connection.ros2Client!,
          callback: _processNavOdomMessage,
          prototype: nav_msgs.Odometry(),
        );
      } else if (type == 'geometry_msgs/msg/PoseWithCovarianceStamped') {
        _odomSubscriber = Subscriber<geometry_msgs.PoseWithCovarianceStamped>(
          name: topic,
          type: geometry_msgs.PoseWithCovarianceStamped().fullType,
          ros2: connection.ros2Client!,
          callback: _processPoseMessage,
          prototype: geometry_msgs.PoseWithCovarianceStamped(),
        );
      }
    } catch (e) {
      print('Error subscribing to odometry: $e');
    }
  }

  void _processNavOdomMessage(nav_msgs.Odometry message) {
    _updateRobotPosition(
      message.pose.pose.position.x,
      message.pose.pose.position.y,
      message.pose.pose.orientation,
    );
  }

  void _processPoseMessage(geometry_msgs.PoseWithCovarianceStamped message) {
    _updateRobotPosition(
      message.pose.pose.position.x,
      message.pose.pose.position.y,
      message.pose.pose.orientation,
    );
  }

  void _updateRobotPosition(double x, double y, geometry_msgs.Quaternion q) {
    if (!mounted) return;

    setState(() {
      final mapPose = transformToMapFrame(
        x,
        y,
        q,
        _mapOriginX,
        _mapOriginY,
        _mapResolution,
        _mapHeight,
        _mapWidth,
        _mapOriginTheta,
      );
      _robotX = mapPose.x;
      _robotY = mapPose.y;
      _robotTheta = mapPose.theta;
    });
  }

  // Create a lighter version of the app mode color
  Color _getLightModeColor() {
    final HSLColor hsl = HSLColor.fromColor(widget.appModeColor);
    return hsl.withLightness((hsl.lightness + 0.4).clamp(0.0, 1.0)).toColor();
  }

  // Create a darker version of the app mode color
  Color _getDarkModeColor() {
    final HSLColor hsl = HSLColor.fromColor(widget.appModeColor);
    return hsl.withLightness((hsl.lightness - 0.2).clamp(0.0, 1.0)).toColor();
  }

  Future<ui.Image> _createMapImage(nav_msgs.OccupancyGrid message) async {
    final int width = message.info.width;
    final int height = message.info.height;
    final Uint8List pixels = Uint8List(width * height * 4);

    // Get light and dark versions of the mode color
    final Color lightColor = _getLightModeColor();
    final Color darkColor = _getDarkModeColor();

    // Extract color components
    final int lightR = lightColor.red;
    final int lightG = lightColor.green;
    final int lightB = lightColor.blue;

    final int darkR = darkColor.red;
    final int darkG = darkColor.green;
    final int darkB = darkColor.blue;

    final int appR = widget.appModeColor.red;
    final int appG = widget.appModeColor.green;
    final int appB = widget.appModeColor.blue;

    // Flip Y-axis by iterating from bottom to top
    for (int y = height - 1; y >= 0; y--) {
      for (int x = 0; x < width; x++) {
        final int index =
            (height - 1 - y) * width + x; // Adjusted index calculation
        final int pixelIndex = (y * width + x) * 4;

        if (index < message.data.length) {
          final int value = message.data[index].toInt();

          if (value == -1) {
            // Unknown space - transparent
            pixels[pixelIndex] = 0;
            pixels[pixelIndex + 1] = 0;
            pixels[pixelIndex + 2] = 0;
            pixels[pixelIndex + 3] = 0;
          } else if (value == 0) {
            // Free space - light version of mode color
            pixels[pixelIndex] = lightR;
            pixels[pixelIndex + 1] = lightG;
            pixels[pixelIndex + 2] = lightB;
            pixels[pixelIndex + 3] = 255; // Fully opaque
          } else if (value == 100) {
            // Fully occupied - dark version of mode color
            pixels[pixelIndex] = darkR;
            pixels[pixelIndex + 1] = darkG;
            pixels[pixelIndex + 2] = darkB;
            pixels[pixelIndex + 3] = 255; // Fully opaque
          } else {
            // Partially occupied - calculate color between mode color and dark
            final double ratio = value / 100.0;
            pixels[pixelIndex] = (appR + (darkR - appR) * ratio).round();
            pixels[pixelIndex + 1] = (appG + (darkG - appG) * ratio).round();
            pixels[pixelIndex + 2] = (appB + (darkB - appB) * ratio).round();
            pixels[pixelIndex + 3] = 255; // Fully opaque
          }
        } else {
          // Out of bounds - transparent
          pixels[pixelIndex] = 0;
          pixels[pixelIndex + 1] = 0;
          pixels[pixelIndex + 2] = 0;
          pixels[pixelIndex + 3] = 0;
        }
      }
    }

    final Completer<ui.Image> completer = Completer();
    ui.decodeImageFromPixels(
      pixels,
      width,
      height,
      ui.PixelFormat.rgba8888,
      (ui.Image image) {
        completer.complete(image);
      },
    );
    return completer.future;
  }

  void _processMapMessage(nav_msgs.OccupancyGrid message) async {
    // Add coordinate system validation
    _validateCoordinateSystem(message.info);

    // Store current transformation before updating the image
    final Matrix4? currentTransform = _mapImage != null
        ? Matrix4.copy(_transformationController.value)
        : null;

    // Remove the update throttling to ensure real-time updates
    final now = DateTime.now();
    _lastUpdateTime = now;

    // print('Processing map update! Width: ${message.info.width}, Height: ${message.info.height}');

    // Only show loading indicator for the first load
    if (_mapImage == null && mounted) {
      setState(() {
        _isLoading = true;
        _statusMessage = 'Processing map data...';
        _hasError = false;
      });
    }

    try {
      // Extract map metadata including origin
      _mapWidth = message.info.width;
      _mapHeight = message.info.height;
      _mapResolution = message.info.resolution;

      // Store origin information
      _mapOriginX = message.info.origin.position.x;
      _mapOriginY = message.info.origin.position.y;

      _mapOriginTheta =
          extractYawFromOriginQuaternion(message.info.origin.orientation);

      //print('Map origin: ($_mapOriginX, $_mapOriginY), theta: $_mapOriginTheta');

      if (_mapWidth <= 0 || _mapHeight <= 0) {
        setState(() {
          _statusMessage = 'Invalid map dimensions: $_mapWidth x $_mapHeight';
          _hasError = true;
          _isLoading = false;
        });
        return;
      }

      // Use the more reliable image creation method
      final ui.Image image = await _createMapImage(message);

      if (mounted) {
        setState(() {
          _mapImage = image;
          _isLoading = false;
          _statusMessage = 'Map rendered successfully';
        });

        // Calculate initial scale to fit screen on first successful load
        if (!_hasCalculatedInitialScale) {
          WidgetsBinding.instance.addPostFrameCallback((_) {
            if (mounted) {
              _calculateInitialScale(context);
            }
          });
        }
      }

      // After updating the image and if not the first load, restore the previous transformation
      if (currentTransform != null && !_isFirstLoad && _mapImage != null) {
        WidgetsBinding.instance.addPostFrameCallback((_) {
          if (mounted) {
            _transformationController.value = currentTransform;
          }
        });
      }
    } catch (e) {
      //print('Error processing map data: $e');
      if (mounted) {
        setState(() {
          _isLoading = false;
          _statusMessage = 'Error: $e';
          _hasError = true;
        });
      }
    }
  }

  void _calculateInitialScale(BuildContext context) {
    if (_mapImage == null || !mounted) return;

    final size = MediaQuery.of(context).size;
    final screenWidth = size.width;
    final screenHeight = size.height;

    final imageWidth = _mapImage!.width.toDouble();
    final imageHeight = _mapImage!.height.toDouble();

    // Calculate scale to fit the image within the screen
    final widthScale = screenWidth / imageWidth;
    final heightScale = screenHeight / imageHeight;

    // Use the smaller scale to ensure the image fits within the screen
    _initialMapFitScale = widthScale < heightScale ? widthScale : heightScale;
    _initialMapFitScale *= 0.9; // 90% of full fit for some padding

    // Set current scale to initial fit scale
    _currentScale = _initialMapFitScale;

    _hasCalculatedInitialScale = true;

    // Only reset view on first load
    if (_isFirstLoad) {
      _resetView();
      _isFirstLoad = false;
    }

    // Notify parent about the initial scale
    if (widget.onScaleChanged != null) {
      widget.onScaleChanged!(_initialMapFitScale);
    }

    //print('Initial map scale calculated: $_initialMapFitScale');
  }

  void _resetView() {
    if (_mapImage == null) return;

    final size = MediaQuery.of(context).size;
    final imageWidth = _mapImage!.width.toDouble();
    final imageHeight = _mapImage!.height.toDouble();

    // Calculate scale to fit the image within the screen
    final widthScale = size.width / imageWidth;
    final heightScale = size.height / imageHeight;
    final scale = (widthScale < heightScale ? widthScale : heightScale) * 0.9;

    // Important: update the _currentScale when resetting
    _currentScale = scale;

    // For proper centering, we need to:
    // 1. Reset to identity
    // 2. Scale appropriately
    // 3. Translate to center
    final matrix = Matrix4.identity();

    // Scale first (this is important for proper calculation)
    matrix.scale(scale, scale);

    // Then translate to center the scaled image
    final scaledWidth = imageWidth * scale;
    final scaledHeight = imageHeight * scale;
    final dx = (size.width - scaledWidth) / 2;
    final dy = (size.height - scaledHeight) / 2;
    matrix.translate(dx / scale, dy / scale);

    // Apply the transformation
    _transformationController.value = matrix;

    // Notify parent about scale change
    if (widget.onScaleChanged != null) {
      widget.onScaleChanged!(scale);
    }

    //print('View reset with scale: $scale');
  }

  void _handleGoalSubmission() async {
    final targetPose = transformFromMapFrame(
      _markerX,
      _markerY,
      _markerTheta,
      _mapOriginX,
      _mapOriginY,
      _mapResolution,
      _mapHeight,
      _mapWidth,
      _mapOriginTheta,
    );
    setState(() {
      _showGoalBar = false;
    });
    final result = await _goalService.sendGoal(
      x: targetPose.targetX,
      y: targetPose.targetY,
      orientation: targetPose.orientation,
      frameId: 'map',
      feedbackHandler: (feedback) => print('Feedback: $feedback'),
    );

    if (result != null) {
      setState(() {
        _showPoseMarker = false;
        _markerX = 0.0;
        _markerY = 0.0;
        _markerTheta = 0.0;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        // Background
        Container(
          color: Colors.black26,
          width: double.infinity,
          height: double.infinity,
        ),

        // Map or loading state
        if (_mapImage != null && !_isLoading)
          // Use a Listener to get pointer events *without* consuming them from InteractiveViewer
          Listener(
            onPointerDown: (details) {
              // Check if pose estimation mode is active on pointer down
              if (_poseEstimationMode) {
                // Potential start of a long press for pose estimation
                // We don't handle it here, let GestureDetector do it
              }
            },
            child: InteractiveViewer(
              transformationController: _transformationController,
              constrained: false,
              minScale: 0.1,
              maxScale: 10.0,
              boundaryMargin: const EdgeInsets.all(double.infinity),
              onInteractionStart: (details) {
                setState(() {
                  _isDraggingMarker = false; // Reset flag
                  final scale =
                      _transformationController.value.getMaxScaleOnAxis();
                  _currentScale = scale;
                  if (widget.onScaleChanged != null) {
                    widget.onScaleChanged!(scale);
                  }
                });
              },
              onInteractionUpdate: (details) {
                setState(() {
                  final scale =
                      _transformationController.value.getMaxScaleOnAxis();
                  _currentScale = scale;
                  if (widget.onScaleChanged != null) {
                    widget.onScaleChanged!(scale);
                  }
                });
                // If we were dragging the pose marker, stop when zoom/pan starts
                if (_isDraggingMarker) {
                  setState(() {
                    _isDraggingMarker = false;
                    _markerX = 0.0;
                    _markerY = 0.0;
                    _markerTheta = 0.0;
                  });
                }
              },
              onInteractionEnd: (details) {
                // Optional: Recalculate scale if needed
              },
              // The actual content that can be panned/zoomed
              child: GestureDetector(
                // Only listen for long press gestures
                onLongPressStart: (details) {
                  if (_poseEstimationMode || _goalMode) {
                    final Offset mapPixelPos = _transformationController
                        .toScene(details.globalPosition);

                    setState(() {
                      _markerX = mapPixelPos.dx;
                      _markerY = mapPixelPos.dy;
                      // Set marker to point right (along x-axis)
                      _markerTheta = 0;
                      _showPoseMarker = true;
                    });
                    _isDraggingMarker = true;
                    _showRotationSlider = false;
                  }
                  if (_goalMode) {
                    _showGoalBar = false;
                  }
                },
                onLongPressMoveUpdate: (details) {
                  if (_poseEstimationMode || _goalMode) {
                    final Offset mapPixelPos = _transformationController
                        .toScene(details.globalPosition);

                    final targetPose = transformFromMapFrame(
                        mapPixelPos.dx,
                        mapPixelPos.dy,
                        0,
                        _mapOriginX,
                        _mapOriginY,
                        _mapResolution,
                        _mapHeight,
                        _mapWidth,
                        _mapOriginTheta);

                    setState(() {
                      _markerX = mapPixelPos.dx;
                      _markerY = mapPixelPos.dy;
                      _markerTheta = extractYawFromOriginQuaternion(
                          targetPose.orientation);
                    });
                    _isDraggingMarker = true;
                  }
                },
                onLongPressEnd: (details) {
                  if (_poseEstimationMode || _goalMode && _isDraggingMarker) {
                    _isDraggingMarker = false;

                    // Position slider centered on marker position
                    setState(() {
                      _showRotationSlider = true;
                      _rotationSliderCenter = Offset(_markerX, _markerY);
                    });
                  }
                },
                // *** IMPORTANT: No onPanStart/Update/End here ***
                child: Stack(
                  // Use ClipRect to ensure children (like markers) don't draw outside the map image bounds if needed
                  // clipBehavior: Clip.hardEdge,
                  children: [
                    RawImage(
                      key: ValueKey(_mapImage.hashCode),
                      image: _mapImage,
                      fit: BoxFit.none, // Important for InteractiveViewer
                      filterQuality: FilterQuality.medium,
                    ),

                    // RobotPositionMarker (scale-invariant)
                    if (_mapImage != null && !_isLoading)
                      Positioned(
                        left: _robotX - 15, // 30/2 = 15
                        top: _robotY - 15,
                        child: IgnorePointer(
                          child: Transform.scale(
                            scale: 1 / _currentScale,
                            alignment: Alignment.center,
                            child: RobotPositionMarker(
                              x: 0,
                              y: 0,
                              theta: _robotTheta,
                              color: widget.appModeColor,
                              size: 30.0,
                            ),
                          ),
                        ),
                      ),

                    // Pose estimation marker (drawn during interaction)
                    if (widget.showMarkers) _buildTargetMarker(),

                    // Rotation slider
                    if (_showRotationSlider)
                      Positioned(
                        left: _rotationSliderCenter.dx - 50,
                        top: _rotationSliderCenter.dy - 50,
                        child: Transform.scale(
                          scale: 1 / _currentScale,
                          child: GestureDetector(
                            onPanStart: (details) {
                              _updateRotation(details.localPosition);
                            },
                            onPanUpdate: (details) {
                              _updateRotation(details.localPosition);
                            },
                            onPanEnd: (_) {
                              setState(() {
                                _showRotationSlider = false;
                                if (_goalMode) {
                                  _showGoalBar = true;
                                } else {
                                  _showGoalBar = false;
                                }
                              });

                              final targetPose = transformFromMapFrame(
                                  _markerX,
                                  _markerY,
                                  _markerTheta,
                                  _mapOriginX,
                                  _mapOriginY,
                                  _mapResolution,
                                  _mapHeight,
                                  _mapWidth,
                                  _mapOriginTheta);

                              if (_poseEstimationMode) {
                                setState(() {
                                  _showPoseMarker = false;
                                });
                                PoseEstimationService.publishPoseEstimate(
                                  context: context,
                                  x: targetPose.targetX,
                                  y: targetPose.targetY,
                                  theta: targetPose.orientation,
                                );
                                Provider.of<NavToolProvider>(context,
                                        listen: false)
                                    .setPoseEstimationEnabled(false);
                              }
                              // Don't send goal immediately for goal mode - wait for slide action
                            },
                            child: Container(
                              width: 100,
                              height: 100,
                              decoration: BoxDecoration(
                                shape: BoxShape.circle,
                                color: Colors.black38,
                              ),
                              child: CustomPaint(
                                painter: SimpleRotationSliderPainter(
                                  angle: _markerTheta,
                                  color: widget.appModeColor,
                                ),
                              ),
                            ),
                          ),
                        ),
                      ),
                  ],
                ),
              ),
            ),
          )
        else
          Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                if (!_hasError)
                  CircularProgressIndicator(color: widget.appModeColor)
                else
                  const Icon(Icons.error_outline, color: Colors.red, size: 40),
                const SizedBox(height: 20),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 24),
                  child: Text(
                    _statusMessage,
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      color: _hasError ? Colors.red : Colors.white70,
                      fontSize: 14,
                    ),
                  ),
                ),
                if (_hasError) ...[
                  const SizedBox(height: 20),
                  ElevatedButton.icon(
                    icon: const Icon(Icons.refresh, size: 18),
                    label: const Text('Retry Connection'),
                    onPressed: () {
                      _unsubscribe();
                      _subscribeToTopic();
                    },
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.red.shade700,
                      foregroundColor: Colors.white,
                      padding: const EdgeInsets.symmetric(
                          horizontal: 20, vertical: 12),
                    ),
                  ),
                ],
              ],
            ),
          ),

        // Reset view button (always on top)
        if (_mapImage != null && !_isLoading)
          Positioned(
            top: 16,
            right: 16,
            child: FloatingActionButton(
              heroTag: 'resetMapView',
              mini: true,
              backgroundColor: widget.appModeColor.withOpacity(0.8),
              onPressed: _resetView,
              tooltip: 'Fit map to screen',
              child: const Icon(
                Icons.fit_screen,
                color: Colors.white,
              ),
            ),
          ),

        // Add NavBottomBar to the stack
        NavBottomBar(
          visible: _showGoalBar,
          onSlideRight: _handleGoalSubmission,
          promptText: 'Slide to send goal',
          color: widget.appModeColor,
        ),
      ],
    );
  }

  // Add new validation method
  void _validateCoordinateSystem(nav_msgs.MapMetaData info) {
    assert(info.resolution > 0, 'Invalid map resolution (must be > 0)');
    assert(info.origin.position.z == 0, 'Z position must be zero for 2D maps');

    final q = info.origin.orientation;
    final rollPitchYaw = quaternionToEuler(q);
    assert(rollPitchYaw[0].abs() < 1e-4 && rollPitchYaw[1].abs() < 1e-4,
        'Map orientation must be 2D (only yaw rotation supported)');
  }

  Widget _buildTargetMarker() {
    // Calculate size based on current scale - larger when zoomed out
    final markerSize = 60.0 * (1 / _currentScale);
    // Adjust positioning based on dynamic size
    final halfWidth = markerSize / 2;
    final height = markerSize;

    return Positioned(
      // Position custom painter exactly at the marker position
      left: _markerX - halfWidth, // Center horizontally based on actual size
      top: _markerY - height, // Bottom of arrow at marker point
      child: IgnorePointer(
        child: Transform.rotate(
          angle: _markerTheta + math.pi / 2,
          alignment: Alignment.bottomCenter, // Pivot at bottom center
          child: CustomPaint(
            size: Size(markerSize, markerSize),
            painter: ArrowPainter(
              color: Colors.greenAccent,
            ),
          ),
        ),
      ),
    );
  }

  // Update the rotation slider positioning and painting logic
  void _updateRotation(Offset localPosition) {
    final center = Offset(50, 50);
    final touchOffset = localPosition - center;

    // Calculate angle where 0 is along x-axis, counterclockwise increases angle
    final mapAngle = math.atan2(touchOffset.dy, touchOffset.dx);

    setState(() {
      _markerTheta = mapAngle;
    });
  }
}

class MapPainter extends CustomPainter {
  final ui.Image mapImage;
  final double resolution;

  MapPainter(this.mapImage, this.resolution);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint();

    // Calculate scaling to fit the map to the screen
    final double scale = size.width / mapImage.width;

    // Draw the map image
    canvas.drawImageRect(
      mapImage,
      Rect.fromLTWH(
          0, 0, mapImage.width.toDouble(), mapImage.height.toDouble()),
      Rect.fromLTWH(0, 0, size.width, size.height),
      paint,
    );

    // Draw grid lines (optional)
    paint.color = Colors.blue.withOpacity(0.3);
    paint.strokeWidth = 1.0;

    // Draw a grid based on map resolution
    final double gridSize = (1.0 / resolution) * scale;
    if (gridSize > 20) {
      // Only draw grid if cells are big enough
      for (double x = 0; x < size.width; x += gridSize) {
        canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
      }
      for (double y = 0; y < size.height; y += gridSize) {
        canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
      }
    }
  }

  @override
  bool shouldRepaint(covariant MapPainter oldDelegate) {
    return mapImage != oldDelegate.mapImage ||
        resolution != oldDelegate.resolution;
  }
}
