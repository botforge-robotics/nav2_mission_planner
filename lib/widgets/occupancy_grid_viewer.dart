import 'dart:typed_data';
import 'dart:ui' as ui;
import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros2_api/ros2_api.dart';
import '../providers/connection_provider.dart';
import 'package:nav_msgs/msg.dart';

class OccupancyGridViewer extends StatefulWidget {
  final bool enabled;
  final String topic;
  final double scale;
  final Function(double)? onScaleChanged;
  final Color appModeColor;

  const OccupancyGridViewer({
    super.key,
    required this.enabled,
    required this.topic,
    this.scale = 1.0,
    this.onScaleChanged,
    required this.appModeColor,
  });

  @override
  State<OccupancyGridViewer> createState() => _OccupancyGridViewerState();
}

class _OccupancyGridViewerState extends State<OccupancyGridViewer> {
  Subscriber<OccupancyGrid>? _subscriber;
  ui.Image? _mapImage;
  double _resolution = 0.05;
  int _width = 0;
  int _height = 0;
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

  @override
  void initState() {
    super.initState();
    // Start with identity matrix (no transformations)
    _transformationController.value = Matrix4.identity();
    _subscribeToTopic();
  }

  @override
  void dispose() {
    _unsubscribe();
    _transformationController.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(OccupancyGridViewer oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.topic != widget.topic ||
        oldWidget.enabled != widget.enabled ||
        oldWidget.appModeColor != widget.appModeColor) {
      _unsubscribe();
      _subscribeToTopic();
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
      _subscriber = Subscriber<OccupancyGrid>(
        name: widget.topic,
        type: OccupancyGrid().fullType,
        ros2: connection.ros2Client!,
        callback: _processMapMessage,
        prototype: OccupancyGrid(),
      );
      print('Subscribed to ${widget.topic}');
      setState(() {
        _statusMessage = 'Subscribed to ${widget.topic}, waiting for data...';
      });
    } catch (e) {
      setState(() {
        _statusMessage = 'Failed to subscribe: $e';
        _hasError = true;
      });
      print('Error subscribing to ${widget.topic}: $e');
    }
  }

  void _unsubscribe() {
    try {
      _subscriber?.shutdown();
      _subscriber = null;
      print('Unsubscribed from ${widget.topic}');
    } catch (e) {
      print('Error unsubscribing: $e');
    }
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

  Future<ui.Image> _createMapImage(OccupancyGrid message) async {
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

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        final int index = y * width + x;
        final int pixelIndex = index * 4;

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

  void _processMapMessage(OccupancyGrid message) async {
    // Store current transformation before updating the image
    final Matrix4? currentTransform = _mapImage != null
        ? Matrix4.copy(_transformationController.value)
        : null;

    // Remove the update throttling to ensure real-time updates
    final now = DateTime.now();
    _lastUpdateTime = now;

    print(
        'Processing map update! Width: ${message.info.width}, Height: ${message.info.height}');

    // Only show loading indicator for the first load
    if (_mapImage == null && mounted) {
      setState(() {
        _isLoading = true;
        _statusMessage = 'Processing map data...';
        _hasError = false;
      });
    }

    try {
      // Extract map metadata
      _width = message.info.width;
      _height = message.info.height;
      _resolution = message.info.resolution;

      if (_width <= 0 || _height <= 0) {
        setState(() {
          _statusMessage = 'Invalid map dimensions: $_width x $_height';
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
      print('Error processing map data: $e');
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

    print('Initial map scale calculated: $_initialMapFitScale');
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

    print('View reset with scale: $scale');
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
          InteractiveViewer(
            transformationController: _transformationController,
            constrained: false,
            minScale: 0.1,
            maxScale: 10.0,
            boundaryMargin: const EdgeInsets.all(double.infinity),
            onInteractionStart: (details) {
              final scale = _transformationController.value.getMaxScaleOnAxis();
              print('Interaction start - scale: $scale');
            },
            onInteractionUpdate: (details) {
              final scale = _transformationController.value.getMaxScaleOnAxis();
              _currentScale = scale;
              if (widget.onScaleChanged != null) {
                widget.onScaleChanged!(scale);
              }
            },
            onInteractionEnd: (details) {
              final scale = _transformationController.value.getMaxScaleOnAxis();
              print('Interaction end - scale: $scale');
            },
            child: RawImage(
              key: ValueKey(_mapImage.hashCode),
              image: _mapImage,
              fit: BoxFit.none,
              filterQuality: FilterQuality.medium,
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
      ],
    );
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
