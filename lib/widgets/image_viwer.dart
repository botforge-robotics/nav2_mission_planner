import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'dart:typed_data';

class ImageViewer extends StatefulWidget {
  final String topic;
  final bool enabled;
  final bool hideTopic;
  const ImageViewer({
    super.key,
    required this.topic,
    required this.enabled,
    this.hideTopic = false,
  });

  @override
  ImageViewerState createState() => ImageViewerState();
}

class ImageViewerState extends State<ImageViewer> {
  String _errorText = '';
  Uint8List? _currentFrame;
  final GlobalKey _frameKey = GlobalKey();

  Uint8List? get currentFrame => _currentFrame;

  @override
  void initState() {
    super.initState();
  }

  @override
  void dispose() {
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (context, constraints) {
        if (!widget.enabled) {
          return Center(child: Text('Camera feed disabled'));
        }

        final connectionProvider = context.read<ConnectionProvider>();
        final robotIp = connectionProvider.ip;

        if (robotIp.isEmpty) {
          return Center(
              child: Text('No robot IP available',
                  style: TextStyle(color: Colors.red)));
        }

        final streamUrl =
            'http://$robotIp:8081/stream?topic=${widget.topic.replaceAll(RegExp(r'/compressed$'), '')}&type=ros_compressed&default_transport=compressed';

        return Container(
          child: _errorText.isNotEmpty
              ? Center(
                  child: Text(_errorText, style: TextStyle(color: Colors.red)))
              : Stack(
                  children: [
                    RepaintBoundary(
                      key: _frameKey,
                      child: Mjpeg(
                        stream: streamUrl,
                        width: constraints.maxWidth,
                        height: constraints.maxHeight,
                        fit: BoxFit.contain,
                        timeout: const Duration(seconds: 5),
                        isLive: true,
                        loading: (context) {
                          return Center(
                            child: Column(
                              mainAxisSize: MainAxisSize.min,
                              children: [
                                CircularProgressIndicator(
                                  color: Colors.white70,
                                ),
                                SizedBox(height: 16),
                                Text(
                                  'Connecting to stream...',
                                  style: TextStyle(color: Colors.white70),
                                ),
                              ],
                            ),
                          );
                        },
                        error: (context, error, stack) {
                          WidgetsBinding.instance.addPostFrameCallback((_) {
                            setState(() {
                              _errorText = 'Stream error: $error';
                            });
                          });
                          return Center(
                            child: Column(
                              mainAxisSize: MainAxisSize.min,
                              children: [
                                Icon(Icons.videocam_off,
                                    color: Colors.red.withOpacity(0.7),
                                    size: 48),
                                SizedBox(height: 16),
                                Text(
                                  'No video streaming\nCheck camera settings',
                                  textAlign: TextAlign.center,
                                  style: TextStyle(
                                      color: Colors.red.withOpacity(0.7)),
                                ),
                              ],
                            ),
                          );
                        },
                      ),
                    ),
                    if (!widget.hideTopic)
                      Positioned(
                        top: 10,
                        right: 8,
                        child: Container(
                          padding:
                              EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                          decoration: BoxDecoration(
                            color: Colors.white.withOpacity(0.2),
                            borderRadius: BorderRadius.circular(8),
                          ),
                          child: Text(
                            widget.topic,
                            style: TextStyle(
                              color: Colors.white,
                              fontSize: 11,
                              fontWeight: FontWeight.w500,
                            ),
                          ),
                        ),
                      ),
                  ],
                ),
        );
      },
    );
  }

  Future<Uint8List?> captureFrame() async {
    try {
      final boundary = _frameKey.currentContext?.findRenderObject();
      if (boundary is RenderRepaintBoundary) {
        final image = await boundary.toImage();
        final byteData = await image.toByteData(format: ImageByteFormat.png);
        return byteData?.buffer.asUint8List();
      }
    } catch (e) {
      print('Frame capture error: $e');
    }
    return null;
  }
}
