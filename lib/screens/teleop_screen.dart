import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/settings_provider.dart';
import '../widgets/image_viwer.dart';
import '../widgets/joystick_thumb_widget.dart';
import '../widgets/camera_snap_button.dart';

class TeleopScreen extends StatelessWidget {
  final Color modeColor;
  const TeleopScreen({super.key, required this.modeColor});

  @override
  Widget build(BuildContext context) {
    final GlobalKey<ImageViewerState> _imageViewerKey = GlobalKey();

    return Consumer<SettingsProvider>(
      builder: (context, settings, child) {
        return Stack(
          children: [
            // Full screen image viewer
            if (settings.cameraEnabled && settings.cameraImageTopic.isNotEmpty)
              Positioned.fill(
                child: Container(
                  color: Colors.black,
                  child: ImageViewer(
                    key: _imageViewerKey,
                    topic: settings.cameraImageTopic,
                    enabled: settings.cameraEnabled,
                  ),
                ),
              ),

            // Show message when camera is disabled or no topic selected
            if (!settings.cameraEnabled || settings.cameraImageTopic.isEmpty)
              Container(
                decoration: BoxDecoration(
                  gradient: LinearGradient(
                    begin: Alignment.topLeft,
                    end: Alignment.bottomRight,
                    colors: [
                      modeColor.withOpacity(0.1),
                      modeColor.withOpacity(0.05),
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
                          color: modeColor.withOpacity(0.2),
                          shape: BoxShape.circle,
                        ),
                        child: Icon(
                          Icons.videocam_off,
                          size: 80,
                          color: modeColor,
                        ),
                      ),
                      const SizedBox(height: 20),
                      Text(
                        'Camera Disabled',
                        style: TextStyle(
                          fontSize: 32,
                          fontWeight: FontWeight.bold,
                          color: modeColor,
                          letterSpacing: 1.2,
                        ),
                      ),
                      const SizedBox(height: 12),
                      Text(
                        'Enable camera in settings',
                        style: TextStyle(
                          fontSize: 16,
                          color: modeColor.withOpacity(0.8),
                          fontStyle: FontStyle.italic,
                        ),
                      ),
                    ],
                  ),
                ),
              ),

            // Joystick
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
                child: JoystickThumbWidget(modeColor: modeColor),
              ),
            ),

            CameraSnapButton(
              modeColor: modeColor,
              isCameraActive: settings.cameraEnabled &&
                  settings.cameraImageTopic.isNotEmpty,
              getCurrentFrame: () async =>
                  _imageViewerKey.currentState?.captureFrame(),
            ),
          ],
        );
      },
    );
  }
}
