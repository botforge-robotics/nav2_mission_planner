import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:provider/provider.dart';
import '../providers/settings_provider.dart';

class VisibilityToolbar extends StatelessWidget {
  final Color modeColor;

  const VisibilityToolbar({super.key, required this.modeColor});

  @override
  Widget build(BuildContext context) {
    final settings = Provider.of<SettingsProvider>(context);

    return Container(
      width: 60,
      decoration: BoxDecoration(
        color: Colors.black.withOpacity(0.5),
        borderRadius: const BorderRadius.only(
          topLeft: Radius.circular(12),
          bottomLeft: Radius.circular(12),
          topRight: Radius.zero,
          bottomRight: Radius.zero,
        ),
      ),
      padding: const EdgeInsets.symmetric(vertical: 5),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          _buildVisibilityButton(
            icon: Icons.camera_alt,
            isVisible: settings.cameraVisible,
            onPressed: () => settings.toggleCameraVisibility(),
          ),
          const SizedBox(height: 16),
          _buildVisibilityButton(
            icon: FontAwesomeIcons.gamepad,
            isVisible: settings.joystickVisible,
            onPressed: () => settings.toggleJoystickVisibility(),
          ),
        ],
      ),
    );
  }

  Widget _buildVisibilityButton({
    required IconData icon,
    required bool isVisible,
    required VoidCallback onPressed,
  }) {
    return Tooltip(
      message: isVisible ? 'Hide' : 'Show',
      child: IconButton(
        icon: AnimatedSwitcher(
          duration: const Duration(milliseconds: 200),
          child: Icon(
            icon,
            key: ValueKey<bool>(isVisible),
            color: isVisible ? modeColor.withOpacity(0.75) : Colors.white30,
            size: 28,
          ),
        ),
        onPressed: onPressed,
      ),
    );
  }
}
