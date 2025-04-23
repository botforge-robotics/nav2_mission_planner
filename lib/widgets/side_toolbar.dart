import 'package:flutter/material.dart';
import '../theme/app_theme.dart';
import '../constants/modes.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';

class SideToolbar extends StatelessWidget {
  final AppModes currentMode;
  final Function(AppModes) onModeChanged;

  const SideToolbar({
    super.key,
    required this.currentMode,
    required this.onModeChanged,
  });

  @override
  Widget build(BuildContext context) {
    return Consumer<ConnectionProvider>(
      builder: (context, connection, child) {
        final isConnected = connection.isConnected;

        return Positioned(
          left: 10, // Floating distance from left
          top: AppTheme.statusBarHeight + 10, // Status bar height + 20px gap
          child: Container(
            width: AppTheme.toolbarWidth,
            decoration: BoxDecoration(
              color: AppTheme.toolbarColor,
              borderRadius: BorderRadius.circular(8),
              boxShadow: [
                BoxShadow(
                  color: Colors.black.withOpacity(0.2),
                  blurRadius: 10,
                  offset: const Offset(0, 4),
                ),
              ],
            ),
            child: IntrinsicHeight(
              // This will make height fit content
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  _buildToolbarButton(
                    icon: FontAwesomeIcons.gamepad,
                    tooltip: 'Teleop',
                    mode: AppModes.teleop,
                    isEnabled: isConnected,
                  ),
                  _buildToolbarButton(
                    icon: FontAwesomeIcons.map,
                    tooltip: 'Mapping',
                    mode: AppModes.mapping,
                    isEnabled: isConnected,
                  ),
                  _buildToolbarButton(
                    icon: FontAwesomeIcons.mapLocationDot,
                    tooltip: 'Navigation',
                    mode: AppModes.navigation,
                    isEnabled: isConnected,
                  ),
                  _buildToolbarButton(
                    icon: FontAwesomeIcons.route,
                    tooltip: 'Mission',
                    mode: AppModes.mission,
                    isEnabled: isConnected,
                  ),
                  _buildToolbarButton(
                    icon: FontAwesomeIcons.gear,
                    tooltip: 'Settings',
                    mode: AppModes.settings,
                    isEnabled: true,
                  ),
                ],
              ),
            ),
          ),
        );
      },
    );
  }

  Widget _buildToolbarButton({
    required IconData icon,
    required String tooltip,
    required AppModes mode,
    required bool isEnabled,
  }) {
    final isSelected = currentMode == mode;
    final color = isSelected ? ModeColors.modeColorMap[mode] : Colors.white;

    return Tooltip(
      message: tooltip,
      child: InkWell(
        onTap: isEnabled ? () => onModeChanged(mode) : null,
        child: Container(
          height: 60,
          width: AppTheme.toolbarWidth,
          decoration: BoxDecoration(
            border: isSelected
                ? Border(
                    left: BorderSide(
                      color: ModeColors.modeColorMap[mode]!,
                      width: 3,
                    ),
                  )
                : null,
          ),
          child: Icon(
            icon,
            color: isEnabled ? color : Colors.grey,
            size: 24,
          ),
        ),
      ),
    );
  }
}
