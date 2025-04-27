import 'package:flutter/material.dart';
import 'package:nav2_mission_planner/services/launch_service.dart';
import 'package:provider/provider.dart';
import '../../theme/app_theme.dart';
import '../../providers/connection_provider.dart';
import '../../constants/modes.dart';
import 'top_status_mode_selector.dart';
import 'top_status_center_title.dart';
import 'top_status_network_info.dart';
import 'top_status_connection_button.dart';
import '../../services/launch_service.dart';

class TopStatusBar extends StatelessWidget {
  final String statusText;
  final Color statusColor;
  final double height;
  final IconData? icon;
  final AppModes currentMode;
  final Function(AppModes) onModeChanged;

  const TopStatusBar({
    super.key,
    required this.statusText,
    required this.statusColor,
    this.height = 50.0,
    this.icon,
    required this.currentMode,
    required this.onModeChanged,
  });

  @override
  Widget build(BuildContext context) {
    return Consumer<ConnectionProvider>(
      builder: (context, connection, child) {
        final connectionStatusColor =
            connection.isConnected ? statusColor : Colors.red;
        final displayStatusText =
            connection.isConnected ? statusText : 'Disconnected';

        return Container(
          height: height,
          color: AppTheme.toolbarColor,
          child: Stack(
            children: [
              TopStatusModeSelector(
                height: height,
                connectionStatusColor: connectionStatusColor,
                displayStatusText: displayStatusText,
                currentMode: currentMode,
                onModeChanged: (newMode) async {
                  // ONLY handle stopping mapping, NEVER start mapping
                  if (currentMode == AppModes.mapping &&
                      newMode != AppModes.mapping) {
                    // Confirm stopping active mapping when LEAVING mapping mode
                    final launchManager =
                        Provider.of<LaunchManager>(context, listen: false);
                    if (launchManager.activeLaunches.isNotEmpty) {
                      final shouldStop = await showDialog<bool>(
                        context: context,
                        builder: (context) => AlertDialog(
                          title: const Text('Active Mapping Session'),
                          content: const Text(
                              'You have an active mapping session. Stop it before changing modes?'),
                          actions: [
                            TextButton(
                              onPressed: () => Navigator.pop(context, false),
                              child: const Text('Keep Mapping'),
                            ),
                            TextButton(
                              onPressed: () => Navigator.pop(context),
                              child: const Text('Cancel',
                                  style: TextStyle(color: Colors.white)),
                            ),
                            ElevatedButton(
                              style: ElevatedButton.styleFrom(
                                backgroundColor: Colors.red,
                                foregroundColor: Colors.white,
                              ),
                              onPressed: () => Navigator.pop(context, true),
                              child: const Text('Stop Mapping'),
                            ),
                          ],
                        ),
                      );

                      if (shouldStop ?? false) {
                        for (final entry
                            in launchManager.activeLaunches.entries) {
                          try {
                            await launchManager.stopLaunch(context, entry.key);
                          } catch (e) {
                            ScaffoldMessenger.of(context).showSnackBar(
                              SnackBar(
                                content: Text(
                                    'Failed to stop mapping: ${e.toString()}'),
                                backgroundColor: Colors.red,
                              ),
                            );
                          }
                        }
                      } else {
                        return; // Abort mode change
                      }
                    }
                  }

                  // Simply change the mode, nothing else
                  onModeChanged(newMode);
                },
              ),
              const TopStatusCenterTitle(),
              TopStatusNetworkInfo(
                  height: height, connectionStatusColor: connectionStatusColor),
              TopStatusConnectionButton(
                  height: height, connectionStatusColor: connectionStatusColor),
            ],
          ),
        );
      },
    );
  }
}
