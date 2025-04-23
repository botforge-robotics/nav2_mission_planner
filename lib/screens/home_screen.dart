import 'package:flutter/material.dart';
import '../widgets/side_toolbar.dart';
import '../widgets/top_status_bar.dart';
import '../widgets/compass_widget.dart';
import '../constants/modes.dart';
import '../theme/app_theme.dart';
import 'teleop_screen.dart';
import 'mapping_screen.dart';
import 'navigation_screen.dart';
import 'mission_screen.dart';
import 'settings/settings_screen.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  AppModes _currentMode = AppModes.teleop;

  Widget _getCurrentScreen() {
    switch (_currentMode) {
      case AppModes.teleop:
        return const TeleopScreen();
      case AppModes.mapping:
        return const MappingScreen();
      case AppModes.navigation:
        return const NavigationScreen();
      case AppModes.mission:
        return const MissionScreen();
      case AppModes.settings:
        return const SettingsScreen();
    }
  }

  String _getModeStatusText(bool isConnected) {
    if (!isConnected) {
      return 'Connect to Robot';
    }
    switch (_currentMode) {
      case AppModes.teleop:
        return 'Teleoperation Mode';
      case AppModes.mapping:
        return 'Mapping Mode';
      case AppModes.navigation:
        return 'Navigation Mode';
      case AppModes.mission:
        return 'Mission Planning Mode';
      case AppModes.settings:
        return 'Settings';
    }
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<ConnectionProvider>(
      builder: (context, connectionProvider, child) {
        return Scaffold(
          body: Stack(
            children: [
              Column(
                children: [
                  TopStatusBar(
                    statusText:
                        _getModeStatusText(connectionProvider.isConnected),
                    statusColor: connectionProvider.isConnected
                        ? ModeColors.modeColorMap[_currentMode]!
                        : Colors.red,
                    height: AppTheme.statusBarHeight,
                    icon: connectionProvider.isConnected
                        ? null
                        : FontAwesomeIcons.robot,
                  ),
                  Expanded(
                    child: connectionProvider.isConnected
                        ? _getCurrentScreen()
                        : Center(
                            child: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: [
                                Icon(
                                  FontAwesomeIcons.robot,
                                  size: 100,
                                  color: Colors.grey,
                                ),
                                SizedBox(height: 20),
                                Text(
                                  'Connect to Robot',
                                  style: TextStyle(
                                    fontSize: 24,
                                    color: Colors.grey,
                                  ),
                                ),
                              ],
                            ),
                          ),
                  ),
                ],
              ),
              SideToolbar(
                currentMode: _currentMode,
                onModeChanged: (mode) {
                  setState(() {
                    _currentMode = mode;
                  });
                },
              ),
            ],
          ),
        );
      },
    );
  }
}
