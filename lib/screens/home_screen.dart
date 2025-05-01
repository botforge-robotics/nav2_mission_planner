import 'package:flutter/material.dart';
import 'package:nav2_mission_planner/widgets/top_status_bar/top_status_bar.dart';
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
    final modeColor = ModeColors.modeColorMap[_currentMode]!;
    switch (_currentMode) {
      case AppModes.teleop:
        return TeleopScreen(modeColor: modeColor);
      case AppModes.mapping:
        return MappingScreen(modeColor: modeColor);
      case AppModes.navigation:
        return NavigationScreen(modeColor: modeColor);
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
      case AppModes.settings:
        return 'Settings';
    }
  }

  Widget _buildDisconnectedUI(Color modeColor) {
    return Container(
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
            SizedBox(
              width: 160,
              height: 160,
              child: Stack(
                children: [
                  // Circle background
                  Container(
                    width: 160,
                    height: 160,
                    decoration: BoxDecoration(
                      color: modeColor.withOpacity(0.2),
                      shape: BoxShape.circle,
                    ),
                  ),
                  // Manually positioned icon
                  Positioned(
                    left: 19, // Adjust these values to fine-tune centering
                    top: 25, // Adjust these values to fine-tune centering
                    child: Icon(
                      FontAwesomeIcons.robot,
                      size: 100,
                      color: modeColor,
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 24),
            Text(
              'Connect to Robot',
              style: TextStyle(
                fontSize: 32,
                fontWeight: FontWeight.bold,
                color: modeColor,
                letterSpacing: 1.2,
              ),
            ),
            const SizedBox(height: 16),
            Text(
              'Use the connection button in the status bar',
              style: TextStyle(
                fontSize: 16,
                color: modeColor.withOpacity(0.8),
                fontStyle: FontStyle.italic,
              ),
            ),
          ],
        ),
      ),
    );
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
                    currentMode: _currentMode,
                    onModeChanged: (mode) {
                      setState(() {
                        _currentMode = mode;
                      });
                    },
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
                        : _buildDisconnectedUI(Colors.white60),
                  ),
                ],
              ),
            ],
          ),
        );
      },
    );
  }
}
