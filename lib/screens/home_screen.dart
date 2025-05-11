import 'package:flutter/material.dart';
import 'package:nav2_mission_planner/widgets/top_status_bar/top_status_bar.dart';
import '../constants/modes.dart';
import '../theme/app_theme.dart';
import 'teleop_screen.dart';
import 'mapping_screen.dart';
import 'navigation_screen.dart';
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
  AppModes? _previousMode;

  Widget _getScreen(AppModes mode) {
    final modeColor = ModeColors.modeColorMap[mode]!;
    switch (mode) {
      case AppModes.teleop:
        return TeleopScreen(modeColor: modeColor);
      case AppModes.mapping:
        return MappingScreen(modeColor: modeColor);
      case AppModes.navigation:
        return NavigationScreen(modeColor: modeColor);
      case AppModes.settings:
        return const SizedBox.shrink(); // Should never happen with new design
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
        // Auto-switch to teleop when disconnected
        if (!connectionProvider.isConnected &&
            _currentMode != AppModes.teleop) {
          WidgetsBinding.instance.addPostFrameCallback((_) {
            setState(() {
              _currentMode = AppModes.teleop;
              _previousMode = null;
            });
          });
        }

        // Track the active underlying screen (teleop, mapping, navigation)
        final activeScreen = _currentMode == AppModes.settings
            ? _previousMode ?? AppModes.teleop
            : _currentMode;

        return Scaffold(
          body: Stack(
            children: [
              // Main content with status bar and screen stack
              Column(
                children: [
                  TopStatusBar(
                    currentMode: _currentMode,
                    onModeChanged: (mode) {
                      setState(() {
                        if (mode == AppModes.settings) {
                          // Store previous mode before switching to settings
                          _previousMode = _currentMode;
                        } else {
                          _previousMode = null;
                        }
                        _currentMode = mode;
                      });
                    },
                    statusText:
                        _getModeStatusText(connectionProvider.isConnected),
                    statusColor: connectionProvider.isConnected
                        ? ModeColors.modeColorMap[
                            _currentMode == AppModes.settings
                                ? _previousMode ?? AppModes.teleop
                                : _currentMode]!
                        : Colors.red,
                    height: AppTheme.statusBarHeight,
                    icon: connectionProvider.isConnected
                        ? null
                        : FontAwesomeIcons.robot,
                  ),

                  // Keep all screens alive with IndexedStack
                  Expanded(
                    child: connectionProvider.isConnected
                        ? IndexedStack(
                            index: _getScreenIndex(activeScreen),
                            children: [
                              TeleopScreen(
                                  modeColor: ModeColors
                                      .modeColorMap[AppModes.teleop]!),
                              MappingScreen(
                                  modeColor: ModeColors
                                      .modeColorMap[AppModes.mapping]!),
                              NavigationScreen(
                                  modeColor: ModeColors
                                      .modeColorMap[AppModes.navigation]!),
                            ],
                          )
                        : _buildDisconnectedUI(Colors.white60),
                  ),
                ],
              ),

              // Settings overlay when in settings mode
              if (_currentMode == AppModes.settings)
                Positioned.fill(
                  top: AppTheme.statusBarHeight,
                  child: Material(
                    color: Colors.transparent,
                    child: const SettingsScreen(),
                  ),
                ),
            ],
          ),
        );
      },
    );
  }

  // Add helper method to get the correct index for IndexedStack
  int _getScreenIndex(AppModes mode) {
    switch (mode) {
      case AppModes.teleop:
        return 0;
      case AppModes.mapping:
        return 1;
      case AppModes.navigation:
        return 2;
      case AppModes.settings:
        return 0; // Should never happen with new design
    }
  }
}
