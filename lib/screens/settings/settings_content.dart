import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:nav2_mission_planner/providers/settings_provider.dart';
import 'package:nav2_mission_planner/screens/settings/widgets/setting_card.dart';
import 'package:nav2_mission_planner/screens/settings/widgets/setting_header.dart';
import 'package:provider/provider.dart';
import '../../constants/modes.dart';
import 'teleop_settings.dart';
import 'mapping_settings.dart';
import 'navigation_settings.dart';
import 'widgets/camera_topic_input.dart';

class SettingsContent extends StatelessWidget {
  final String category;
  final Size screenSize;

  const SettingsContent({
    super.key,
    required this.category,
    required this.screenSize,
  });

  // Helper method to get color from mode
  Color _getModeColor(String category) {
    switch (category) {
      case 'Teleop':
        return ModeColors.modeColorMap[AppModes.teleop]!;
      case 'Mapping':
        return ModeColors.modeColorMap[AppModes.mapping]!;
      case 'Navigation':
        return ModeColors.modeColorMap[AppModes.navigation]!;
      case 'Mission':
        return ModeColors.modeColorMap[AppModes.mission]!;
      case 'General':
        return ModeColors.modeColorMap[AppModes.settings]!;
      default:
        return ModeColors.modeColorMap[AppModes.settings]!;
    }
  }

  @override
  Widget build(BuildContext context) {
    if (category == 'Teleop') {
      return TeleopSettings(
        screenSize: screenSize,
        modeColor: _getModeColor('Teleop'),
      );
    } else if (category == 'Mapping') {
      return MappingSettings(
        screenSize: screenSize,
        modeColor: _getModeColor('Mapping'),
      );
    } else if (category == 'Navigation') {
      return NavigationSettings(
        screenSize: screenSize,
        modeColor: _getModeColor('Navigation'),
      );
    } else if (category == 'General') {
      return GeneralSettings(
        screenSize: screenSize,
        modeColor: _getModeColor('General'),
      );
    }
    return Center(
      child: Text(
        '$category Settings Coming Soon',
        style: TextStyle(fontSize: screenSize.height * 0.04),
      ),
    );
  }
}

class GeneralSettings extends StatelessWidget {
  final Size screenSize;
  final Color modeColor;

  const GeneralSettings({
    super.key,
    required this.screenSize,
    required this.modeColor,
  });

  @override
  Widget build(BuildContext context) {
    return Consumer<SettingsProvider>(
      builder: (context, settings, child) {
        return SingleChildScrollView(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              SettingHeader(
                title: 'General Settings',
                icon: FontAwesomeIcons.gear,
                screenSize: screenSize,
                modeColor: modeColor,
              ),
              SizedBox(height: screenSize.height * 0.02),
              SettingCard(
                title: 'Camera Image Topic',
                description: 'Set topic for compressed image stream\n'
                    'Message type: sensor_msgs/CompressedImage',
                modeColor: modeColor,
                screenSize: screenSize,
                content: CameraTopicInput(
                  initialValue: settings.cameraImageTopic,
                  onChanged: settings.setCameraImageTopic,
                  screenSize: screenSize,
                  modeColor: modeColor,
                  enabled: settings.cameraEnabled,
                  onEnabledChanged: settings.setCameraEnabled,
                ),
              ),
            ],
          ),
        );
      },
    );
  }
}
