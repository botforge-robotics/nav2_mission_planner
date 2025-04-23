import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:nav2_mission_planner/theme/app_theme.dart';
import 'package:provider/provider.dart';
import '../../providers/settings_provider.dart';
import 'widgets/setting_card.dart';
import 'widgets/setting_header.dart';
import 'widgets/launch_file_input.dart';
import 'widgets/arguments_list.dart';

class NavigationSettings extends StatelessWidget {
  final Size screenSize;
  final Color modeColor;

  const NavigationSettings({
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
              // Header
              SettingHeader(
                title: 'Navigation Settings',
                icon: FontAwesomeIcons.locationArrow,
                screenSize: screenSize,
                modeColor: modeColor,
              ),

              SizedBox(height: screenSize.height * 0.02),

              // Settings content in two columns
              Row(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  // Left column
                  Expanded(
                    flex: 1,
                    child: SettingCard(
                      title: 'Navigation Launch File',
                      description: 'Set the launch file for navigation',
                      modeColor: modeColor,
                      screenSize: screenSize,
                      content: LaunchFileInput(
                        initialValue: settings.navigationLaunchFile,
                        onChanged: settings.setNavigationLaunchFile,
                        screenSize: screenSize,
                        modeColor: modeColor,
                        hintText: 'nav2_bringup/navigation',
                      ),
                    ),
                  ),

                  SizedBox(width: screenSize.width * 0.02),

                  // Right column
                  Expanded(
                    flex: 1,
                    child: SettingCard(
                      title: 'Launch Arguments',
                      description:
                          'Add command-line arguments for the navigation launch file',
                      modeColor: modeColor,
                      screenSize: screenSize,
                      content: ArgumentsList(
                        arguments: settings.navigationArgs,
                        onRemove: settings.removeNavigationArg,
                        onAdd: settings.addNavigationArg,
                        onUpdate: settings.updateNavigationArg,
                        screenSize: screenSize,
                        modeColor: modeColor,
                      ),
                    ),
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
