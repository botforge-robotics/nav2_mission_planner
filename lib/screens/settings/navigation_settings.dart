import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
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
    return Consumer<SettingsProvider>(builder: (context, settings, child) {
      return SingleChildScrollView(
        child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          // Header
          SettingHeader(
            title: 'Navigation Settings',
            icon: FontAwesomeIcons.route,
            screenSize: screenSize,
            modeColor: modeColor,
          ),

          SizedBox(height: screenSize.height * 0.02),

          SettingCard(
            title: 'Navigation Launch File',
            description: 'Set the launch file for navigation',
            modeColor: modeColor,
            screenSize: screenSize,
            content: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                LaunchFileInput(
                  initialValue: settings.navigationLaunchFile,
                  onChanged: settings.setNavigationLaunchFile,
                  screenSize: screenSize,
                  modeColor: modeColor,
                  hintText: 'nav2_bringup/navigation',
                ),
                SizedBox(height: screenSize.height * 0.02),
                ArgumentsList(
                  arguments: settings.navigationArgs,
                  onRemove: settings.removeNavigationArg,
                  onAdd: settings.addNavigationArg,
                  onUpdate: settings.updateNavigationArg,
                  screenSize: screenSize,
                  modeColor: modeColor,
                ),
                SizedBox(height: screenSize.height * 0.015),
                Text(
                  'Command will be: ros2 launch [input].launch.py [arguments]',
                  style: TextStyle(
                    fontSize: 10,
                    color: Colors.grey.shade400,
                    fontStyle: FontStyle.italic,
                  ),
                ),
                Text(
                  'Note: The "map_name" argument will be asked when starting navigation. Please make sure the argument name is correct in your launch file.',
                  style: TextStyle(
                    fontSize: 10,
                    color: Colors.orange.shade300,
                  ),
                ),
              ],
            ),
          ),
        ]),
      );
    });
  }
}
