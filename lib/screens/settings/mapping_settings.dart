import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:provider/provider.dart';
import '../../providers/settings_provider.dart';
import 'widgets/setting_card.dart';
import 'widgets/setting_header.dart';
import '../../theme/app_theme.dart';
import 'widgets/maps_path_input.dart';
import 'widgets/launch_file_input.dart';
import 'widgets/arguments_list.dart';

class MappingSettings extends StatelessWidget {
  final Size screenSize;
  final Color modeColor;

  const MappingSettings({
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
                title: 'Mapping Settings',
                icon: FontAwesomeIcons.map,
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
                    child: Column(
                      children: [
                        // Maps Path Setting
                        SettingCard(
                          title: 'Maps Storage Path',
                          description: 'Set the path where maps will be saved',
                          modeColor: modeColor,
                          screenSize: screenSize,
                          content: MapsPathInput(
                            initialValue: settings.mapsPath,
                            onChanged: settings.setMapsPath,
                            screenSize: screenSize,
                            modeColor: modeColor,
                          ),
                        ),
                        SizedBox(height: screenSize.height * 0.02),
                        // Launch File Setting
                        SettingCard(
                          title: 'Mapping Launch File',
                          description: 'Set the launch file for mapping',
                          modeColor: modeColor,
                          screenSize: screenSize,
                          content: LaunchFileInput(
                            initialValue: settings.mappingLaunchFile,
                            onChanged: settings.setMappingLaunchFile,
                            screenSize: screenSize,
                            modeColor: modeColor,
                            hintText: 'cartographer_ros/cartographer',
                          ),
                        ),
                      ],
                    ),
                  ),

                  SizedBox(width: screenSize.width * 0.02),

                  // Right column
                  Expanded(
                    flex: 1,
                    child: SettingCard(
                      title: 'Launch Arguments',
                      description:
                          'Add command-line arguments for the mapping launch file',
                      modeColor: modeColor,
                      screenSize: screenSize,
                      content: ArgumentsList(
                        arguments: settings.mappingArgs,
                        onRemove: settings.removeMappingArg,
                        onAdd: settings.addMappingArg,
                        onUpdate: settings.updateMappingArg,
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
