import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:provider/provider.dart';
import '../../providers/settings_provider.dart';
import 'widgets/setting_card.dart';
import 'widgets/setting_header.dart';
import 'widgets/velocity_control.dart';

class TeleopSettings extends StatelessWidget {
  final Size screenSize;
  final Color modeColor;

  const TeleopSettings({
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
                title: 'Teleop Settings',
                icon: FontAwesomeIcons.gamepad,
                screenSize: screenSize,
                modeColor: modeColor,
              ),

              SizedBox(height: screenSize.height * 0.02),

              // Settings content
              Row(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  // Left column
                  Expanded(
                    flex: 1,
                    child: SettingCard(
                      title: 'CMD_VEL Topic',
                      description: 'Set the topic name for velocity commands',
                      modeColor: modeColor,
                      screenSize: screenSize,
                      content: _buildTopicInput(settings),
                    ),
                  ),
                  SizedBox(width: screenSize.width * 0.02),
                  // Right column
                  Expanded(
                    flex: 1,
                    child: Column(
                      children: [
                        // Linear velocity setting
                        SettingCard(
                          title: 'Linear Velocity',
                          description: 'Set maximum linear velocity (m/s)',
                          modeColor: modeColor,
                          screenSize: screenSize,
                          content: VelocityControl(
                            value: settings.linearVelocity,
                            onIncrement: settings.incrementLinearVelocity,
                            onDecrement: settings.decrementLinearVelocity,
                            onChanged: (value) {
                              if (value != null)
                                settings.setLinearVelocity(value);
                            },
                            screenSize: screenSize,
                            modeColor: modeColor,
                          ),
                        ),
                        SizedBox(height: screenSize.height * 0.02),
                        // Angular velocity setting
                        SettingCard(
                          title: 'Angular Velocity',
                          description: 'Set maximum angular velocity (rad/s)',
                          modeColor: modeColor,
                          screenSize: screenSize,
                          content: VelocityControl(
                            value: settings.angularVelocity,
                            onIncrement: settings.incrementAngularVelocity,
                            onDecrement: settings.decrementAngularVelocity,
                            onChanged: (value) {
                              if (value != null)
                                settings.setAngularVelocity(value);
                            },
                            screenSize: screenSize,
                            modeColor: modeColor,
                          ),
                        ),
                      ],
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

  Widget _buildTopicInput(SettingsProvider settings) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        TextFormField(
          initialValue: settings.cmdVelTopic,
          style: TextStyle(fontSize: screenSize.height * 0.025),
          decoration: InputDecoration(
            hintText: 'Enter topic name (e.g. cmd_vel)',
            hintStyle: TextStyle(color: Colors.grey.shade700),
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(8),
              borderSide: BorderSide(color: modeColor),
            ),
            focusedBorder: OutlineInputBorder(
              borderRadius: BorderRadius.circular(8),
              borderSide: BorderSide(color: modeColor, width: 2),
            ),
            contentPadding: EdgeInsets.symmetric(
              horizontal: screenSize.width * 0.015,
              vertical: screenSize.height * 0.015,
            ),
          ),
          onChanged: settings.setCmdVelTopic,
        ),
      ],
    );
  }
}
