import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:nav2_mission_planner/screens/settings/widgets/setting_card.dart';
import 'package:provider/provider.dart';
import '../../providers/settings_provider.dart';
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
              SettingHeader(
                title: 'Teleop Settings',
                icon: FontAwesomeIcons.gamepad,
                screenSize: screenSize,
                modeColor: modeColor,
              ),
              SizedBox(height: screenSize.height * 0.02),
              SettingCard(
                title: 'CMD_VEL Topic',
                description: 'Set the topic name for velocity commands',
                content: _buildTopicInput(settings),
                modeColor: modeColor,
                screenSize: screenSize,
              ),
              _buildSettingRow(
                title: 'Linear Velocity',
                description: 'Set maximum linear velocity (m/s)',
                content: VelocityControl(
                  value: settings.linearVelocity,
                  onIncrement: settings.incrementLinearVelocity,
                  onDecrement: settings.decrementLinearVelocity,
                  onChanged: (value) {
                    if (value != null) settings.setLinearVelocity(value);
                  },
                  screenSize: screenSize,
                  modeColor: modeColor,
                ),
              ),
              _buildSettingRow(
                title: 'Angular Velocity',
                description: 'Set maximum angular velocity (rad/s)',
                content: VelocityControl(
                  value: settings.angularVelocity,
                  onIncrement: settings.incrementAngularVelocity,
                  onDecrement: settings.decrementAngularVelocity,
                  onChanged: (value) {
                    if (value != null) settings.setAngularVelocity(value);
                  },
                  screenSize: screenSize,
                  modeColor: modeColor,
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Widget _buildSettingRow({
    required String title,
    required String description,
    required Widget content,
  }) {
    return Container(
      padding: EdgeInsets.all(screenSize.width * 0.02),
      margin: EdgeInsets.only(bottom: screenSize.height * 0.01),
      decoration: BoxDecoration(
        color: Colors.grey.shade900,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: modeColor.withOpacity(0.3), width: 2),
      ),
      child: Row(
        children: [
          // Left side - Title and Description
          Expanded(
            flex: 2,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: TextStyle(
                    fontSize: screenSize.height * 0.03,
                    fontWeight: FontWeight.bold,
                    color: modeColor,
                  ),
                ),
                SizedBox(height: screenSize.height * 0.01),
                Text(
                  description,
                  style: TextStyle(
                    fontSize: screenSize.height * 0.024,
                    color: Colors.grey,
                  ),
                ),
              ],
            ),
          ),
          SizedBox(width: screenSize.width * 0.02),
          // Right side - Content
          Expanded(
            flex: 3,
            child: content,
          ),
        ],
      ),
    );
  }

  Widget _buildTopicInput(SettingsProvider settings) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      mainAxisSize: MainAxisSize.min,
      children: [
        TextFormField(
          initialValue: settings.cmdVelTopic,
          style: TextStyle(fontSize: screenSize.height * 0.025),
          decoration: InputDecoration(
            hintText: 'Enter topic name (e.g. cmd_vel)',
            hintStyle: TextStyle(color: Colors.grey.shade700),
            border: UnderlineInputBorder(
              borderSide: BorderSide(color: modeColor),
            ),
            focusedBorder: UnderlineInputBorder(
              borderSide: BorderSide(color: modeColor, width: 2),
            ),
            contentPadding: EdgeInsets.symmetric(
              horizontal: screenSize.width * 0.015,
              vertical: screenSize.height * 0.015,
            ),
          ),
          onChanged: settings.setCmdVelTopic,
        ),
        SizedBox(height: screenSize.height * 0.01),
        Text(
          'Type: geometry_msgs/msg/Twist',
          style: TextStyle(
            fontSize: screenSize.height * 0.02,
            color: Colors.grey.shade400,
            fontStyle: FontStyle.italic,
          ),
        ),
      ],
    );
  }
}
