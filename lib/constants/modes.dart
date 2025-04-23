import 'package:flutter/material.dart';

enum AppModes { teleop, mapping, navigation, mission, settings }

class ModeColors {
  static const Map<AppModes, Color> modeColorMap = {
    AppModes.teleop: Colors.orange,
    AppModes.mapping: Colors.green,
    AppModes.navigation: Colors.blue,
    AppModes.mission: Colors.deepPurple,
    AppModes.settings: Colors.teal,
  };
}
