import 'package:flutter/material.dart';

enum AppModes { teleop, mapping, navigation, settings }

class ModeColors {
  static const Map<AppModes, Color> modeColorMap = {
    AppModes.teleop: Colors.orange,
    AppModes.mapping: Colors.green,
    AppModes.navigation: Colors.blue,
    AppModes.settings: Colors.teal,
  };
}
