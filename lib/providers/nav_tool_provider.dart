import 'package:flutter/material.dart';

class NavToolProvider extends ChangeNotifier {
  bool _poseEstimationEnabled = false;
  String _selectedTool = '';
  bool _goalMode = false;

  bool get poseEstimationEnabled => _poseEstimationEnabled;
  String get selectedTool => _selectedTool;
  bool get goalMode => _goalMode;

  void setPoseEstimationEnabled(bool value) {
    _poseEstimationEnabled = value;
    _selectedTool = value ? 'localization' : '';
    notifyListeners();
  }

  void setGoalEnabled(bool value) {
    _goalMode = value;
    _selectedTool = value ? 'goal' : '';
    notifyListeners();
  }

  void setSelectedTool(String tool) {
    _selectedTool = tool;
    _poseEstimationEnabled = tool == 'localization';
    _goalMode = tool == 'goal';
    notifyListeners();
  }
}
