import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import '../constants/default_settings.dart';
import 'dart:convert';

class SettingsProvider extends ChangeNotifier {
  // Teleop Settings
  String _cmdVelTopic = DefaultSettings.cmdVelTopic;
  double _linearVelocity = DefaultSettings.defaultLinearVelocity;
  double _angularVelocity = DefaultSettings.defaultAngularVelocity;

  // Mapping Settings
  String _mapsPath = DefaultSettings.defaultMapsFolder;
  String _mappingLaunchFile = DefaultSettings.defaultMappingLaunchFile;
  List<Map<String, String>> _mappingArgs = [];

  // Navigation Settings
  String _navigationLaunchFile = DefaultSettings.defaultNavigationLaunchFile;
  List<Map<String, String>> _navigationArgs = [];

  // General Settings
  String _cameraImageTopic = '/camera/image/compressed';

  // Add new property
  bool _cameraEnabled = false;

  // Getters
  String get cmdVelTopic => _cmdVelTopic;
  double get linearVelocity => _linearVelocity;
  double get angularVelocity => _angularVelocity;
  String get mapsPath => _mapsPath;
  String get mappingLaunchFile => _mappingLaunchFile;
  List<Map<String, String>> get mappingArgs => _mappingArgs;
  String get navigationLaunchFile => _navigationLaunchFile;
  List<Map<String, String>> get navigationArgs => _navigationArgs;
  String get cameraImageTopic => _cameraImageTopic;
  bool get cameraEnabled => _cameraEnabled;

  SettingsProvider() {
    _loadSettings();
  }

  // Load settings from shared preferences
  Future<void> _loadSettings() async {
    final prefs = await SharedPreferences.getInstance();
    _cmdVelTopic =
        prefs.getString('cmdVelTopic') ?? DefaultSettings.cmdVelTopic;
    _linearVelocity = prefs.getDouble('linearVelocity') ??
        DefaultSettings.defaultLinearVelocity;
    _angularVelocity = prefs.getDouble('angularVelocity') ??
        DefaultSettings.defaultAngularVelocity;
    _mapsPath =
        prefs.getString('mapsPath') ?? DefaultSettings.defaultMapsFolder;
    _mappingLaunchFile = prefs.getString('mappingLaunchFile') ??
        DefaultSettings.defaultMappingLaunchFile;
    _navigationLaunchFile = prefs.getString('navigationLaunchFile') ??
        DefaultSettings.defaultNavigationLaunchFile;

    // Load mapping and navigation args
    final String? mappingArgsJson = prefs.getString('mappingArgs');
    if (mappingArgsJson != null) {
      List<dynamic> argsList = json.decode(mappingArgsJson);
      _mappingArgs = List<Map<String, String>>.from(
          argsList.map((item) => Map<String, String>.from(item)));
    }

    final String? navigationArgsJson = prefs.getString('navigationArgs');
    if (navigationArgsJson != null) {
      List<dynamic> argsList = json.decode(navigationArgsJson);
      _navigationArgs = List<Map<String, String>>.from(
          argsList.map((item) => Map<String, String>.from(item)));
    }

    // Add camera topic loading
    _cameraImageTopic =
        prefs.getString('cameraImageTopic') ?? '/camera/image/compressed';

    // Update _cameraEnabled
    _cameraEnabled = prefs.getBool('cameraEnabled') ?? false;

    notifyListeners();
  }

  // Save settings to shared preferences
  Future<void> _saveSettings() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('cmdVelTopic', _cmdVelTopic);
    await prefs.setDouble('linearVelocity', _linearVelocity);
    await prefs.setDouble('angularVelocity', _angularVelocity);
    await prefs.setString('mapsPath', _mapsPath);
    await prefs.setString('mappingLaunchFile', _mappingLaunchFile);
    await prefs.setString('navigationLaunchFile', _navigationLaunchFile);
    await prefs.setString('mappingArgs', json.encode(_mappingArgs));
    await prefs.setString('navigationArgs', json.encode(_navigationArgs));
    await prefs.setString('cameraImageTopic', _cameraImageTopic);
    await prefs.setBool('cameraEnabled', _cameraEnabled);
  }

  // Setters with validation
  void setCmdVelTopic(String topic) {
    _cmdVelTopic = topic;
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void setLinearVelocity(double velocity) {
    _linearVelocity = velocity.clamp(
      DefaultSettings.minVelocity,
      DefaultSettings.maxVelocity,
    );
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void setAngularVelocity(double velocity) {
    _angularVelocity = velocity.clamp(
      DefaultSettings.minVelocity,
      DefaultSettings.maxVelocity,
    );
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  // Helper methods for step increment/decrement
  void incrementLinearVelocity() {
    setLinearVelocity(_linearVelocity + DefaultSettings.velocityStep);
  }

  void decrementLinearVelocity() {
    setLinearVelocity(_linearVelocity - DefaultSettings.velocityStep);
  }

  void incrementAngularVelocity() {
    setAngularVelocity(_angularVelocity + DefaultSettings.velocityStep);
  }

  void decrementAngularVelocity() {
    setAngularVelocity(_angularVelocity - DefaultSettings.velocityStep);
  }

  // Mapping Setters
  void setMapsPath(String path) {
    _mapsPath = path;
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void setMappingLaunchFile(String file) {
    _mappingLaunchFile = file;
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void addMappingArg(String name, String value) {
    _mappingArgs.add({'name': name, 'value': value});
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void removeMappingArg(int index) {
    if (index >= 0 && index < _mappingArgs.length) {
      _mappingArgs.removeAt(index);
      _saveSettings(); // Save after updating
      notifyListeners();
    }
  }

  void updateMappingArg(int index, String name, String value) {
    if (index >= 0 && index < _mappingArgs.length) {
      _mappingArgs[index] = {'name': name, 'value': value};
      _saveSettings(); // Save after updating
      notifyListeners();
    }
  }

  // Navigation Setters
  void setNavigationLaunchFile(String file) {
    _navigationLaunchFile = file;
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void addNavigationArg(String name, String value) {
    _navigationArgs.add({'name': name, 'value': value});
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void removeNavigationArg(int index) {
    if (index >= 0 && index < _navigationArgs.length) {
      _navigationArgs.removeAt(index);
      _saveSettings(); // Save after updating
      notifyListeners();
    }
  }

  void updateNavigationArg(int index, String name, String value) {
    if (index >= 0 && index < _navigationArgs.length) {
      _navigationArgs[index] = {'name': name, 'value': value};
      _saveSettings(); // Save after updating
      notifyListeners();
    }
  }

  // General Setters
  void setCameraImageTopic(String topic) {
    _cameraImageTopic = topic;
    _saveSettings(); // Add this line to persist
    notifyListeners();
  }

  // Add new setter
  void setCameraEnabled(bool enabled) {
    _cameraEnabled = enabled;
    _saveSettings();
    notifyListeners();
  }
}
