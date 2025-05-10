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

  // Add new save map settings
  String _saveMapLaunchFile = DefaultSettings.defaultSaveMapLaunchFile;
  List<Map<String, String>> _saveMapArgs = [];

  // Navigation Settings
  String _navigationLaunchFile = DefaultSettings.defaultNavigationLaunchFile;
  List<Map<String, String>> _navigationArgs = [];

  // General Settings
  String _cameraImageTopic = '/camera/image/compressed';

  // Add new property
  bool _cameraEnabled = false;

  // Add new properties
  String _odomTopic = DefaultSettings.defaultOdomTopic;

  // Add to existing properties
  String _lidarTopic = DefaultSettings.defaultLidarTopic;

  // Add these to the class
  bool _cameraVisible = DefaultSettings.defaultCameraVisible;
  bool _joystickVisible = DefaultSettings.defaultJoystickVisible;

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
  String get odomTopic => _odomTopic;
  String get saveMapLaunchFile => _saveMapLaunchFile;
  List<Map<String, String>> get saveMapArgs => _saveMapArgs;
  String get lidarTopic => _lidarTopic;
  bool get cameraVisible => _cameraVisible;
  bool get joystickVisible => _joystickVisible;

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
    _cameraImageTopic = prefs.getString('cameraImageTopic') ??
        DefaultSettings.defaultCameraTopic;

    // Update _cameraEnabled
    _cameraEnabled = prefs.getBool('cameraEnabled') ?? false;

    // Load new topics
    _odomTopic =
        prefs.getString('odomTopic') ?? DefaultSettings.defaultOdomTopic;

    // Load save map settings
    _saveMapLaunchFile = prefs.getString('saveMapLaunchFile') ??
        DefaultSettings.defaultSaveMapLaunchFile;

    final String? saveMapArgsJson = prefs.getString('saveMapArgs');
    if (saveMapArgsJson != null) {
      List<dynamic> argsList = json.decode(saveMapArgsJson);
      _saveMapArgs = List<Map<String, String>>.from(
          argsList.map((item) => Map<String, String>.from(item)));
    }

    // Load lidar topic and enabled status
    _lidarTopic =
        prefs.getString('lidarTopic') ?? DefaultSettings.defaultLidarTopic;

    // Load visibility settings
    _cameraVisible =
        prefs.getBool('cameraVisible') ?? DefaultSettings.defaultCameraVisible;
    _joystickVisible = prefs.getBool('joystickVisible') ??
        DefaultSettings.defaultJoystickVisible;

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
    await prefs.setString('odomTopic', _odomTopic);
    await prefs.setString('saveMapLaunchFile', _saveMapLaunchFile);
    await prefs.setString('saveMapArgs', json.encode(_saveMapArgs));
    await prefs.setString('lidarTopic', _lidarTopic);
    await prefs.setBool('cameraVisible', _cameraVisible);
    await prefs.setBool('joystickVisible', _joystickVisible);
  }

  // Setters with validation
  void setCmdVelTopic(String topic) {
    _cmdVelTopic = topic.trim();
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
    _mapsPath = path.trim();
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void setMappingLaunchFile(String file) {
    _mappingLaunchFile = file.trim();
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void addMappingArg(String name, String value) {
    _mappingArgs.add({'name': name.trim(), 'value': value.trim()});
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
      _mappingArgs[index] = {'name': name.trim(), 'value': value.trim()};
      _saveSettings(); // Save after updating
      notifyListeners();
    }
  }

  // Navigation Setters
  void setNavigationLaunchFile(String file) {
    _navigationLaunchFile = file.trim();
    _saveSettings(); // Save after updating
    notifyListeners();
  }

  void addNavigationArg(String name, String value) {
    _navigationArgs.add({'name': name.trim(), 'value': value.trim()});
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
      _navigationArgs[index] = {'name': name.trim(), 'value': value.trim()};
      _saveSettings(); // Save after updating
      notifyListeners();
    }
  }

  // General Setters
  void setCameraImageTopic(String topic) {
    _cameraImageTopic = topic.trim();
    _saveSettings();
    notifyListeners();
  }

  // Add new setter
  void setCameraEnabled(bool enabled) {
    _cameraEnabled = enabled;
    _saveSettings();
    notifyListeners();
  }

  // Add new setters
  void setOdomTopic(String topic) {
    _odomTopic = topic.trim();
    _saveSettings();
    notifyListeners();
  }

  // Save Map Configuration Setters
  void setSaveMapLaunchFile(String file) {
    _saveMapLaunchFile = file.trim();
    _saveSettings();
    notifyListeners();
  }

  void addSaveMapArg(String name, String value) {
    _saveMapArgs.add({'name': name.trim(), 'value': value.trim()});
    _saveSettings();
    notifyListeners();
  }

  void removeSaveMapArg(int index) {
    if (index >= 0 && index < _saveMapArgs.length) {
      _saveMapArgs.removeAt(index);
      _saveSettings();
      notifyListeners();
    }
  }

  void updateSaveMapArg(int index, String name, String value) {
    if (index >= 0 && index < _saveMapArgs.length) {
      _saveMapArgs[index] = {'name': name.trim(), 'value': value.trim()};
      _saveSettings();
      notifyListeners();
    }
  }

  // Lidar Setters
  void setLidarTopic(String topic) {
    _lidarTopic = topic.trim();
    _saveSettings();
    notifyListeners();
  }

  void toggleCameraVisibility() {
    _cameraVisible = !_cameraVisible;
    notifyListeners();
  }

  void toggleJoystickVisibility() {
    _joystickVisible = !_joystickVisible;
    notifyListeners();
  }
}
