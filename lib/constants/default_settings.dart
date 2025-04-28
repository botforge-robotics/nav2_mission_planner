class DefaultSettings {
  // Teleop Settings
  static const String cmdVelTopic = '/cmd_vel';
  static const double defaultLinearVelocity = 1.0;
  static const double defaultAngularVelocity = 1.0;
  static const double velocityStep = 0.05;

  // Min-Max values
  static const double minVelocity = 0.0;
  static const double maxVelocity = 100.0;

  // Mapping Settings

  static const String defaultMapsFolder = 'rio_mapping/maps';
  static const String defaultMappingLaunchFile = 'rio_mapping/mapping';

  // Navigation Settings
  static const String defaultNavigationLaunchFile = 'rio_navigation/navigation';

  // Camera Topics
  static const String defaultCameraTopic = '';

  // Add new defaults
  static const String defaultOdomTopic = '/odom';

  static const defaultSaveMapLaunchFile = 'rio_mapping/save_map';
}
