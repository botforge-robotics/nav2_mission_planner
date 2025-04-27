class DefaultSettings {
  // Teleop Settings
  static const String cmdVelTopic = '/cmd_vel';
  static const double defaultLinearVelocity = 1.0;
  static const double defaultAngularVelocity = 1.0;
  static const double velocityStep = 0.05;

  // Min-Max values
  static const double minVelocity = 0.0;
  static const double maxVelocity = 2.0;

  // Mapping Settings

  static const String defaultMapsFolder = 'slam_toolbox/maps';
  static const String defaultMappingLaunchFile = 'slam_toolbox/online_async';

  // Navigation Settings
  static const String defaultNavigationLaunchFile = 'nav2_bringup/navigation';

  // Camera Topics
  static const String defaultCameraTopic = '';

  // Add new defaults
  static const String defaultOdomTopic = '/odom';
  static const String defaultScanTopic = '/scan';

  static const defaultSaveMapLaunchFile = 'nav2_map_server/map_saver';
}
