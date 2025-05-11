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
  static const String defaultMappingOdomTopic = '/odom';
  static const String defaultMappingOdomTopicType = 'nav_msgs/msg/Odometry';
  // Navigation Settings
  static const String defaultNavigationLaunchFile = 'rio_navigation/navigation';
  static const String defaultNavigationOdomTopic = '/amcl_pose';
  static const String defaultNavigationOdomTopicType =
      'geometry_msgs/msg/PoseWithCovarianceStamped';
  // Camera Topics
  static const String defaultCameraTopic = '';

  // Add new defaults
  static const String defaultOdomTopic = '/odom';
  static const String defaultOdomTopicType = 'nav_msgs/msg/Odometry';
  static const String defaultLidarTopic = '/scan';

  static const defaultSaveMapLaunchFile = 'rio_mapping/save_map';

  // Add these new constants
  static const bool defaultCameraVisible = true;
  static const bool defaultJoystickVisible = true;
}
