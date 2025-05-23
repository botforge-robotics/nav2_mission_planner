
// Generated by ros_to_dart_converter
// Do not edit manually

import 'package:ros2_msg_utils/ros2_msg_utils.dart';
import 'dart:convert';

class Float32 extends RosMessage<Float32> {
  
  
  double data;

  static Float32 $prototype = Float32();

  // Constructor
  Float32({
    this.data = 0.0
  });

  @override
  String get fullType => 'std_msgs/msg/Float32';

  @override
  String get messageDefinition => '''float32 data''';

  @override
  int getMessageSize() {
     return (4).toInt();
  }

  @override
  Map<String, dynamic> toJson() => {
    'data': data
  };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  Float32 fromJson(Map<String, dynamic> jsonMap) {
    return Float32(
      data: jsonMap['data'] as double
    );
  }
}
