// Generated by ros_to_dart_converter
// Do not edit manually

// ignore_for_file: non_constant_identifier_names

import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

import 'package:std_msgs/msg.dart' as std_msgs;
import 'RegionOfInterest.dart';

class CameraInfo extends RosMessage<CameraInfo> {
  std_msgs.Header header;
  int height;
  int width;
  String distortion_model;
  List<double> d;
  List<double> k;
  List<double> r;
  List<double> p;
  int binning_x;
  int binning_y;
  RegionOfInterest roi;

  static CameraInfo $prototype = CameraInfo();

  // Constructor
  CameraInfo(
      {std_msgs.Header? header,
      int? height,
      int? width,
      String? distortion_model,
      List<double>? d,
      List<double>? k,
      List<double>? r,
      List<double>? p,
      int? binning_x,
      int? binning_y,
      RegionOfInterest? roi})
      : header = header ?? std_msgs.Header(),
        height = height ?? 0,
        width = width ?? 0,
        distortion_model = distortion_model ?? "",
        d = d ?? const [],
        k = k ?? const [],
        r = r ?? const [],
        p = p ?? const [],
        binning_x = binning_x ?? 0,
        binning_y = binning_y ?? 0,
        roi = roi ?? RegionOfInterest();

  @override
  String get fullType => 'sensor_msgs/msg/CameraInfo';

  @override
  String get messageDefinition => '''std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
RegionOfInterest roi''';

  @override
  int getMessageSize() {
    return (header.getMessageSize() +
            4 +
            4 +
            4 +
            distortion_model.length +
            4 +
            (d.length * 8) +
            4 +
            (k.length * 8) +
            4 +
            (r.length * 8) +
            4 +
            (p.length * 8) +
            4 +
            4 +
            roi.getMessageSize())
        .toInt();
  }

  @override
  Map<String, dynamic> toJson() => {
        'header': header.toJson(),
        'height': height,
        'width': width,
        'distortion_model': distortion_model,
        'd': d,
        'k': k,
        'r': r,
        'p': p,
        'binning_x': binning_x,
        'binning_y': binning_y,
        'roi': roi.toJson()
      };

  @override
  String toJsonString() => json.encode(toJson());

  @override
  CameraInfo fromJson(Map<String, dynamic> jsonMap) {
    return CameraInfo(
        header: std_msgs.Header()
            .fromJson(jsonMap['header'] as Map<String, dynamic>),
        height: jsonMap['height'] as int,
        width: jsonMap['width'] as int,
        distortion_model: jsonMap['distortion_model'] as String,
        d: (jsonMap['d'] as List).cast<double>(),
        k: (jsonMap['k'] as List).cast<double>(),
        r: (jsonMap['r'] as List).cast<double>(),
        p: (jsonMap['p'] as List).cast<double>(),
        binning_x: jsonMap['binning_x'] as int,
        binning_y: jsonMap['binning_y'] as int,
        roi: RegionOfInterest()
            .fromJson(jsonMap['roi'] as Map<String, dynamic>));
  }
}
