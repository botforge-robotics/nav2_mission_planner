import 'dart:convert';
import 'package:ros2_msg_utils/ros2_msg_utils.dart';

class GetTimeRequest extends RosMessage<GetTimeRequest> {
  static GetTimeRequest $prototype = GetTimeRequest();

  @override
  String get fullType => 'rosapi_msgs/srv/GetTime_Request';

  @override
  String get messageDefinition => '';

  @override
  int getMessageSize() => 0;

  @override
  Map<String, dynamic> toJson() => {};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetTimeRequest fromJson(Map<String, dynamic> jsonMap) => GetTimeRequest();
}

class GetTimeResponse extends RosMessage<GetTimeResponse> {
  int sec;
  int nsec;

  static GetTimeResponse $prototype = GetTimeResponse();

  GetTimeResponse({this.sec = 0, this.nsec = 0});

  @override
  String get fullType => 'rosapi_msgs/srv/GetTime_Response';

  @override
  String get messageDefinition => '''int32 sec
int32 nsec''';

  @override
  int getMessageSize() => 8;

  @override
  Map<String, dynamic> toJson() => {'sec': sec, 'nsec': nsec};

  @override
  String toJsonString() => json.encode(toJson());

  @override
  GetTimeResponse fromJson(Map<String, dynamic> jsonMap) => GetTimeResponse(
        sec: jsonMap['sec'] as int,
        nsec: jsonMap['nsec'] as int,
      );
}

class GetTime extends RosServiceMessage<GetTimeRequest, GetTimeResponse> {
  @override
  GetTimeRequest get request => GetTimeRequest();

  @override
  GetTimeResponse get response => GetTimeResponse();

  @override
  String get fullType => 'rosapi_msgs/srv/GetTime';

  @override
  String get messageDefinition => '''---
int32 sec
int32 nsec''';
}
