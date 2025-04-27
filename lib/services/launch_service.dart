import 'package:flutter/material.dart';
import 'package:nav2_mission_planner/providers/settings_provider.dart';
import 'package:nav2_mission_planner/providers/connection_provider.dart';
import 'package:nav2_mission_planner_interfaces/srv.dart';
import 'package:ros2_api/ros2_api.dart';
import 'package:provider/provider.dart';

class LaunchManager extends ChangeNotifier {
  final Map<String, String> _activeLaunches = {}; // unique_id -> description

  Map<String, String> get activeLaunches => Map.unmodifiable(_activeLaunches);

  Future<bool> startMapping(BuildContext context) async {
    final settings = Provider.of<SettingsProvider>(context, listen: false);
    final connection = Provider.of<ConnectionProvider>(context, listen: false);

    final parts = settings.mappingLaunchFile.split('/');
    if (parts.length != 2) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Invalid launch file format'),
          backgroundColor: Colors.red,
        ),
      );
      return false;
    }

    // Ensure the launch file ends with ".launch.py"
    String launchFile = parts[1];
    if (!launchFile.endsWith('.launch.py')) {
      launchFile = '$launchFile.launch.py';
    }

    try {
      final serviceClient = ServiceClient<LaunchWithArgs, LaunchWithArgsRequest,
          LaunchWithArgsResponse>(
        name: '/launch_with_args',
        ros2: connection.ros2Client!,
        type: LaunchWithArgs().fullType,
        serviceType: LaunchWithArgs(),
      );

      final request = LaunchWithArgsRequest(
        package: parts[0],
        launch_file: launchFile,
        arguments: settings.mappingArgs.join(' '),
      );
      print('Request: $request');
      final response = await serviceClient.call(request);
      if (response?.success ?? false) {
        // Track the unique_id and description
        _activeLaunches[response!.unique_id] =
            '${parts[0]}/${parts[1]} ${settings.mappingArgs.join(' ')}';
        notifyListeners();
        return true;
      } else {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text(
                'Failed to start mapping: ${response?.message ?? "Unknown error"}'),
            backgroundColor: Colors.red,
          ),
        );
        return false;
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Error calling launch service: $e'),
          backgroundColor: Colors.red,
        ),
      );
      return false;
    }
  }

  Future<void> stopLaunch(BuildContext context, String uniqueId) async {
    final connection = Provider.of<ConnectionProvider>(context, listen: false);
    try {
      final serviceClient =
          ServiceClient<StopLaunch, StopLaunchRequest, StopLaunchResponse>(
        name: '/stop_launch',
        ros2: connection.ros2Client!,
        type: StopLaunch().fullType,
        serviceType: StopLaunch(),
      );

      final request = StopLaunchRequest(unique_id: uniqueId);
      await serviceClient.call(request);
      _activeLaunches.remove(uniqueId);
      notifyListeners();
    } catch (e) {
      print('Error stopping launch: $e');
      rethrow;
    }
  }
}
