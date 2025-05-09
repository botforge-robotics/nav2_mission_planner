import 'dart:async';

import 'package:flutter/material.dart';
import 'package:ros2_api/ros2_api.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'dart:convert';
import 'dart:io';

class ConnectionProvider extends ChangeNotifier {
  String _ip = '';
  String _port = '9090';
  bool _isConnected = false;
  List<Map<String, String>> _recentConnections = [];
  Ros2? _ros2Client;

  // Add this stream controller
  final StreamController<ConnectionState> _connectionController =
      StreamController<ConnectionState>.broadcast();

  Stream<ConnectionState> get connectionStream => _connectionController.stream;

  // Getters
  String get ip => _ip;
  String get port => _port;
  bool get isConnected => _isConnected;
  List<Map<String, String>> get recentConnections => _recentConnections;
  Ros2? get ros2Client => _ros2Client;

  ConnectionProvider() {
    _loadRecentConnections();
  }

  // Load recent connections from shared preferences
  Future<void> _loadRecentConnections() async {
    final prefs = await SharedPreferences.getInstance();
    final String? connectionsJson = prefs.getString('recentConnections');
    if (connectionsJson != null) {
      List<dynamic> connectionsList = json.decode(connectionsJson);
      _recentConnections = List<Map<String, String>>.from(
          connectionsList.map((item) => Map<String, String>.from(item)));
      notifyListeners();
    }
  }

  // Save recent connections to shared preferences
  Future<void> _saveRecentConnections() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('recentConnections', json.encode(_recentConnections));
  }

  // Add this helper method to check IP availability
  Future<bool> _isRobotAvailable(String ip) async {
    try {
      final socket =
          await Socket.connect(ip, 9090, timeout: Duration(seconds: 2));
      socket.destroy();
      return true;
    } catch (_) {
      return false;
    }
  }

  // Modified connect method
  Future<bool> connect(String ip, String port) async {
    try {
      _connectionController.add(ConnectionState.connecting);
      if (!await _isRobotAvailable(ip)) {
        throw Exception('Robot not available at $ip');
      }

      _ros2Client = Ros2(url: 'ws://$ip:$port');
      _ros2Client!.connect();
      _isConnected = true;
      _ip = ip;
      _port = port;

      // Add to recent connections if not exists
      final connection = {'ip': ip, 'port': port};

      // Check if the connection already exists by comparing ip and port values
      bool connectionExists = _recentConnections
          .any((conn) => conn['ip'] == ip && conn['port'] == port);

      if (!connectionExists) {
        // If this is a new connection, add it to the beginning of the list
        _recentConnections.insert(0, connection);

        // Limit to last 10 connections
        if (_recentConnections.length > 10) {
          _recentConnections.removeLast();
        }

        await _saveRecentConnections(); // Save after updating
      } else {
        // If it exists already, move it to the top of the list
        _recentConnections
            .removeWhere((conn) => conn['ip'] == ip && conn['port'] == port);
        _recentConnections.insert(0, connection);
        await _saveRecentConnections();
      }

      notifyListeners();
      _connectionController.add(ConnectionState.connected);
      return true;
    } catch (e) {
      _connectionController.add(ConnectionState.disconnected);
      rethrow;
    }
  }

  // Disconnect from ROS2
  Future<void> disconnect() async {
    _connectionController.add(ConnectionState.disconnecting);
    try {
      await _ros2Client?.close();
      _isConnected = false;
      notifyListeners();
    } catch (e) {
      rethrow;
    } finally {
      _connectionController.add(ConnectionState.disconnected);
    }
  }
}

enum ConnectionState { connecting, connected, disconnecting, disconnected }
