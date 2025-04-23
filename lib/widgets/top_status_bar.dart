import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_theme.dart';
import '../providers/connection_provider.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart'; // Import for FontAwesome icons
import 'package:wifi_info_plugin_plus/wifi_info_plugin_plus.dart';

class TopStatusBar extends StatelessWidget {
  final String statusText;
  final Color statusColor;
  final double height;
  final IconData? icon;

  const TopStatusBar({
    super.key,
    required this.statusText,
    required this.statusColor,
    this.height = 50.0,
    this.icon,
  });

  @override
  Widget build(BuildContext context) {
    return Consumer<ConnectionProvider>(
      builder: (context, connection, child) {
        // Add WiFi info update
        WidgetsBinding.instance.addPostFrameCallback((_) {
          connection.updateWifiInfo();
        });

        // Determine status color based on connection
        final connectionStatusColor =
            connection.isConnected ? statusColor : Colors.red;
        final displayStatusText =
            connection.isConnected ? statusText : 'Disconnected';

        return Container(
          height: height,
          color: AppTheme.toolbarColor,
          child: Stack(
            children: [
              // Left-aligned app mode
              Positioned(
                left: 0,
                top: 0,
                bottom: 0,
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  decoration: BoxDecoration(
                    gradient: LinearGradient(
                      begin: Alignment.centerLeft,
                      end: Alignment.centerRight,
                      colors: [
                        connectionStatusColor,
                        connectionStatusColor.withOpacity(0.8),
                        connectionStatusColor.withOpacity(0.0),
                      ],
                      stops: const [0.0, 0.7, 1.0],
                    ),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      if (icon != null)
                        Icon(
                          icon,
                          color: _getTextColor(connectionStatusColor),
                          size: height * 0.4,
                        ),
                      SizedBox(width: 8),
                      Text(
                        displayStatusText,
                        style: TextStyle(
                          color: _getTextColor(connectionStatusColor),
                          fontSize: height * 0.32,
                          fontWeight: FontWeight.w500,
                        ),
                      ),
                    ],
                  ),
                ),
              ),

              // Centered app name
              Center(
                child: Text(
                  'Nav2 Mission Planner',
                  style: TextStyle(
                    color: _getTextColor(connectionStatusColor),
                    fontSize: height * 0.27,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),

              // Updated right-aligned network info
              Positioned(
                right: 16,
                top: 0,
                bottom: 0,
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    if (connection.isConnected)
                      Padding(
                        padding: const EdgeInsets.only(right: 20),
                        child: Row(
                          children: [
                            Column(
                              mainAxisSize: MainAxisSize.min,
                              crossAxisAlignment: CrossAxisAlignment.end,
                              children: [
                                Text(
                                  'RSSI: ${connection.signalStrength}',
                                  style: TextStyle(
                                    color: _getTextColor(connectionStatusColor),
                                    fontSize: height * 0.19,
                                  ),
                                ),
                                Text(
                                  'Robot IP: ${connection.ip}',
                                  style: TextStyle(
                                    color: _getTextColor(connectionStatusColor),
                                    fontSize: height * 0.19,
                                  ),
                                ),
                              ],
                            ),
                            SizedBox(width: 10),
                            Icon(Icons.wifi,
                                size: height * 0.35,
                                color: _getTextColor(connectionStatusColor)),
                          ],
                        ),
                      ),
                    _buildConnectButton(context, connection),
                  ],
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Widget _buildConnectButton(
      BuildContext context, ConnectionProvider connection) {
    return Padding(
      padding: EdgeInsets.symmetric(horizontal: height * 0.16),
      child: IconButton(
        icon: Row(
          children: [
            Icon(
              connection.isConnected ? Icons.link : Icons.link_off,
              color: connection.isConnected ? Colors.green : Colors.white,
              size: height * 0.6,
            ),
          ],
        ),
        onPressed: () {
          if (connection.isConnected) {
            _showDisconnectConfirmation(context, connection);
          } else {
            _showConnectionDialog(context);
          }
        },
      ),
    );
  }

  void _showDisconnectConfirmation(
      BuildContext context, ConnectionProvider connection) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        backgroundColor: AppTheme.toolbarColor,
        title: Text(
          'Disconnect',
          style: TextStyle(
            color: Colors.white,
            fontSize: height * 0.3,
          ),
        ),
        content: Text(
          'Are you sure you want to disconnect from ${connection.ip}:${connection.port}?',
          style: TextStyle(
            color: Colors.white70,
            fontSize: height * 0.25,
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text(
              'Cancel',
              style: TextStyle(
                color: Colors.grey[400],
                fontSize: height * 0.25,
              ),
            ),
          ),
          TextButton(
            onPressed: () {
              connection.disconnect();
              Navigator.of(context).pop();
            },
            child: Text(
              'Disconnect',
              style: TextStyle(
                color: Colors.red,
                fontSize: height * 0.25,
              ),
            ),
          ),
        ],
      ),
    );
  }

  void _showConnectionDialog(BuildContext context) {
    final ipController = TextEditingController();
    final portController = TextEditingController(text: '9090');

    // Use green color for the connection dialog
    final Color connectionColor = Colors.green;

    Navigator.of(context).push(
      PageRouteBuilder(
        opaque: false,
        pageBuilder: (context, animation, secondaryAnimation) {
          return Scaffold(
            backgroundColor: Colors.transparent,
            resizeToAvoidBottomInset: true,
            body: Stack(
              children: [
                // Semi-transparent backdrop
                GestureDetector(
                  onTap: () => Navigator.pop(context),
                  child: Container(
                    color: Colors.black54,
                    width: double.infinity,
                    height: double.infinity,
                  ),
                ),

                // Panel
                Positioned(
                  right: 0,
                  top: 0,
                  bottom: 0,
                  width: MediaQuery.of(context).size.width * 0.4,
                  child: SlideTransition(
                    position: Tween<Offset>(
                      begin: const Offset(1.0, 0.0),
                      end: Offset.zero,
                    ).animate(animation),
                    child: Material(
                      color: Colors.transparent,
                      child: Container(
                        decoration: BoxDecoration(
                          color: AppTheme.toolbarColor,
                          borderRadius: const BorderRadius.only(
                            topLeft: Radius.circular(12),
                            bottomLeft: Radius.circular(12),
                          ),
                          boxShadow: [
                            BoxShadow(
                              color: Colors.black.withOpacity(0.3),
                              spreadRadius: 5,
                              blurRadius: 7,
                              offset: const Offset(-3, 0),
                            ),
                          ],
                        ),
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.stretch,
                          children: [
                            // Header
                            Container(
                              padding: const EdgeInsets.all(16),
                              decoration: BoxDecoration(
                                color: connectionColor.withOpacity(0.1),
                                borderRadius: const BorderRadius.only(
                                  topLeft: Radius.circular(12),
                                ),
                              ),
                              child: Row(
                                mainAxisAlignment:
                                    MainAxisAlignment.spaceBetween,
                                children: [
                                  Text(
                                    'Connect to Robot',
                                    style: TextStyle(
                                      fontSize: height * 0.3,
                                      color: Colors.white,
                                      fontWeight: FontWeight.bold,
                                    ),
                                  ),
                                  IconButton(
                                    icon: const Icon(Icons.close),
                                    color: Colors.white,
                                    onPressed: () => Navigator.pop(context),
                                  ),
                                ],
                              ),
                            ),

                            Expanded(
                              child: SingleChildScrollView(
                                padding: const EdgeInsets.all(16),
                                child: Column(
                                  crossAxisAlignment:
                                      CrossAxisAlignment.stretch,
                                  children: [
                                    _buildConnectionForm(
                                      context,
                                      ipController,
                                      portController,
                                      connectionColor,
                                    ),
                                    const SizedBox(height: 24),
                                    _buildRecentConnectionsList(
                                      context,
                                      connectionColor,
                                    ),
                                  ],
                                ),
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            ),
          );
        },
      ),
    );
  }

  Widget _buildConnectionForm(
    BuildContext context,
    TextEditingController ipController,
    TextEditingController portController,
    Color connectionColor,
  ) {
    return LayoutBuilder(
      builder: (context, constraints) {
        return SingleChildScrollView(
          child: Padding(
            padding: EdgeInsets.only(
              bottom: MediaQuery.of(context).viewInsets.bottom,
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Text(
                  'New Robot',
                  style: TextStyle(
                    fontSize: height * 0.25,
                    color: Colors.white70,
                    fontWeight: FontWeight.bold,
                  ),
                ),
                const SizedBox(height: 16),
                TextField(
                  controller: ipController,
                  style:
                      TextStyle(color: Colors.white, fontSize: height * 0.22),
                  decoration: InputDecoration(
                    labelText: 'IP Address',
                    labelStyle: const TextStyle(color: Colors.white70),
                    hintText: '192.168.1.100',
                    hintStyle: TextStyle(color: Colors.white54),
                    filled: true,
                    fillColor: Colors.white.withOpacity(0.1),
                    border: OutlineInputBorder(
                      borderRadius: BorderRadius.circular(8),
                      borderSide: BorderSide.none,
                    ),
                  ),
                ),
                const SizedBox(height: 16),
                TextField(
                  controller: portController,
                  style:
                      TextStyle(color: Colors.white, fontSize: height * 0.22),
                  decoration: InputDecoration(
                    labelText: 'Port',
                    labelStyle: const TextStyle(color: Colors.white70),
                    hintText: '9090',
                    hintStyle: TextStyle(color: Colors.white54),
                    filled: true,
                    fillColor: Colors.white.withOpacity(0.1),
                    border: OutlineInputBorder(
                      borderRadius: BorderRadius.circular(8),
                      borderSide: BorderSide.none,
                    ),
                  ),
                ),
                const SizedBox(height: 24),
                ElevatedButton(
                  onPressed: () async {
                    try {
                      final connection = context.read<ConnectionProvider>();
                      await connection.connect(
                        ipController.text,
                        portController.text,
                      );
                      Navigator.pop(context);
                    } catch (e) {
                      final message = e.toString().contains('not available')
                          ? 'Robot not available at ${ipController.text}'
                          : 'Connection failed: ${e.toString().split(':').last}';

                      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                        content: Text(message),
                        backgroundColor: Colors.red,
                      ));
                    }
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: connectionColor,
                    padding: EdgeInsets.symmetric(vertical: height * 0.2),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(8),
                    ),
                  ),
                  child: Text(
                    'Connect',
                    style: TextStyle(
                      fontSize: height * 0.25,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }

  Widget _buildRecentConnectionsList(
      BuildContext context, Color connectionColor) {
    return Consumer<ConnectionProvider>(
      builder: (context, connection, child) {
        return Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Text(
              'Recent Robots',
              style: TextStyle(
                fontSize: height * 0.25,
                color: Colors.white70,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 12),
            ...connection.recentConnections.map((conn) => GestureDetector(
                  onTap: () async {
                    try {
                      await connection.connect(conn['ip']!, conn['port']!);
                      Navigator.pop(context);
                    } catch (e) {
                      final message = e.toString().contains('not available')
                          ? 'Robot not available at ${conn['ip']}'
                          : 'Connection failed: ${e.toString().split(':').last}';

                      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                        content: Text(message),
                        backgroundColor: Colors.red,
                      ));
                    }
                  },
                  child: ListTile(
                    contentPadding: EdgeInsets.zero,
                    leading: Icon(Icons.history, color: Colors.white54),
                    title: Text(
                      '${conn['ip']}:${conn['port']}',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: height * 0.22,
                      ),
                    ),
                    trailing: IconButton(
                      icon: Icon(Icons.connect_without_contact,
                          color: connectionColor),
                      onPressed: () async {
                        try {
                          await connection.connect(conn['ip']!, conn['port']!);
                          Navigator.pop(context);
                        } catch (e) {
                          final message = e.toString().contains('not available')
                              ? 'Robot not available at ${conn['ip']}'
                              : 'Connection failed: ${e.toString().split(':').last}';

                          ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                            content: Text(message),
                            backgroundColor: Colors.red,
                          ));
                        }
                      },
                    ),
                  ),
                )),
          ],
        );
      },
    );
  }

  Widget _buildStatusItem(IconData icon, String text) {
    return Padding(
      padding: EdgeInsets.symmetric(horizontal: height * 0.16),
      child: Row(
        children: [
          Icon(
            icon,
            color: Colors.white,
            size: height * 0.32,
          ),
          SizedBox(width: height * 0.08),
          Text(
            text,
            style: TextStyle(
              color: Colors.white,
              fontSize: height * 0.28,
            ),
          ),
        ],
      ),
    );
  }

  // Helper function to determine text color based on background color
  Color _getTextColor(Color backgroundColor) {
    // Calculate relative luminance
    double luminance = backgroundColor.computeLuminance();
    return luminance > 0.5 ? Colors.black : Colors.white;
  }
}
