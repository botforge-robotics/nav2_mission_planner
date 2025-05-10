import 'dart:async';

import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:provider/provider.dart';
import '../../providers/connection_provider.dart';
import '../../theme/app_theme.dart';
import '../../services/launch_service.dart';

class TopStatusConnectionButton extends StatefulWidget {
  final double height;
  final Color connectionStatusColor;

  const TopStatusConnectionButton({
    super.key,
    required this.height,
    required this.connectionStatusColor,
  });

  @override
  _TopStatusConnectionButtonState createState() =>
      _TopStatusConnectionButtonState();
}

class _TopStatusConnectionButtonState extends State<TopStatusConnectionButton> {
  StreamSubscription? _connectionSub;
  bool _disposed = false;

  @override
  void initState() {
    super.initState();
    final connectionProvider =
        Provider.of<ConnectionProvider>(context, listen: false);
    _connectionSub = connectionProvider.connectionStream.listen((state) {
      if (!mounted || _disposed) return;
      // Handle connection state changes here
    });
  }

  @override
  void dispose() {
    _disposed = true;
    _connectionSub?.cancel();
    super.dispose();
  }

  // Standardized SnackBar colors
  void _showInfoSnackBar(BuildContext context, String message) {
    if (!mounted || _disposed) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.orange,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ),
    );
  }

  void _showErrorSnackBar(BuildContext context, String message) {
    if (!mounted || _disposed) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.red,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ),
    );
  }

  void _showDisconnectDialog(BuildContext context) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Row(
          children: [
            Container(
              padding: const EdgeInsets.all(8),
              decoration: BoxDecoration(
                color: Colors.red.withOpacity(0.1),
                borderRadius: BorderRadius.circular(8),
              ),
              child: const Icon(Icons.link_off, color: Colors.red),
            ),
            const SizedBox(width: 12),
            const Text('Disconnect from Robot'),
          ],
        ),
        content: const Text(
          'Are you sure you want to disconnect from the current robot?',
          style: TextStyle(height: 1.5),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel'),
          ),
          ElevatedButton.icon(
            icon: const Icon(Icons.link_off),
            label: const Text('Disconnect'),
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.red,
              foregroundColor: Colors.white,
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(8),
              ),
            ),
            onPressed: () async {
              try {
                final launchManager =
                    Provider.of<LaunchManager>(context, listen: false);

                // Stop all active launches
                if (launchManager.activeLaunches.isNotEmpty) {
                  _showInfoSnackBar(context, 'Stopping active processes...');

                  // Make a copy of the active launches to avoid modification during iteration
                  final activeIds = launchManager.activeLaunches.keys.toList();

                  // Stop each launch
                  for (final id in activeIds) {
                    await launchManager.stopLaunch(context, id);
                  }
                }

                // Disconnect from ROS2
                await Provider.of<ConnectionProvider>(context, listen: false)
                    .disconnect();

                Navigator.pop(context); // Close dialog

                _showInfoSnackBar(context, 'Disconnected successfully');
              } catch (e) {
                _showErrorSnackBar(
                    context, 'Failed to disconnect: ${e.toString()}');
              }
            },
          ),
        ],
      ),
    );
  }

  void _showConnectionPanel(BuildContext context) {
    final ipController = TextEditingController(
      text: Provider.of<ConnectionProvider>(context, listen: false).ip,
    );
    final portController = TextEditingController(
      text: Provider.of<ConnectionProvider>(context, listen: false).port,
    );

    // Define state to track loading separately
    bool isLoadingRecentConnections = true;

    // Show dialog immediately
    final dialog = showGeneralDialog(
      context: context,
      barrierDismissible: true,
      barrierLabel: "ConnectionPanel",
      transitionDuration: const Duration(milliseconds: 300),
      pageBuilder: (_, __, ___) => Container(),
      transitionBuilder: (context, animation, secondaryAnimation, child) {
        return SlideTransition(
          position:
              Tween<Offset>(begin: const Offset(1, 0), end: const Offset(0, 0))
                  .animate(CurvedAnimation(
                      parent: animation, curve: Curves.easeOutQuint)),
          child: FadeTransition(
            opacity: animation,
            child: StatefulBuilder(
              builder: (context, setState) {
                // Load recent connections after dialog is visible
                if (isLoadingRecentConnections) {
                  Future.microtask(() async {
                    await Future.delayed(Duration(
                        milliseconds: 100)); // Small delay for animation
                    if (context.mounted) {
                      setState(() => isLoadingRecentConnections = false);
                    }
                  });
                }

                return Align(
                  alignment: Alignment.centerRight,
                  child: Material(
                    elevation: 16,
                    shadowColor: Colors.black26,
                    color: AppTheme.toolbarColor,
                    borderRadius: const BorderRadius.only(
                      topLeft: Radius.circular(24),
                      bottomLeft: Radius.circular(24),
                    ),
                    child: Container(
                      height: double.infinity,
                      width: MediaQuery.of(context).size.width * 0.4,
                      decoration: BoxDecoration(
                        borderRadius: const BorderRadius.only(
                          topLeft: Radius.circular(24),
                          bottomLeft: Radius.circular(24),
                        ),
                        gradient: LinearGradient(
                          begin: Alignment.topLeft,
                          end: Alignment.bottomRight,
                          colors: [
                            AppTheme.toolbarColor,
                            AppTheme.toolbarColor.withBlue(
                                (AppTheme.toolbarColor.blue * 1.05)
                                    .round()
                                    .clamp(0, 255)),
                          ],
                        ),
                      ),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Container(
                            padding: const EdgeInsets.symmetric(
                                horizontal: 24, vertical: 18),
                            decoration: BoxDecoration(
                              color: Colors.green.withOpacity(0.15),
                              borderRadius: const BorderRadius.only(
                                topLeft: Radius.circular(24),
                              ),
                            ),
                            child: Row(
                              mainAxisAlignment: MainAxisAlignment.spaceBetween,
                              children: [
                                Row(
                                  children: [
                                    Container(
                                      padding: const EdgeInsets.all(10),
                                      child: const Icon(
                                        FontAwesomeIcons.robot,
                                        color: Colors.white,
                                        size: 22,
                                      ),
                                    ),
                                    const SizedBox(width: 16),
                                    const Text(
                                      'Connect to Robot',
                                      style: TextStyle(
                                        fontSize: 21,
                                        fontWeight: FontWeight.bold,
                                        color: Colors.white,
                                      ),
                                    ),
                                  ],
                                ),
                                IconButton(
                                  icon: const Icon(Icons.close,
                                      color: Colors.white),
                                  onPressed: () => Navigator.pop(context),
                                  tooltip: 'Close',
                                  style: IconButton.styleFrom(
                                    backgroundColor: Colors.black26,
                                    foregroundColor: Colors.white,
                                  ),
                                ),
                              ],
                            ),
                          ),
                          Expanded(
                            child: ClipRRect(
                              borderRadius: const BorderRadius.only(
                                bottomLeft: Radius.circular(24),
                              ),
                              child: SingleChildScrollView(
                                padding: const EdgeInsets.symmetric(
                                    horizontal: 24, vertical: 20),
                                child: Column(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    // Connection form section
                                    Container(
                                      padding: const EdgeInsets.all(20),
                                      decoration: BoxDecoration(
                                        color: Colors.white.withOpacity(0.06),
                                        borderRadius: BorderRadius.circular(16),
                                        border: Border.all(
                                          color: Colors.white.withOpacity(0.12),
                                          width: 1,
                                        ),
                                        boxShadow: [
                                          BoxShadow(
                                            color:
                                                Colors.black.withOpacity(0.05),
                                            blurRadius: 15,
                                            offset: const Offset(0, 5),
                                          ),
                                        ],
                                      ),
                                      child: Column(
                                        crossAxisAlignment:
                                            CrossAxisAlignment.start,
                                        children: [
                                          const Text(
                                            'Connection Details',
                                            style: TextStyle(
                                              fontWeight: FontWeight.bold,
                                              color: Colors.white,
                                              fontSize: 18,
                                            ),
                                          ),
                                          const SizedBox(height: 20),
                                          TextField(
                                            controller: ipController,
                                            style: const TextStyle(
                                                color: Colors.white),
                                            decoration: InputDecoration(
                                              labelText: 'IP Address',
                                              labelStyle: const TextStyle(
                                                  color: Colors.white70),
                                              hintText: 'Enter IP address',
                                              hintStyle: const TextStyle(
                                                  color: Colors.white54),
                                              prefixIcon: const Icon(
                                                  Icons.computer,
                                                  color: Colors.white70),
                                              border: OutlineInputBorder(
                                                borderRadius:
                                                    BorderRadius.circular(12),
                                                borderSide: BorderSide.none,
                                              ),
                                              filled: true,
                                              fillColor: Colors.white
                                                  .withOpacity(0.08),
                                              contentPadding:
                                                  const EdgeInsets.symmetric(
                                                vertical: 16,
                                                horizontal: 16,
                                              ),
                                            ),
                                          ),
                                          const SizedBox(height: 16),
                                          TextField(
                                            controller: portController,
                                            style: const TextStyle(
                                                color: Colors.white),
                                            decoration: InputDecoration(
                                              labelText: 'Port',
                                              labelStyle: const TextStyle(
                                                  color: Colors.white70),
                                              hintText: 'Enter port number',
                                              hintStyle: const TextStyle(
                                                  color: Colors.white54),
                                              prefixIcon: const Icon(
                                                  Icons.settings_ethernet,
                                                  color: Colors.white70),
                                              border: OutlineInputBorder(
                                                borderRadius:
                                                    BorderRadius.circular(12),
                                                borderSide: BorderSide.none,
                                              ),
                                              filled: true,
                                              fillColor: Colors.white
                                                  .withOpacity(0.08),
                                              contentPadding:
                                                  const EdgeInsets.symmetric(
                                                vertical: 16,
                                                horizontal: 16,
                                              ),
                                            ),
                                          ),
                                          const SizedBox(height: 24),
                                          ElevatedButton.icon(
                                            style: ElevatedButton.styleFrom(
                                              minimumSize: const Size(
                                                  double.infinity, 56),
                                              shape: RoundedRectangleBorder(
                                                borderRadius:
                                                    BorderRadius.circular(12),
                                              ),
                                              backgroundColor: Colors.green,
                                              foregroundColor: Colors.white,
                                              elevation: 4,
                                              shadowColor:
                                                  Colors.green.withOpacity(0.4),
                                              padding: const EdgeInsets.all(12),
                                            ),
                                            onPressed: () async {
                                              Navigator.pop(context);
                                              if (!mounted) return;

                                              try {
                                                _showInfoSnackBar(context,
                                                    'Checking robot availability...');

                                                final success = await Provider
                                                        .of<ConnectionProvider>(
                                                            context,
                                                            listen: false)
                                                    .connect(ipController.text,
                                                        portController.text);

                                                if (!mounted) return;
                                                if (success) {
                                                  _showInfoSnackBar(context,
                                                      'Connected successfully!');
                                                }
                                              } catch (e) {
                                                if (!mounted) return;
                                                _showErrorSnackBar(context,
                                                    'Connection failed: ${e.toString()}');
                                              }
                                            },
                                            label: const Text(
                                              'Connect',
                                              style: TextStyle(
                                                fontSize: 16,
                                                fontWeight: FontWeight.bold,
                                              ),
                                            ),
                                          ),
                                        ],
                                      ),
                                    ),

                                    const SizedBox(height: 28),

                                    // Recent robots section
                                    Row(
                                      children: [
                                        Container(
                                          padding: const EdgeInsets.all(8),
                                          decoration: BoxDecoration(
                                            color:
                                                Colors.green.withOpacity(0.15),
                                            borderRadius:
                                                BorderRadius.circular(8),
                                          ),
                                          child: const Icon(
                                            Icons.history,
                                            color: Colors.white,
                                            size: 18,
                                          ),
                                        ),
                                        const SizedBox(width: 12),
                                        const Text(
                                          'Recent Robots',
                                          style: TextStyle(
                                            fontWeight: FontWeight.bold,
                                            color: Colors.white,
                                            fontSize: 18,
                                          ),
                                        ),
                                      ],
                                    ),
                                    const SizedBox(height: 16),
                                    Consumer<ConnectionProvider>(
                                      builder: (context, provider, child) {
                                        if (isLoadingRecentConnections) {
                                          return Container(
                                            padding: const EdgeInsets.all(24),
                                            alignment: Alignment.center,
                                            decoration: BoxDecoration(
                                              color: Colors.white
                                                  .withOpacity(0.05),
                                              borderRadius:
                                                  BorderRadius.circular(16),
                                              border: Border.all(
                                                  color: Colors.white
                                                      .withOpacity(0.1)),
                                            ),
                                            child: Column(
                                              children: [
                                                CircularProgressIndicator(
                                                  valueColor:
                                                      AlwaysStoppedAnimation<
                                                              Color>(
                                                          Colors.green
                                                              .withOpacity(
                                                                  0.7)),
                                                  strokeWidth: 3,
                                                ),
                                                const SizedBox(height: 16),
                                                const Text(
                                                  'Loading recent connections',
                                                  style: TextStyle(
                                                      color: Colors.white70),
                                                ),
                                              ],
                                            ),
                                          );
                                        }

                                        if (provider
                                            .recentConnections.isEmpty) {
                                          return Container(
                                            padding: const EdgeInsets.all(24),
                                            alignment: Alignment.center,
                                            decoration: BoxDecoration(
                                              color: Colors.white
                                                  .withOpacity(0.05),
                                              borderRadius:
                                                  BorderRadius.circular(16),
                                              border: Border.all(
                                                  color: Colors.white
                                                      .withOpacity(0.1)),
                                            ),
                                            child: Column(
                                              children: [
                                                Icon(
                                                  Icons.router_outlined,
                                                  size: 48,
                                                  color: Colors.white
                                                      .withOpacity(0.5),
                                                ),
                                                const SizedBox(height: 16),
                                                const Text(
                                                  'No recent connections',
                                                  style: TextStyle(
                                                    fontStyle: FontStyle.italic,
                                                    color: Colors.white70,
                                                  ),
                                                ),
                                              ],
                                            ),
                                          );
                                        }

                                        return ListView.builder(
                                          shrinkWrap: true,
                                          physics:
                                              const NeverScrollableScrollPhysics(),
                                          itemCount:
                                              provider.recentConnections.length,
                                          itemBuilder: (context, index) {
                                            final connection = provider
                                                .recentConnections[index];
                                            return Container(
                                              margin: const EdgeInsets.only(
                                                  bottom: 12),
                                              decoration: BoxDecoration(
                                                color: Colors.green
                                                    .withOpacity(0.1),
                                                borderRadius:
                                                    BorderRadius.circular(16),
                                                border: Border.all(
                                                    color: Colors.green
                                                        .withOpacity(0.2)),
                                              ),
                                              child: Material(
                                                color: Colors.transparent,
                                                child: InkWell(
                                                  borderRadius:
                                                      BorderRadius.circular(16),
                                                  onTap: () async {
                                                    Navigator.pop(context);
                                                    if (!mounted) return;

                                                    try {
                                                      _showInfoSnackBar(context,
                                                          'Connecting...');
                                                      await provider.connect(
                                                          connection['ip'] ??
                                                              '',
                                                          connection['port'] ??
                                                              '9090');

                                                      if (!mounted) return;
                                                      _showInfoSnackBar(context,
                                                          'Connected successfully!');
                                                    } catch (e) {
                                                      if (!mounted) return;
                                                      _showErrorSnackBar(
                                                          context,
                                                          'Connection failed: ${e.toString()}');
                                                    }
                                                  },
                                                  child: Padding(
                                                    padding:
                                                        const EdgeInsets.all(
                                                            16.0),
                                                    child: Row(
                                                      children: [
                                                        Container(
                                                          padding:
                                                              const EdgeInsets
                                                                  .all(12),
                                                          child: const Icon(
                                                            FontAwesomeIcons
                                                                .robot,
                                                            size: 24,
                                                            color: Colors.white,
                                                          ),
                                                        ),
                                                        const SizedBox(
                                                            width: 16),
                                                        Expanded(
                                                          child: Column(
                                                            crossAxisAlignment:
                                                                CrossAxisAlignment
                                                                    .start,
                                                            children: [
                                                              Text(
                                                                '${connection['ip']}',
                                                                style:
                                                                    const TextStyle(
                                                                  fontWeight:
                                                                      FontWeight
                                                                          .bold,
                                                                  fontSize: 16,
                                                                  color: Colors
                                                                      .white,
                                                                ),
                                                              ),
                                                              const SizedBox(
                                                                  height: 4),
                                                              Text(
                                                                'Port: ${connection['port']}',
                                                                style:
                                                                    const TextStyle(
                                                                  color: Colors
                                                                      .white70,
                                                                  fontSize: 13,
                                                                ),
                                                              ),
                                                            ],
                                                          ),
                                                        ),
                                                        Container(
                                                          decoration:
                                                              BoxDecoration(
                                                            color: Colors.green
                                                                .withOpacity(
                                                                    0.3),
                                                            shape:
                                                                BoxShape.circle,
                                                          ),
                                                          child: IconButton(
                                                            icon: const Icon(
                                                              Icons.link,
                                                              color:
                                                                  Colors.white,
                                                            ),
                                                            onPressed:
                                                                () async {
                                                              Navigator.pop(
                                                                  context);
                                                              if (!mounted) {
                                                                return;
                                                              }

                                                              try {
                                                                _showInfoSnackBar(
                                                                    context,
                                                                    'Connecting...');
                                                                await provider.connect(
                                                                    connection[
                                                                            'ip'] ??
                                                                        '',
                                                                    connection[
                                                                            'port'] ??
                                                                        '9090');
                                                                if (!mounted) {
                                                                  return;
                                                                }
                                                                _showInfoSnackBar(
                                                                    context,
                                                                    'Connected successfully!');
                                                              } catch (e) {
                                                                if (!mounted) {
                                                                  return;
                                                                }
                                                                _showErrorSnackBar(
                                                                    context,
                                                                    'Connection failed: ${e.toString()}');
                                                              }
                                                            },
                                                            tooltip: 'Connect',
                                                          ),
                                                        ),
                                                      ],
                                                    ),
                                                  ),
                                                ),
                                              ),
                                            );
                                          },
                                        );
                                      },
                                    ),
                                    // Add some bottom padding
                                    const SizedBox(height: 20),
                                  ],
                                ),
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                );
              },
            ),
          ),
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<ConnectionProvider>(
      builder: (context, connection, _) {
        return Align(
          alignment: Alignment.centerRight,
          child: Container(
            margin: const EdgeInsets.only(right: 10),
            child: AnimatedSwitcher(
              duration: const Duration(milliseconds: 300),
              transitionBuilder: (Widget child, Animation<double> animation) {
                return ScaleTransition(scale: animation, child: child);
              },
              child: IconButton(
                key: ValueKey<bool>(connection.isConnected),
                icon: Icon(
                  connection.isConnected ? Icons.link : Icons.link_off,
                  color: connection.isConnected ? Colors.green : Colors.red,
                  size: 28,
                ),
                onPressed: () {
                  if (connection.isConnected) {
                    _showDisconnectDialog(context);
                  } else {
                    _showConnectionPanel(context);
                  }
                },
                tooltip: connection.isConnected
                    ? 'Connected to ${connection.ip}:${connection.port}'
                    : 'Connection Settings',
                style: IconButton.styleFrom(
                  padding: const EdgeInsets.all(8),
                  backgroundColor: connection.isConnected
                      ? Colors.green.withOpacity(0.15)
                      : Colors.red.withOpacity(0.15),
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}
