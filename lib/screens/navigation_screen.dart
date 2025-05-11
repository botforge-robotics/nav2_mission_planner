import 'package:flutter/material.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
import 'package:provider/provider.dart';
import 'package:nav2_mission_planner/services/get_map_list_service.dart';
import 'package:nav2_mission_planner/theme/app_theme.dart';
import 'package:nav2_mission_planner/services/launch_service.dart';
import 'package:nav2_mission_planner/widgets/occupancy_grid_viewer.dart';
import 'package:nav2_mission_planner/widgets/joystick_thumb_widget.dart';
import 'package:nav2_mission_planner/widgets/image_viwer.dart';
import 'package:nav2_mission_planner/providers/settings_provider.dart';
import 'package:nav2_mission_planner/services/delete_map_service.dart';
import 'package:nav2_mission_planner/widgets/navigation_toolbar.dart';
import 'package:nav2_mission_planner/widgets/visibility_toolbar.dart';
import 'package:nav2_mission_planner/services/pose_estimation_service.dart';
import 'package:nav2_mission_planner/providers/nav_tool_provider.dart';

class NavigationScreen extends StatefulWidget {
  final Color modeColor;
  const NavigationScreen({super.key, required this.modeColor});

  @override
  State<NavigationScreen> createState() => _NavigationScreenState();
}

class _NavigationScreenState extends State<NavigationScreen> {
  List<String> _mapList = [];
  String? _selectedMap;
  bool _loadingMaps = true;
  bool _isNavigationActive = false;

  // Map display variables
  double _scale = 1.0;
  double _previousScale = 1.0;
  Offset _offset = Offset.zero;
  Offset _previousOffset = Offset.zero;
  Widget? _mapWidget;

  // track which maps are currently being deleted
  final Set<String> _deletingMaps = {};

  // Inside _NavigationScreenState class
  String _selectedTool = '';

  // Add this field to the state class
  late NavToolProvider _navToolProvider;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      PoseEstimationService.initializePublisher(context);
    });
    _navToolProvider = NavToolProvider();
    WidgetsBinding.instance.addPostFrameCallback((_) => _loadMaps());
  }

  @override
  void dispose() {
    _navToolProvider.dispose();
    super.dispose();
  }

  Future<void> _loadMaps() async {
    final maps = await MapListService().getMapList(context);
    setState(() {
      _mapList = maps;
      _selectedMap = maps.isNotEmpty ? maps.first : null;
      _loadingMaps = false;
    });
  }

  /// Delete remote+local map, then remove from the list on success
  Future<void> _deleteMap(String mapName) async {
    setState(() {
      _deletingMaps.add(mapName);
    });

    final success = await DeleteMapService().deleteMap(context, mapName);

    setState(() {
      _deletingMaps.remove(mapName);

      if (success) {
        _mapList.remove(mapName);

        if (_selectedMap == mapName) {
          _selectedMap = _mapList.isNotEmpty ? _mapList.first : null;
        }
      }
    });
  }

  Future<void> _startNavigation() async {
    if (_selectedMap == null) return;

    final launchManager = Provider.of<LaunchManager>(context, listen: false);

    try {
      // Show loading indicator
      showDialog(
        context: context,
        barrierDismissible: false,
        builder: (context) => AlertDialog(
          backgroundColor: Colors.transparent,
          elevation: 0,
          content: Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                CircularProgressIndicator(color: widget.modeColor),
                const SizedBox(height: 16),
                Text(
                  'Starting Navigation...',
                  style: TextStyle(color: Colors.white, fontSize: 16),
                ),
              ],
            ),
          ),
        ),
      );

      final success =
          await launchManager.startNavigation(context, _selectedMap!);
      await Future.delayed(const Duration(seconds: 3));
      // Dismiss loading dialog
      if (mounted) Navigator.pop(context);

      if (success) {
        setState(() {
          _isNavigationActive = true;

          // Create map widget
          _mapWidget = GestureDetector(
            onScaleStart: (details) {
              _previousScale = _scale;
              _previousOffset = _offset;
            },
            onScaleUpdate: (details) {
              setState(() {
                _scale = (_previousScale * details.scale).clamp(0.5, 5.0);
                if (details.pointerCount == 1) {
                  final delta = details.focalPoint - details.localFocalPoint;
                  _offset = delta;
                }
              });
            },
            onScaleEnd: (_) {
              _previousScale = _scale;
            },
            child: Transform.translate(
              offset: _offset,
              child: OccupancyGridViewer(
                topic: '/map',
                enabled: true,
                scale: _scale,
                appModeColor: widget.modeColor,
                onScaleChanged: (newScale) {
                  setState(() {
                    _scale = newScale;
                  });
                },
              ),
            ),
          );
        });

        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Navigation started with map: $_selectedMap'),
            backgroundColor: Colors.green,
          ),
        );
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Error starting navigation: ${e.toString()}'),
          backgroundColor: Colors.red,
        ),
      );
    }
  }

  Future<void> _stopNavigation() async {
    final launchManager = Provider.of<LaunchManager>(context, listen: false);

    try {
      // Show loading indicator
      showDialog(
        context: context,
        barrierDismissible: false,
        builder: (context) => AlertDialog(
          backgroundColor: Colors.transparent,
          elevation: 0,
          content: Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                CircularProgressIndicator(color: Colors.red),
                const SizedBox(height: 16),
                Text(
                  'Stopping Navigation...',
                  style: TextStyle(color: Colors.white, fontSize: 16),
                ),
              ],
            ),
          ),
        ),
      );

      for (final entry in launchManager.activeLaunches.entries) {
        await launchManager.stopLaunch(context, entry.key);
      }

      // Dismiss loading dialog
      if (mounted) Navigator.pop(context);

      setState(() {
        _isNavigationActive = false;
        _mapWidget = null;
      });

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: const Text('Navigation stopped'),
          backgroundColor: Colors.orange,
        ),
      );
    } catch (e) {
      // Dismiss loading dialog if still showing
      if (mounted) Navigator.pop(context);

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Error stopping navigation: ${e.toString()}'),
          backgroundColor: Colors.red,
        ),
      );
    }
  }

  // Handle tool selection
  void _handleToolSelected(String tool) {
    setState(() {
      _selectedTool = tool;
    });
    // Add your tool-specific logic here
    print('Selected tool: $tool');
  }

  // Add new confirmation method
  void _confirmStopNavigation() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Row(
          children: [
            Icon(Icons.warning_amber, color: widget.modeColor),
            const SizedBox(width: 12),
            const Text('Confirm Stop'),
          ],
        ),
        content: const Text('Are you sure you want to stop navigation?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel'),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(
              backgroundColor: widget.modeColor,
            ),
            onPressed: () {
              Navigator.pop(context);
              _stopNavigation();
            },
            child: const Text('Stop Navigation',
                style: TextStyle(color: Colors.white)),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final settings = Provider.of<SettingsProvider>(context);

    if (_isNavigationActive && _mapWidget != null) {
      // Active navigation view
      return ChangeNotifierProvider.value(
        value: _navToolProvider, // Use value provider with existing controller
        child: Stack(
          children: [
            // Map Background
            Positioned.fill(child: Container(color: Colors.black)),

            // Map Display
            Positioned.fill(child: _mapWidget!),

            // Add Navigation Toolbar
            Align(
              alignment: Alignment.centerLeft,
              child: Container(
                margin: const EdgeInsets.only(left: 5),
                child: NavToolbar(modeColor: widget.modeColor),
              ),
            ),

            // Joystick Control
            Visibility(
              visible: settings.joystickVisible,
              child: Positioned(
                bottom: 30,
                right: 40,
                child: Container(
                  width: 150,
                  height: 150,
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.3),
                    shape: BoxShape.circle,
                  ),
                  child: JoystickThumbWidget(modeColor: widget.modeColor),
                ),
              ),
            ),

            // Stop Button
            Positioned(
              top: 16,
              right: 65,
              child: FloatingActionButton(
                mini: true,
                backgroundColor: Colors.red,
                onPressed: _confirmStopNavigation,
                child: const Icon(Icons.stop, color: Colors.white),
              ),
            ),

            // Camera view
            if (settings.cameraEnabled && settings.cameraVisible)
              Positioned(
                top: 10,
                left: 100,
                child: SizedBox(
                  width: MediaQuery.of(context).size.width * 0.25,
                  height: MediaQuery.of(context).size.width * 0.25 * (9 / 16),
                  child: Container(
                    decoration: BoxDecoration(
                      color: Colors.black,
                      borderRadius: BorderRadius.circular(8),
                      border: Border.all(color: widget.modeColor, width: 2),
                    ),
                    child: ClipRRect(
                      borderRadius: BorderRadius.circular(6),
                      child: ImageViewer(
                        topic: settings.cameraImageTopic,
                        enabled: settings.cameraEnabled,
                        hideTopic: true,
                      ),
                    ),
                  ),
                ),
              ),

            // Visibility Toolbar
            Positioned(
              right: 0,
              top: 80,
              child: VisibilityToolbar(modeColor: widget.modeColor),
            ),
          ],
        ),
      );
    }

    // Map selection screen
    return Container(
      color: Colors.black87, // 60% - Primary background
      child: Row(
        children: [
          // Left side - Map List
          Container(
            width: 300,
            decoration: BoxDecoration(
              color: Colors.grey.shade900, // 30% - Secondary color
              border: Border(
                  right: BorderSide(color: Colors.grey.shade800, width: 1)),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                // Header
                Container(
                  padding: const EdgeInsets.all(16),
                  decoration: BoxDecoration(
                    color: Colors.grey.shade800, // 30% - Secondary color
                    border: Border(
                        bottom:
                            BorderSide(color: Colors.grey.shade700, width: 1)),
                  ),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      const Text(
                        'Available Maps',
                        style: TextStyle(
                          color: Colors.white,
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      IconButton(
                        icon: Icon(Icons.refresh,
                            color: widget.modeColor), // 10% - Accent for action
                        onPressed: () {
                          setState(() => _loadingMaps = true);
                          _loadMaps();
                        },
                      ),
                    ],
                  ),
                ),

                // Loading indicator or list
                Expanded(
                  child: _loadingMaps
                      ? Center(
                          child: CircularProgressIndicator(
                              color: widget.modeColor), // 10% - Accent
                        )
                      : _mapList.isEmpty
                          ? Center(
                              child: Text(
                                'No maps available',
                                style: TextStyle(color: Colors.grey.shade500),
                              ),
                            )
                          : ListView.builder(
                              itemCount: _mapList.length,
                              itemBuilder: (context, index) {
                                final map = _mapList[index];
                                final isSelected = map == _selectedMap;

                                return Dismissible(
                                  key: Key(map),
                                  direction: DismissDirection.endToStart,
                                  confirmDismiss: (_) async {
                                    return await showDialog<bool>(
                                          context: context,
                                          builder: (context) => AlertDialog(
                                            title: Text('Delete Map'),
                                            content: Text(
                                                'Are you sure you want to delete "$map"?'),
                                            actions: [
                                              TextButton(
                                                style: TextButton.styleFrom(
                                                  foregroundColor: Colors.white,
                                                ),
                                                onPressed: () => Navigator.pop(
                                                    context, false),
                                                child: Text('Cancel'),
                                              ),
                                              TextButton(
                                                style: TextButton.styleFrom(
                                                  foregroundColor: Colors.red,
                                                ),
                                                onPressed: () => Navigator.pop(
                                                    context, true),
                                                child: Text('Delete'),
                                              ),
                                            ],
                                          ),
                                        ) ??
                                        false;
                                  },
                                  onDismissed: (_) => _deleteMap(map),
                                  background: Container(
                                    color: Colors.transparent,
                                  ),
                                  secondaryBackground: Container(
                                    alignment: Alignment.centerRight,
                                    padding: EdgeInsets.only(right: 20),
                                    color: Colors.red.shade800,
                                    child: Icon(
                                      Icons.delete_forever,
                                      color: Colors.white,
                                    ),
                                  ),
                                  child: Container(
                                    margin:
                                        const EdgeInsets.symmetric(vertical: 4),
                                    decoration: BoxDecoration(
                                      color: isSelected
                                          ? Colors.grey.shade800
                                          : Colors.grey.shade900,
                                      borderRadius: BorderRadius.circular(8),
                                      boxShadow: isSelected
                                          ? [
                                              BoxShadow(
                                                color: Colors.black
                                                    .withOpacity(0.3),
                                                blurRadius: 3,
                                                offset: Offset(0, 2),
                                              )
                                            ]
                                          : null,
                                      border: Border.all(
                                        color: isSelected
                                            ? widget.modeColor
                                            : Colors.transparent,
                                        width: isSelected ? 1 : 0,
                                      ),
                                    ),
                                    child: Stack(
                                      children: [
                                        if (isSelected)
                                          Positioned(
                                            left: 0,
                                            top: 0,
                                            bottom: 0,
                                            width: 4,
                                            child: Container(
                                              decoration: BoxDecoration(
                                                color: widget.modeColor,
                                                borderRadius: BorderRadius.only(
                                                  topLeft: Radius.circular(8),
                                                  bottomLeft:
                                                      Radius.circular(8),
                                                ),
                                              ),
                                            ),
                                          ),
                                        ListTile(
                                          contentPadding: EdgeInsets.only(
                                            left: isSelected ? 16 : 16,
                                            right: 16,
                                          ),
                                          leading: Icon(
                                            FontAwesomeIcons.map,
                                            color: isSelected
                                                ? widget.modeColor
                                                : Colors.grey.shade600,
                                            size: 20,
                                          ),
                                          title: Text(
                                            map,
                                            style: TextStyle(
                                              color: Colors.white,
                                              fontWeight: isSelected
                                                  ? FontWeight.bold
                                                  : FontWeight.normal,
                                            ),
                                          ),
                                          onTap: () => setState(
                                              () => _selectedMap = map),
                                        ),
                                      ],
                                    ),
                                  ),
                                );
                              },
                            ),
                ),
              ],
            ),
          ),

          // Right side - Controls
          Expanded(
            child: Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  // Icon and title
                  Container(
                    padding: const EdgeInsets.all(25),
                    decoration: BoxDecoration(
                      color: Colors.grey.shade800, // 30% - Secondary color
                      shape: BoxShape.circle,
                    ),
                    child: Icon(
                      FontAwesomeIcons.route,
                      size: 80,
                      color: Colors.white,
                    ),
                  ),
                  const SizedBox(height: 20),
                  const Text(
                    'Navigation Mode',
                    style: TextStyle(
                      fontSize: 28,
                      fontWeight: FontWeight.bold,
                      color: Colors.white,
                      letterSpacing: 1.2,
                    ),
                  ),
                  const SizedBox(height: 15),

                  // Selected map display
                  Text(
                    _selectedMap != null
                        ? 'Selected: $_selectedMap'
                        : 'No map selected',
                    style: TextStyle(
                      fontSize: 16,
                      color: Colors.grey.shade500,
                    ),
                  ),
                  const SizedBox(height: 15),

                  // Start button - Primary action
                  ElevatedButton(
                    onPressed: _mapList.isEmpty || _selectedMap == null
                        ? null
                        : _startNavigation,
                    style: ElevatedButton.styleFrom(
                      backgroundColor: widget
                          .modeColor, // 10% - Primary accent for main action
                      foregroundColor: Colors.white,
                      padding: const EdgeInsets.symmetric(
                          horizontal: 30, vertical: 16),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(30),
                      ),
                      elevation: 4,
                    ),
                    child: Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        const Icon(
                          Icons.play_arrow,
                          color: Colors.white,
                          size: 30,
                        ),
                        const SizedBox(width: 12),
                        const Text(
                          'Start Navigation',
                          style: TextStyle(
                            fontSize: 20,
                            color: Colors.white,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}
