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

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => _loadMaps());
  }

  Future<void> _loadMaps() async {
    final maps = await MapListService().getMapList(context);
    setState(() {
      _mapList = maps;
      _selectedMap = maps.isNotEmpty ? maps.first : null;
      _loadingMaps = false;
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

  @override
  Widget build(BuildContext context) {
    final settings = Provider.of<SettingsProvider>(context);

    if (_isNavigationActive && _mapWidget != null) {
      // Active navigation view
      return Stack(
        children: [
          // Map Background
          Positioned.fill(child: Container(color: Colors.black)),

          // Map Display
          Positioned.fill(child: _mapWidget!),

          // Joystick Control
          Positioned(
            bottom: 60,
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

          // Stop Button
          Positioned(
            top: 16,
            right: 16,
            child: FloatingActionButton(
              mini: true,
              backgroundColor: Colors.red,
              onPressed: _stopNavigation,
              child: const Icon(Icons.stop, color: Colors.white),
            ),
          ),

          // Camera view
          if (settings.cameraEnabled && settings.cameraImageTopic.isNotEmpty)
            Positioned(
              bottom: 20,
              left: 20,
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
        ],
      );
    }

    // Map selection screen (unchanged)
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Container(
            padding: const EdgeInsets.all(20),
            decoration: BoxDecoration(
              color: widget.modeColor.withOpacity(0.2),
              shape: BoxShape.circle,
            ),
            child: Icon(
              FontAwesomeIcons.route,
              size: 80,
              color: widget.modeColor,
            ),
          ),
          const SizedBox(height: 10),
          Text(
            'Navigation Mode',
            style: TextStyle(
              fontSize: 28,
              fontWeight: FontWeight.bold,
              color: widget.modeColor,
              letterSpacing: 1.2,
            ),
          ),
          const SizedBox(height: 10),
          Container(
            width: 300,
            decoration: BoxDecoration(
              border: Border(
                bottom: BorderSide(color: widget.modeColor, width: 1.5),
              ),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Select Map',
                  style: TextStyle(
                    color: widget.modeColor,
                    fontSize: 16,
                    fontWeight: FontWeight.w500,
                    letterSpacing: 0.8,
                  ),
                ),
                const SizedBox(height: 4),
                _loadingMaps
                    ? Padding(
                        padding: const EdgeInsets.symmetric(vertical: 8),
                        child: Center(
                          child: CircularProgressIndicator(
                            color: widget.modeColor,
                            strokeWidth: 2,
                          ),
                        ),
                      )
                    : DropdownButton<String>(
                        value: _selectedMap,
                        dropdownColor: AppTheme.toolbarColor,
                        icon: Icon(Icons.arrow_drop_down,
                            color: widget.modeColor),
                        isExpanded: true,
                        style: TextStyle(
                          color: widget.modeColor,
                          fontSize: 16,
                          fontWeight: FontWeight.w500,
                        ),
                        itemHeight: 48,
                        menuMaxHeight: 150,
                        underline: const SizedBox(),
                        items: _mapList
                            .map((map) => DropdownMenuItem<String>(
                                  value: map,
                                  child: Container(
                                    height: 48,
                                    alignment: Alignment.centerLeft,
                                    child: Text(
                                      map,
                                      style: TextStyle(
                                        color: widget.modeColor,
                                        height: 1.2,
                                      ),
                                    ),
                                  ),
                                ))
                            .toList(),
                        onChanged: (value) =>
                            setState(() => _selectedMap = value),
                      ),
              ],
            ),
          ),
          const SizedBox(height: 20),
          ElevatedButton(
            onPressed: _mapList.isEmpty || _isNavigationActive
                ? null
                : _startNavigation,
            style: ElevatedButton.styleFrom(
              backgroundColor: _isNavigationActive
                  ? widget.modeColor.withOpacity(0.4)
                  : widget.modeColor.withOpacity(0.2),
              padding: const EdgeInsets.symmetric(horizontal: 25, vertical: 15),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(20),
                side: BorderSide(
                    color: _mapList.isEmpty
                        ? widget.modeColor.withOpacity(0.5)
                        : widget.modeColor,
                    width: 2),
              ),
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Icon(
                  _isNavigationActive ? Icons.stop : Icons.play_arrow,
                  color: _mapList.isEmpty
                      ? widget.modeColor.withOpacity(0.5)
                      : widget.modeColor,
                  size: 28,
                ),
                const SizedBox(width: 10),
                Text(
                  _isNavigationActive ? 'Stop Navigation' : 'Start Navigation',
                  style: TextStyle(
                    fontSize: 20,
                    color: _mapList.isEmpty
                        ? widget.modeColor.withOpacity(0.5)
                        : widget.modeColor,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
