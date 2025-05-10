import 'package:flutter/material.dart';
import 'package:nav2_mission_planner/providers/nav_tool_provider.dart';
import 'package:provider/provider.dart';

class NavToolbar extends StatelessWidget {
  final Color modeColor;

  const NavToolbar({
    super.key,
    required this.modeColor,
  });

  @override
  Widget build(BuildContext context) {
    final provider = Provider.of<NavToolProvider>(context);

    return Container(
      width: 80,
      decoration: BoxDecoration(
        color: Colors.black.withOpacity(0.5),
        borderRadius: BorderRadius.circular(12),
      ),
      padding: const EdgeInsets.symmetric(vertical: 5),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          _buildToolButton(
            context: context,
            icon: Icons.navigation,
            label: 'Pose Estimate',
            tool: 'localization',
            isActive: provider.selectedTool == 'localization',
          ),
          _buildDivider(),
          _buildToolButton(
            context: context,
            icon: Icons.flag,
            label: 'Send Goal',
            tool: 'goal',
            isActive: provider.selectedTool == 'goal',
          ),
          _buildDivider(),
          _buildToolButton(
            context: context,
            icon: Icons.place,
            label: 'Waypoints',
            tool: 'waypoint',
            isActive: provider.selectedTool == 'waypoint',
          ),
          _buildDivider(),
          _buildToolButton(
            context: context,
            icon: Icons.timeline,
            label: 'Path',
            tool: 'path',
            isActive: provider.selectedTool == 'path',
          ),
        ],
      ),
    );
  }

  Widget _buildToolButton({
    required BuildContext context,
    required IconData icon,
    required String label,
    required String tool,
    required bool isActive,
  }) {
    return Tooltip(
      message: label,
      preferBelow: false,
      verticalOffset: 20,
      child: InkWell(
        onTap: () {
          final provider = Provider.of<NavToolProvider>(context, listen: false);
          if (tool == 'localization') {
            provider.setPoseEstimationEnabled(!isActive);
          } else {
            provider.setSelectedTool(isActive ? '' : tool);
          }
        },
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          width: 80,
          height: 60,
          margin: const EdgeInsets.symmetric(vertical: 4),
          decoration: BoxDecoration(
            color: isActive ? modeColor.withOpacity(0.05) : Colors.transparent,
            borderRadius: BorderRadius.circular(10),
            border: Border.all(
              color: isActive ? modeColor : Colors.transparent,
              width: 1.5,
            ),
            boxShadow: isActive
                ? [
                    BoxShadow(
                      color: modeColor.withOpacity(0.3),
                      blurRadius: 5,
                      spreadRadius: 1,
                    )
                  ]
                : null,
          ),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(
                icon,
                color: isActive ? modeColor : Colors.white70,
                size: 25,
              ),
              const SizedBox(height: 4),
              SizedBox(
                height: 24,
                child: FittedBox(
                  fit: BoxFit.scaleDown,
                  child: Text(
                    label,
                    maxLines: 2,
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      color: isActive ? modeColor : Colors.white70,
                      fontSize: 9,
                      fontWeight: isActive ? FontWeight.bold : FontWeight.w500,
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildDivider() {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 5),
      child: Container(
        height: 1,
        width: 30,
        color: Colors.grey.withOpacity(0.3),
      ),
    );
  }
}
