import 'package:flutter/material.dart';

class MissionScreen extends StatelessWidget {
  final Color modeColor;
  const MissionScreen({super.key, required this.modeColor});

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Text(
        'Mission Mode',
        style: TextStyle(color: modeColor),
      ),
    );
  }
}
