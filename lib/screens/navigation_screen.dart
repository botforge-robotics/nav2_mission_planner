import 'package:flutter/material.dart';

class NavigationScreen extends StatelessWidget {
  final Color modeColor;
  const NavigationScreen({super.key, required this.modeColor});

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Text(
        'Navigation Mode',
        style: TextStyle(color: modeColor),
      ),
    );
  }
}
