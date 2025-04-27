import 'package:flutter/material.dart';

class TopStatusCenterTitle extends StatelessWidget {
  const TopStatusCenterTitle({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(
      child: Text(
        'Nav2 Mission Planner',
        style: TextStyle(
          color: Colors.white,
          fontSize: 14,
          fontWeight: FontWeight.bold,
        ),
      ),
    );
  }
}
