import 'package:flutter/material.dart';

class TopStatusNetworkInfo extends StatelessWidget {
  final double height;
  final Color connectionStatusColor;

  const TopStatusNetworkInfo({
    super.key,
    required this.height,
    required this.connectionStatusColor,
  });

  @override
  Widget build(BuildContext context) {
    // ...implement your network info widget here...
    return const SizedBox.shrink();
  }
}
