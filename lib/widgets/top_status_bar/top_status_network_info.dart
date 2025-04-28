import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../../providers/connection_provider.dart';

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
    final connection = Provider.of<ConnectionProvider>(context);

    return Container(
      height: height,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      margin: const EdgeInsets.only(right: 8),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(Icons.language, color: connectionStatusColor, size: 15),
          const SizedBox(width: 2),
          Text(
            connection.ip.isNotEmpty
                ? 'Robot IP: ${connection.ip}'
                : 'Not Connected',
            style: TextStyle(
              color: Colors.white,
              fontSize: 11,
              fontWeight: FontWeight.w500,
            ),
          ),
        ],
      ),
    );
  }
}
