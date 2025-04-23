import 'package:flutter/material.dart';

class Mapping extends StatelessWidget {
  const Mapping({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      color: const Color.fromARGB(221, 255, 255, 255),
      child: const Center(
        child: Text(
          'Map View',
          style: TextStyle(color: Colors.white),
        ),
      ),
    );
  }
}
