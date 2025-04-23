import 'package:flutter/material.dart';

class LaunchFileInput extends StatelessWidget {
  final String initialValue;
  final Function(String) onChanged;
  final Size screenSize;
  final Color modeColor;
  final String hintText;

  const LaunchFileInput({
    super.key,
    required this.initialValue,
    required this.onChanged,
    required this.screenSize,
    required this.modeColor,
    this.hintText = 'package_name/launch_file',
  });

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Format: \${package_name}/\${launchfile_name}',
          style: TextStyle(
            fontSize: screenSize.height * 0.024,
            color: Colors.grey.shade400,
          ),
        ),
        SizedBox(height: screenSize.height * 0.02),
        Row(
          children: [
            Expanded(
              child: TextFormField(
                initialValue: initialValue,
                style: TextStyle(fontSize: screenSize.height * 0.025),
                decoration: InputDecoration(
                  hintText: hintText,
                  hintStyle: TextStyle(color: Colors.grey.shade700),
                  border: OutlineInputBorder(
                    borderRadius: BorderRadius.circular(8),
                    borderSide: BorderSide(color: modeColor),
                  ),
                  focusedBorder: OutlineInputBorder(
                    borderRadius: BorderRadius.circular(8),
                    borderSide: BorderSide(color: modeColor, width: 2),
                  ),
                  contentPadding: EdgeInsets.symmetric(
                    horizontal: screenSize.width * 0.015,
                    vertical: screenSize.height * 0.015,
                  ),
                ),
                onChanged: onChanged,
              ),
            ),
          ],
        ),
        SizedBox(height: screenSize.height * 0.01),
        Text(
          'Command will be: ros2 launch [input].launch.py',
          style: TextStyle(
            fontSize: screenSize.height * 0.018,
            color: Colors.grey.shade400,
            fontStyle: FontStyle.italic,
          ),
        ),
      ],
    );
  }
}
