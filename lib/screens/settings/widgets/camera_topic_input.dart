import 'package:flutter/material.dart';

class CameraTopicInput extends StatelessWidget {
  final String initialValue;
  final Function(String) onChanged;
  final Size screenSize;
  final Color modeColor;
  final bool enabled;
  final ValueChanged<bool> onEnabledChanged;

  const CameraTopicInput({
    super.key,
    required this.initialValue,
    required this.onChanged,
    required this.screenSize,
    required this.modeColor,
    required this.enabled,
    required this.onEnabledChanged,
  });

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Expanded(
              child: TextFormField(
                initialValue: initialValue,
                style: TextStyle(fontSize: screenSize.height * 0.025),
                decoration: InputDecoration(
                  hintText:
                      'Enter camera topic (e.g. /camera/image/compressed)',
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
            SizedBox(width: 10),
            Column(
              children: [
                Text(
                  'Enable Camera',
                  style: TextStyle(
                    fontSize: screenSize.height * 0.018,
                    color: Colors.grey.shade400,
                  ),
                ),
                Switch(
                  value: enabled,
                  onChanged: onEnabledChanged,
                  activeColor: modeColor,
                ),
              ],
            ),
          ],
        ),
        SizedBox(height: screenSize.height * 0.01),
        Text(
          'Accepts sensor_msgs/CompressedImage messages',
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
