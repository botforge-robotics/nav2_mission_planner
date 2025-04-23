import 'package:flutter/material.dart';

class VelocityControl extends StatelessWidget {
  final double value;
  final VoidCallback onIncrement;
  final VoidCallback onDecrement;
  final ValueChanged<double?> onChanged;
  final Size screenSize;
  final Color modeColor;

  const VelocityControl({
    super.key,
    required this.value,
    required this.onIncrement,
    required this.onDecrement,
    required this.onChanged,
    required this.screenSize,
    required this.modeColor,
  });

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        ElevatedButton(
          onPressed: onDecrement,
          style: ElevatedButton.styleFrom(
            backgroundColor: modeColor.withOpacity(0.2),
            foregroundColor: modeColor,
            shape: const CircleBorder(),
            padding: EdgeInsets.all(screenSize.height * 0.008),
            minimumSize: Size.zero,
            tapTargetSize: MaterialTapTargetSize.shrinkWrap,
            elevation: 0,
          ),
          child: Icon(
            Icons.remove,
            color: modeColor,
            size: screenSize.height * 0.05,
          ),
        ),
        SizedBox(width: screenSize.width * 0.01),
        Expanded(
          child: TextFormField(
            textAlign: TextAlign.center,
            controller: TextEditingController(
              text: value.toStringAsFixed(2),
            ),
            style: TextStyle(fontSize: screenSize.height * 0.025),
            keyboardType: const TextInputType.numberWithOptions(decimal: true),
            decoration: InputDecoration(
              border: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: BorderSide(color: modeColor),
              ),
              focusedBorder: OutlineInputBorder(
                borderRadius: BorderRadius.circular(8),
                borderSide: BorderSide(color: modeColor, width: 2),
              ),
              contentPadding: EdgeInsets.symmetric(
                vertical: screenSize.height * 0.015,
              ),
            ),
            onChanged: (value) {
              final parsed = double.tryParse(value);
              if (parsed != null) onChanged(parsed);
            },
          ),
        ),
        SizedBox(width: screenSize.width * 0.01),
        ElevatedButton(
          onPressed: onIncrement,
          style: ElevatedButton.styleFrom(
            backgroundColor: modeColor.withOpacity(0.2),
            foregroundColor: modeColor,
            shape: const CircleBorder(),
            padding: EdgeInsets.all(screenSize.height * 0.008),
            minimumSize: Size.zero,
            tapTargetSize: MaterialTapTargetSize.shrinkWrap,
            elevation: 0,
          ),
          child:
              Icon(Icons.add, color: modeColor, size: screenSize.height * 0.05),
        ),
      ],
    );
  }
}
