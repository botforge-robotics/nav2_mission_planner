import 'package:flutter/material.dart';

class MapSaveDialog extends StatefulWidget {
  final Size screenSize;
  final Color modeColor;

  const MapSaveDialog({
    super.key,
    required this.screenSize,
    required this.modeColor,
  });

  @override
  State<MapSaveDialog> createState() => _MapSaveDialogState();
}

class _MapSaveDialogState extends State<MapSaveDialog> {
  final _formKey = GlobalKey<FormState>();
  final _controller = TextEditingController();
  bool _shouldStopMapping = true; // Default to stopping mapping

  bool _isValidMapName(String value) {
    if (value.isEmpty) return false;
    if (value.length < 4) return false;
    return RegExp(r'^[a-zA-Z0-9_\-]+$').hasMatch(value);
  }

  @override
  Widget build(BuildContext context) {
    return Dialog(
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(16),
      ),
      backgroundColor: Colors.grey.shade900,
      child: Container(
        width:
            widget.screenSize.width * 0.5, // Set width to 50% of screen width
        child: SingleChildScrollView(
          child: Padding(
            padding: const EdgeInsets.all(20.0),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                // Header
                Row(
                  children: [
                    Icon(
                      Icons.save,
                      color: widget.modeColor,
                      size: 28,
                    ),
                    const SizedBox(width: 10),
                    Text(
                      'Save Current Map',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 22,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 20),

                // Form
                Form(
                  key: _formKey,
                  child: TextFormField(
                    controller: _controller,
                    style: const TextStyle(color: Colors.white),
                    decoration: InputDecoration(
                      labelText: 'Map Name',
                      labelStyle: TextStyle(color: Colors.grey.shade300),
                      hintText: 'Enter map name (min 4 characters)',
                      hintStyle: TextStyle(color: Colors.grey.shade600),
                      filled: true,
                      fillColor: Colors.transparent,
                      border: UnderlineInputBorder(
                        borderSide: BorderSide(color: widget.modeColor),
                      ),
                      focusedBorder: UnderlineInputBorder(
                        borderSide:
                            BorderSide(color: widget.modeColor, width: 2),
                      ),
                      errorStyle: const TextStyle(
                          color: Colors.redAccent, fontWeight: FontWeight.bold),
                    ),
                    validator: (value) {
                      if (value == null || value.isEmpty) {
                        return 'Please enter a map name';
                      }
                      if (value.length < 4) {
                        return 'Name must be at least 4 characters';
                      }
                      if (!_isValidMapName(value)) {
                        return 'Only letters, numbers, hyphens and underscores';
                      }
                      return null;
                    },
                  ),
                ),

                const SizedBox(height: 16),

                // Stop mapping checkbox
                CheckboxListTile(
                  value: _shouldStopMapping,
                  onChanged: (value) {
                    setState(() {
                      _shouldStopMapping = value ?? true;
                    });
                  },
                  title: const Text(
                    'Stop mapping after saving',
                    style: TextStyle(color: Colors.white, fontSize: 16),
                  ),
                  activeColor: widget.modeColor,
                  checkColor: Colors.black,
                  controlAffinity: ListTileControlAffinity.leading,
                  contentPadding: EdgeInsets.zero,
                ),

                const SizedBox(height: 20),

                // Buttons
                Row(
                  mainAxisAlignment: MainAxisAlignment.end,
                  children: [
                    TextButton(
                      onPressed: () => Navigator.pop(context),
                      style: TextButton.styleFrom(
                        foregroundColor: Colors.white,
                      ),
                      child: const Text(
                        'Cancel',
                        style: TextStyle(fontSize: 16),
                      ),
                    ),
                    const SizedBox(width: 12),
                    ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        backgroundColor: widget.modeColor,
                        foregroundColor: Colors.black,
                        padding: const EdgeInsets.symmetric(
                          horizontal: 20,
                          vertical: 12,
                        ),
                      ),
                      onPressed: () {
                        if (_formKey.currentState!.validate()) {
                          // Return both map name and whether to stop mapping
                          Navigator.pop(context, {
                            'mapName': _controller.text.trim(),
                            'stopMapping': _shouldStopMapping,
                          });
                        }
                      },
                      child: const Text(
                        'Save Map',
                        style: TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
