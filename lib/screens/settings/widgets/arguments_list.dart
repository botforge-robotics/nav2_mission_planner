import 'package:flutter/material.dart';
import 'argument_dialogs.dart';

class ArgumentsList extends StatelessWidget {
  final List<Map<String, String>> arguments;
  final Function(int) onRemove;
  final Function(String, String) onAdd;
  final Function(int, String, String) onUpdate;
  final Size screenSize;
  final Color modeColor;

  const ArgumentsList({
    super.key,
    required this.arguments,
    required this.onRemove,
    required this.onAdd,
    required this.onUpdate,
    required this.screenSize,
    required this.modeColor,
  });

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        // Arguments list
        Container(
          constraints: BoxConstraints(
            maxHeight: screenSize.height * 0.3,
          ),
          child: arguments.isEmpty
              ? Center(
                  child: Text(
                    'No arguments added yet',
                    style: TextStyle(
                      fontSize: screenSize.height * 0.02,
                      color: Colors.grey,
                      fontStyle: FontStyle.italic,
                    ),
                  ),
                )
              : ListView.builder(
                  shrinkWrap: true,
                  itemCount: arguments.length,
                  itemBuilder: (context, index) {
                    final arg = arguments[index];
                    return Card(
                      color: modeColor.withOpacity(0.1),
                      margin: EdgeInsets.only(bottom: 8),
                      child: ListTile(
                        title: Text(
                          '${arg['name']} = ${arg['value']}',
                          style: TextStyle(
                            fontSize: screenSize.height * 0.022,
                          ),
                        ),
                        trailing: IconButton(
                          iconSize: screenSize.height * 0.05,
                          icon: Icon(Icons.delete, color: modeColor),
                          onPressed: () => onRemove(index),
                          style: ButtonStyle(
                            backgroundColor: MaterialStateProperty.all(
                                modeColor.withOpacity(0.2)),
                            foregroundColor:
                                MaterialStateProperty.all(modeColor),
                          ),
                        ),
                        onTap: () => showEditArgumentDialog(
                          context: context,
                          screenSize: screenSize,
                          modeColor: modeColor,
                          name: arg['name'] ?? '',
                          value: arg['value'] ?? '',
                          onUpdate: (name, value) =>
                              onUpdate(index, name, value),
                        ),
                      ),
                    );
                  },
                ),
        ),

        SizedBox(height: screenSize.height * 0.02),

        // Add argument button
        ElevatedButton.icon(
          onPressed: () => showAddArgumentDialog(
            context: context,
            screenSize: screenSize,
            modeColor: modeColor,
            onAdd: onAdd,
          ),
          icon: Icon(Icons.add, color: modeColor),
          label: Text(
            'Add Argument',
            style: TextStyle(
              color: modeColor,
              fontSize: screenSize.height * 0.023,
            ),
          ),
          style: ElevatedButton.styleFrom(
            backgroundColor: modeColor.withOpacity(0.2),
            shadowColor: modeColor.withOpacity(0.2),
            foregroundColor: modeColor,
            elevation: 0,
            padding: EdgeInsets.symmetric(
              horizontal: screenSize.width * 0.02,
              vertical: screenSize.height * 0.015,
            ),
          ),
        ),
      ],
    );
  }
}
