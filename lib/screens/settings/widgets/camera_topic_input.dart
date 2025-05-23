import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros2_api/ros2_api.dart';
import '../../../../providers/connection_provider.dart';
import 'package:rosapi_msgs/srvs.dart';

class CameraTopicInput extends StatefulWidget {
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
  State<CameraTopicInput> createState() => _CameraTopicInputState();
}

class _CameraTopicInputState extends State<CameraTopicInput> {
  // Static list to maintain topics across widget instances
  static List<String> _sessionTopics = [];
  bool _isLoading = false;
  String? _errorMessage;

  @override
  void initState() {
    super.initState();
    // Only fetch if we haven't fetched before in this session
    if (_sessionTopics.isEmpty) {
      _fetchTopics();
    } else {
      // If we have topics and initialValue isn't in the list, select first topic
      if (!_sessionTopics.contains(widget.initialValue) &&
          _sessionTopics.isNotEmpty) {
        widget.onChanged(_sessionTopics.first);
      }
    }
  }

  Future<void> _fetchTopics() async {
    final connectionProvider = context.read<ConnectionProvider>();
    final ros2 = connectionProvider.ros2Client;

    setState(() {
      _isLoading = true;
      _errorMessage = null;
    });

    try {
      final client = ServiceClient<TopicsForType, TopicsForTypeRequest,
          TopicsForTypeResponse>(
        ros2: ros2!,
        name: '/rosapi/topics_for_type',
        type: TopicsForType().fullType,
        serviceType: TopicsForType(),
      );

      final response = await client
          .call(TopicsForTypeRequest(type: 'sensor_msgs/msg/CompressedImage'));

      final topics = response.topics.where((t) => t.isNotEmpty).toList();

      // Auto-select logic
      String? selectedTopic;
      if (topics.contains(widget.initialValue)) {
        selectedTopic = widget.initialValue;
      } else if (topics.isNotEmpty) {
        selectedTopic = topics.first;
        widget.onChanged(selectedTopic);
      }

      setState(() {
        _sessionTopics = topics; // Update the static list
        if (selectedTopic != null) {
          widget.onChanged(selectedTopic);
        }
      });
    } catch (e) {
      setState(() => _errorMessage = 'Failed to fetch topics: ${e.toString()}');
    } finally {
      setState(() => _isLoading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Expanded(
              child: _buildTopicDropdown(),
            ),
            SizedBox(width: 10),
            Column(
              children: [
                Text(
                  'Enable Camera',
                  style: TextStyle(
                    fontSize: widget.screenSize.height * 0.018,
                    color: Colors.grey.shade400,
                  ),
                ),
                Switch(
                  value: widget.enabled,
                  onChanged: widget.onEnabledChanged,
                  activeColor: widget.modeColor,
                ),
              ],
            ),
          ],
        ),
        Text(
          'Type: sensor_msgs/msg/CompressedImage',
          style: TextStyle(
            fontSize: 10,
            color: Colors.grey.shade400,
            fontStyle: FontStyle.italic,
          ),
        ),
        if (_errorMessage != null)
          Padding(
            padding: const EdgeInsets.only(top: 4),
            child: Text(
              _errorMessage!,
              style: TextStyle(
                color: Colors.red,
                fontSize: 10,
              ),
            ),
          ),
      ],
    );
  }

  Widget _buildTopicDropdown() {
    return Container(
      decoration: BoxDecoration(
        color: Colors.grey[900],
        borderRadius: BorderRadius.circular(8),
        border: Border.all(
          color: widget.modeColor.withOpacity(0.5),
          width: 1,
        ),
      ),
      child: Row(
        children: [
          Expanded(
            child: Theme(
              data: Theme.of(context).copyWith(
                inputDecorationTheme: InputDecorationTheme(
                  border: InputBorder.none,
                  contentPadding: EdgeInsets.zero,
                ),
              ),
              child: DropdownButtonHideUnderline(
                child: ButtonTheme(
                  alignedDropdown: true,
                  child: DropdownButton<String>(
                    value: _sessionTopics.contains(widget.initialValue)
                        ? widget.initialValue
                        : null,
                    items: [
                      if (_sessionTopics.isEmpty)
                        DropdownMenuItem(
                          value: '',
                          enabled: false,
                          child: Text(
                            'No topics found',
                            style: TextStyle(
                              color: Colors.grey[600],
                              fontSize: 12,
                            ),
                          ),
                        )
                      else
                        ..._sessionTopics.map((topic) => DropdownMenuItem(
                              value: topic,
                              child: Text(
                                topic,
                                style: TextStyle(
                                  color: Colors.white,
                                  fontSize: 12,
                                ),
                              ),
                            )),
                    ],
                    onChanged: (value) {
                      if (value != null) {
                        widget.onChanged(value);
                      }
                    },
                    isExpanded: true,
                    icon: Icon(
                      Icons.arrow_drop_down,
                      color: widget.modeColor,
                    ),
                    hint: Text(
                      _isLoading ? 'Loading topics...' : 'Select topic',
                      style: TextStyle(
                        color: Colors.grey[400],
                        fontSize: 12,
                      ),
                    ),
                    dropdownColor: Colors.grey[850],
                    menuMaxHeight: 150,
                  ),
                ),
              ),
            ),
          ),
          Container(
            height: 42,
            width: 1,
            color: widget.modeColor.withOpacity(0.3),
          ),
          Material(
            color: Colors.transparent,
            child: InkWell(
              onTap: _isLoading ? null : _fetchTopics,
              borderRadius: BorderRadius.horizontal(right: Radius.circular(7)),
              child: Container(
                width: 42,
                height: 42,
                padding: EdgeInsets.all(8),
                child: _isLoading
                    ? SizedBox(
                        width: 20,
                        height: 20,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor:
                              AlwaysStoppedAnimation<Color>(widget.modeColor),
                        ),
                      )
                    : Icon(
                        Icons.refresh_rounded,
                        size: 24,
                        color: widget.modeColor,
                      ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
