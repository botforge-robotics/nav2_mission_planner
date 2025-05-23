import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros2_api/ros2_api.dart';
import 'package:rosapi_msgs/srvs.dart';
import 'package:nav2_mission_planner/providers/connection_provider.dart';

class OdomTopicInput extends StatefulWidget {
  final String initialValue;
  final String initialValueType;
  final Function(String, String) onChanged;
  final Size screenSize;
  final Color modeColor;

  const OdomTopicInput({
    super.key,
    required this.initialValue,
    required this.onChanged,
    required this.screenSize,
    required this.modeColor,
    required this.initialValueType,
  });

  @override
  State<OdomTopicInput> createState() => _OdomTopicInputState();
}

class _OdomTopicInputState extends State<OdomTopicInput> {
  static Map<String, String> _sessionTopics = {};
  bool _isLoading = false;
  String? _errorMessage;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_sessionTopics.isEmpty) {
        _fetchTopics();
      } else if (!_sessionTopics.containsKey(widget.initialValue) &&
          _sessionTopics.isNotEmpty) {
        widget.onChanged(
            _sessionTopics.keys.first, _sessionTopics.values.first);
      }
    });
  }

  Future<void> _fetchTopics() async {
    final connectionProvider = context.read<ConnectionProvider>();
    final ros2 = connectionProvider.ros2Client;

    setState(() {
      _isLoading = true;
      _errorMessage = null;
      _sessionTopics = {};
    });

    try {
      final client = ServiceClient<TopicsForType, TopicsForTypeRequest,
          TopicsForTypeResponse>(
        ros2: ros2!,
        name: '/rosapi/topics_for_type',
        type: TopicsForType().fullType,
        serviceType: TopicsForType(),
      );

      final typesToCheck = [
        'nav_msgs/msg/Odometry',
        'geometry_msgs/msg/PoseWithCovarianceStamped'
      ];

      for (final type in typesToCheck) {
        final response = await client.call(TopicsForTypeRequest(type: type));
        for (final topic in response.topics) {
          if (topic.isNotEmpty) {
            _sessionTopics[topic] = type;
          }
        }
      }

      String? selectedTopic = widget.initialValue;
      String? selectedType = widget.initialValueType;

      if (!_sessionTopics.containsKey(selectedTopic) ||
          _sessionTopics[selectedTopic] != selectedType) {
        if (_sessionTopics.isNotEmpty) {
          selectedTopic = _sessionTopics.keys.first;
          selectedType = _sessionTopics.values.first;
        }
      }

      setState(() {
        if (selectedTopic != null && selectedType != null) {
          widget.onChanged(selectedTopic, selectedType);
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
        _buildTopicDropdown(),
        SizedBox(height: widget.screenSize.height * 0.01),
        Text(
          'Type: nav_msgs/msg/Odometry or geometry_msgs/msg/PoseWithCovarianceStamped',
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
                fontSize: widget.screenSize.height * 0.02,
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
                    value: _sessionTopics.containsKey(widget.initialValue) &&
                            _sessionTopics[widget.initialValue] ==
                                widget.initialValueType
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
                        ..._sessionTopics.entries
                            .map((entry) => DropdownMenuItem(
                                  value: entry.key,
                                  child: Column(
                                    crossAxisAlignment:
                                        CrossAxisAlignment.start,
                                    mainAxisSize: MainAxisSize.min,
                                    children: [
                                      Text(entry.key,
                                          style: TextStyle(
                                            color: Colors.white,
                                            fontSize: 12,
                                          )),
                                      Text(
                                        entry.value,
                                        style: TextStyle(
                                          fontSize: 10,
                                          color: Colors.grey[400],
                                        ),
                                      ),
                                    ],
                                  ),
                                )),
                    ],
                    onChanged: (value) {
                      if (value != null) {
                        final type = _sessionTopics[value];
                        if (type != null) {
                          widget.onChanged(value, type);
                        }
                      }
                    },
                    isExpanded: true,
                    icon: Icon(Icons.arrow_drop_down, color: widget.modeColor),
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
                    : Icon(Icons.refresh_rounded,
                        size: 24, color: widget.modeColor),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
