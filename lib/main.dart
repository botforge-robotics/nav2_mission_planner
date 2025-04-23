import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'screens/screens.dart';
import 'providers/settings_provider.dart';
import 'theme/app_theme.dart';
import 'providers/connection_provider.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();

  // Force landscape mode
  SystemChrome.setPreferredOrientations([
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);

  // Hide system UI bars
  SystemChrome.setEnabledSystemUIMode(
    SystemUiMode.immersiveSticky,
    overlays: [],
  );

  runApp(const Nav2MissionPlanner());
}

class Nav2MissionPlanner extends StatelessWidget {
  const Nav2MissionPlanner({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => SettingsProvider()),
        ChangeNotifierProvider(create: (_) => ConnectionProvider()),
      ],
      child: MaterialApp(
        title: 'Nav2 Mission Planner',
        theme: AppTheme.darkTheme,
        home: const HomeScreen(),
        debugShowCheckedModeBanner: false,
      ),
    );
  }
}
