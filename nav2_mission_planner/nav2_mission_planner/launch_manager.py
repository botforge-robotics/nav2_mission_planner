import uuid
import rclpy
from rclpy.node import Node
from nav2_mission_planner_interfaces.srv import LaunchWithArgs, StopLaunch, GetMapList, DeleteMap
import subprocess
from collections import OrderedDict
import os
import signal
from pathlib import Path
from ament_index_python import get_package_share_directory, PackageNotFoundError
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from std_msgs.msg import Int32


class LaunchManager(Node):
    def __init__(self):
        super().__init__('launch_manager')
        self.launch_service = self.create_service(
            LaunchWithArgs, 'launch_with_args', self.launch_callback)
        self.stop_service = self.create_service(
            StopLaunch, 'stop_launch', self.stop_callback)
        self.map_list_service = self.create_service(
            GetMapList, 'get_map_list', self.get_map_list_callback
        )
        self.delete_map_service = self.create_service(
            DeleteMap, 'delete_map', self.delete_map_callback)
        self.active_launches = OrderedDict()
        # Set logger to debug level for verbose internal state reporting
        try:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        except Exception:
            # Fallback in case the API is unavailable in some distributions
            pass

        self.get_logger().info('Launch Manager ready with start/stop capabilities')
        self._shutting_down = False  # Add shutdown flag

        # Subscribe to client count to monitor active UI/clients
        self.client_count_sub = self.create_subscription(
            Int32,
            '/client_count',  # Topic name
            self.client_count_callback,
            10
        )

    def check_package_exists(self, package_name):
        """Check if ROS package exists using ament index"""
        self.get_logger().debug(f"Checking package existence: {package_name}")
        try:
            get_package_share_directory(package_name)
            self.get_logger().debug(f"Package '{package_name}' found")
            return True
        except PackageNotFoundError:
            self.get_logger().debug(f"Package '{package_name}' not found")
            return False

    def check_launch_file_exists(self, package_name, launch_file):
        """Check if launch file exists using substitution-based path joining"""
        self.get_logger().debug(
            f"Validating launch file '{launch_file}' in package '{package_name}'")
        try:
            package_share = get_package_share_directory(package_name)
            launch_path = Path(package_share) / 'launch' / launch_file
            exists = launch_path.exists()
            self.get_logger().debug(
                f"Launch file path: {launch_path} exists: {exists}")
            return exists
        except Exception:
            self.get_logger().debug("Exception while checking launch file existence", exc_info=True)
            return False

    def launch_callback(self, request, response):
        self.get_logger().debug(
            f"launch_callback called with package={request.package}, launch_file={request.launch_file}, arguments='{request.arguments}'")
        try:
            # Validate package existence
            if not self.check_package_exists(request.package):
                response.success = False
                response.message = f"Package '{request.package}' not found"
                response.unique_id = ""
                self.get_logger().error(response.message)
                return response

            # Validate launch file existence
            if not self.check_launch_file_exists(request.package, request.launch_file):
                response.success = False
                response.message = f"Launch file '{request.launch_file}' not found in package '{request.package}'"
                response.unique_id = ""
                self.get_logger().error(response.message)
                return response

            # Generate unique ID and prepare command
            unique_id = str(uuid.uuid4())
            cmd = ['ros2', 'launch', request.package, request.launch_file]

            # Handle arguments if provided
            if request.arguments.strip():
                try:
                    args_list = request.arguments.split()
                    cmd.extend(args_list)
                except Exception as e:
                    response.success = False
                    response.message = f"Invalid arguments format: {str(e)}"
                    response.unique_id = ""
                    self.get_logger().error(response.message)
                    return response

            # Check if this is a map saving operation by launch file name
            is_map_saver = request.launch_file == 'save_map.launch.py'

            # For map saving, run synchronously
            if is_map_saver:
                self.get_logger().info("Starting synchronous map save operation")
                self.get_logger().debug(
                    f"Executing map save command: {' '.join(cmd)}")

                # -------------------------------------------------
                # Pre-save: capture existing maps (if path detectable)
                # -------------------------------------------------
                verification_path = None
                args_dict = {}
                try:
                    for token in args_list if 'args_list' in locals() else []:
                        if ':=' in token:
                            k, v = token.split(':=', 1)
                            args_dict[k] = v

                    # Common key names for map directory
                    map_dir_key = next(
                        (k for k in ['path', 'map_path', 'map_dir'] if k in args_dict), None)
                    if map_dir_key:
                        # If the provided value already looks like 'package/subdir', use as-is,
                        # otherwise prefix with the launch package name.
                        val = args_dict[map_dir_key]
                        verification_path = val if '/' in val else f"{request.package}/{val}"
                except Exception:
                    pass

                pre_maps = None
                if verification_path:
                    try:
                        pre_maps = self._retrieve_map_list(verification_path)
                    except Exception as e:
                        self.get_logger().warning(
                            f"Pre-save map list retrieval failed: {e}")

                # -------------------------------------------------
                # Execute the save map launch synchronously
                # -------------------------------------------------
                process = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    check=False
                )

                # Log process completion details
                self.get_logger().debug(
                    f"Map save process finished with return code {process.returncode}")
                self.get_logger().debug(f"Map save stdout: {process.stdout}")
                self.get_logger().debug(f"Map save stderr: {process.stderr}")

                # -------------------------------------------------
                # Post-save verification
                # -------------------------------------------------
                post_maps = None
                map_list_change = False
                if verification_path:
                    try:
                        post_maps = self._retrieve_map_list(verification_path)
                        if pre_maps is not None and post_maps is not None:
                            map_list_change = len(post_maps) > len(pre_maps)
                    except Exception as e:
                        self.get_logger().warning(
                            f"Post-save map list retrieval failed: {e}")

                # Determine success criteria (ensure boolean)
                newly_created = map_list_change
                response.success = (
                    process.returncode == 0 and
                    (not verification_path or newly_created)
                )

                # Decide message based on outcome
                if verification_path and not newly_created and process.returncode == 0:
                    # Map file already existed; nothing new created
                    response.message = "already exist"
                    response.success = False
                else:
                    response.message = "Map save " + (
                        "successful" if response.success else "failed"
                    )

                response.unique_id = unique_id

                # Additional high-level log for success/failure
                if response.success:
                    self.get_logger().info("Map saved and verified successfully")
                else:
                    self.get_logger().error("Map save verification failed; see debug logs for details")

                return response

            # For normal async launches
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True
            )

            self.get_logger().debug(
                f"Subprocess started with PID {process.pid}")

            # Check if process started successfully
            if process.poll() is not None:
                stderr_output = process.stderr.read()
                response.success = False
                response.message = f"Process failed to start: {stderr_output}"
                response.unique_id = ""
                self.get_logger().error(response.message)
                return response

            # Store process information for async launches
            self.active_launches[unique_id] = {
                'process': process,
                'cmd': ' '.join(cmd),
                'start_time': self.get_clock().now().to_msg()
            }

            self.get_logger().debug(
                f"Active launches updated: {list(self.active_launches.keys())}")

            # Set success response
            response.success = True
            response.message = f"Launch initiated with ID: {unique_id}"
            response.unique_id = unique_id
            self.get_logger().info(
                f"Started launch {unique_id}: {' '.join(cmd)}")

        except Exception as e:
            self.get_logger().debug("Unexpected exception in launch_callback", exc_info=True)
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            response.unique_id = ""
            self.get_logger().error(response.message)

        return response

    def stop_callback(self, request, response):
        self.get_logger().debug(
            f"stop_callback called with unique_id={request.unique_id}")
        unique_id = request.unique_id
        if unique_id not in self.active_launches:
            response.success = False
            response.message = f"Invalid launch ID: {unique_id}"
            self.get_logger().warning(response.message)
            return response

        launch_info = self.active_launches.pop(unique_id)
        try:
            process = launch_info['process']
            self.get_logger().debug(f"Process PID to stop: {process.pid}")
            graceful = False

            # Send SIGINT (Ctrl+C equivalent) and wait for 60 seconds
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            self.get_logger().info("Attempting graceful stop with 60 second timeout")

            try:
                process.wait(timeout=60.0)
                graceful = True
                response.message = "Process stopped gracefully"
                self.get_logger().debug("Process stopped gracefully within timeout")
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Graceful stop timed out, forcing termination")
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                try:
                    process.wait(timeout=30.0)
                    response.message = "Process forcibly terminated after timeout"
                    self.get_logger().debug("Process terminated with SIGTERM")
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    response.message = "Process killed with SIGKILL after failed termination"
                    self.get_logger().debug("Process killed with SIGKILL")

            response.success = True
            self.get_logger().info(
                f"Successfully stopped: {launch_info['cmd']}")

        except ProcessLookupError:
            self.get_logger().debug("ProcessLookupError: process already terminated")
            response.success = True
            response.message = "Process already terminated"
        except Exception as e:
            response.success = False
            response.message = f"Stop failed: {str(e)}"
            self.get_logger().error(response.message)

        # After handling, log current active launches
        self.get_logger().debug(
            f"Remaining active launches: {list(self.active_launches.keys())}")

        return response

    def get_map_list_callback(self, request, response):
        try:
            map_files = self._retrieve_map_list(request.path)
            response.maplist = map_files
            response.success = True
            response.message = f"Found {len(map_files)} maps in {request.path}"
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.maplist = []
            self.get_logger().error(f"Map list error: {str(e)}")

        return response

    # -------------------------------------------------
    # Internal utility: retrieve map list for a given package/path
    # -------------------------------------------------
    def _retrieve_map_list(self, path: str):
        """Return list of map (yaml) names for a given 'package/relative_path'.

        Raises ValueError on any validation failure.
        """
        self.get_logger().debug(f"_retrieve_map_list called with path={path}")

        if '/' not in path:
            raise ValueError("Path format must be 'package_name/path_to_maps'")

        package_name, map_path = path.split('/', 1)

        # Get package share directory using ament index
        try:
            package_share = get_package_share_directory(package_name)
        except PackageNotFoundError:
            raise ValueError(f"Package '{package_name}' not found")

        # Build full path using pathlib
        maps_dir = Path(package_share) / map_path

        if not maps_dir.exists():
            raise ValueError(f"Map directory not found: {maps_dir}")

        # Find all YAML files and extract names
        map_files = [f.stem for f in maps_dir.glob('*.yaml') if f.is_file()]

        if not map_files:
            raise ValueError("No .yaml map files found in directory")

        self.get_logger().debug(f"Maps found: {map_files}")
        return map_files

    def delete_map_callback(self, request, response):
        self.get_logger().debug(
            f"delete_map_callback called with map_path={request.map_path}, map_name={request.map_name}")
        # 1) Path format
        if '/' not in request.map_path:
            response.success = False
            response.message = "Invalid map_path. Use 'package_name/path_to_maps'"
            return response

        package_name, rel_path = request.map_path.split('/', 1)

        # 2) Package exists?
        try:
            pkg_share = get_package_share_directory(package_name)
        except PackageNotFoundError:
            response.success = False
            response.message = f"Package '{package_name}' not found"
            return response

        # 3) Directory exists?
        maps_dir = Path(pkg_share) / rel_path
        if not maps_dir.is_dir():
            response.success = False
            response.message = f"Maps directory not found: {maps_dir}"
            return response

        # 4) Delete both .yaml and .pgm files
        exts = ['.yaml', '.pgm']
        deleted = []
        errors = []

        for ext in exts:
            fpath = maps_dir / (request.map_name + ext)
            if fpath.is_file():
                try:
                    fpath.unlink()
                    deleted.append(fpath.name)
                except Exception as e:
                    errors.append(f"Failed to delete {fpath.name}: {e}")
            else:
                errors.append(f"File not found: {fpath.name}")

        # 5) Prepare response
        if deleted:
            response.success = True
            response.message = f"Deleted: {', '.join(deleted)}"
            if errors:
                response.message += f"; warnings: {', '.join(errors)}"
        else:
            response.success = False
            response.message = '; '.join(errors)

        self.get_logger().debug(
            f"Delete results - deleted: {deleted}, errors: {errors}")

        return response

    def destroy_node(self):
        self.get_logger().debug("destroy_node called")
        self._shutting_down = True
        self.get_logger().info("Shutting down and cleaning up launches")
        # Cleanup all active processes
        for uid, info in list(self.active_launches.items()):
            try:
                self.get_logger().info(f"Terminating {uid}")
                os.killpg(os.getpgid(info['process'].pid), signal.SIGTERM)
                info['process'].wait(timeout=1)
            except Exception as e:
                self.get_logger().error(f"Cleanup error for {uid}: {str(e)}")
                self.get_logger().debug("Exception during destroy_node cleanup", exc_info=True)
        self.active_launches.clear()
        super().destroy_node()

    def __del__(self):
        self.get_logger().debug("__del__ called")
        if not self._shutting_down:
            self.get_logger().warning("Destructor called without proper shutdown")
            for uid, info in self.active_launches.items():
                info['process'].terminate()
            self.active_launches.clear()

    # -------------------------------------------------
    # Callback: Client count monitoring
    # -------------------------------------------------
    def client_count_callback(self, msg: Int32):
        """Stop all active launches if no clients are connected.

        The assumption is that an external entity publishes the current number
        of connected clients (e.g., web UI sessions). When the count drops to
        zero, we terminate all running launches to free resources.
        """
        self.get_logger().debug(
            f"client_count_callback received count={msg.data}")
        try:
            if msg.data == 0 and self.active_launches:
                self.get_logger().info(
                    "Client count is 0. Stopping all active launches.")
                # Iterate over a copy to avoid modification during iteration
                for uid, info in list(self.active_launches.items()):
                    try:
                        self.get_logger().info(f"Auto-stopping launch {uid}")
                        process = info['process']
                        # Re-use the same stop logic: SIGINT then fallback force
                        os.killpg(os.getpgid(process.pid), signal.SIGINT)
                        try:
                            process.wait(timeout=30.0)
                            self.get_logger().info(
                                f"Launch {uid} stopped gracefully")
                        except subprocess.TimeoutExpired:
                            self.get_logger().warning(
                                f"Graceful stop of {uid} timed out, forcing")
                            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                            try:
                                process.wait(timeout=2.0)
                            except subprocess.TimeoutExpired:
                                os.killpg(os.getpgid(process.pid),
                                          signal.SIGKILL)
                                self.get_logger().error(
                                    f"Launch {uid} killed with SIGKILL")
                    except Exception as e:
                        self.get_logger().error(
                            f"Failed to auto-stop {uid}: {str(e)}")
                    finally:
                        # Ensure removal from active list
                        self.active_launches.pop(uid, None)
        except Exception as e:
            self.get_logger().error(f"Client count callback error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = LaunchManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
