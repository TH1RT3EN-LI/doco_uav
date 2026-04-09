from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


SUPPORTED_TRANSPORTS = {"udp4", "udp6", "tcp4", "tcp6", "serial"}


def _create_micro_xrce_agent_process(context):
    agent_executable = LaunchConfiguration("agent_executable").perform(context).strip()
    transport = LaunchConfiguration("transport").perform(context).strip().lower()
    port = LaunchConfiguration("port").perform(context).strip()
    serial_device = LaunchConfiguration("serial_device").perform(context).strip()
    serial_baudrate = LaunchConfiguration("serial_baudrate").perform(context).strip()
    middleware = LaunchConfiguration("middleware").perform(context).strip()
    refs_file = LaunchConfiguration("refs_file").perform(context).strip()
    verbose_level = LaunchConfiguration("verbose_level").perform(context).strip()

    if transport not in SUPPORTED_TRANSPORTS:
        raise RuntimeError(
            "micro_xrce_agent transport must be one of "
            f"{', '.join(sorted(SUPPORTED_TRANSPORTS))}; got '{transport}'"
        )

    cmd = [agent_executable, transport]

    if transport in {"udp4", "udp6", "tcp4", "tcp6"}:
        if not port:
            raise RuntimeError(
                f"micro_xrce_agent transport '{transport}' requires a non-empty port"
            )
        cmd.extend(["-p", port])
    elif transport == "serial":
        if not serial_device:
            raise RuntimeError("micro_xrce_agent serial transport requires serial_device")
        cmd.extend(["-D", serial_device, "-b", serial_baudrate])

    if middleware:
        cmd.extend(["-m", middleware])
    if refs_file:
        cmd.extend(["-r", refs_file])
    if verbose_level:
        cmd.extend(["-v", verbose_level])

    return [ExecuteProcess(cmd=cmd, output="screen", emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "agent_executable",
                default_value=EnvironmentVariable(
                    "MICRO_XRCE_AGENT_EXECUTABLE", default_value="MicroXRCEAgent"
                ),
            ),
            DeclareLaunchArgument(
                "transport",
                default_value=EnvironmentVariable(
                    "UAV_UXRCE_AGENT_TRANSPORT", default_value="udp4"
                ),
            ),
            DeclareLaunchArgument(
                "port",
                default_value=EnvironmentVariable("UAV_UXRCE_AGENT_PORT", default_value="8888"),
            ),
            DeclareLaunchArgument(
                "serial_device",
                default_value=EnvironmentVariable(
                    "UAV_UXRCE_AGENT_DEVICE", default_value="/dev/ttyUSB0"
                ),
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value=EnvironmentVariable(
                    "UAV_UXRCE_AGENT_BAUDRATE", default_value="921600"
                ),
            ),
            DeclareLaunchArgument(
                "middleware",
                default_value=EnvironmentVariable(
                    "UAV_UXRCE_AGENT_MIDDLEWARE", default_value="dds"
                ),
            ),
            DeclareLaunchArgument(
                "refs_file",
                default_value=EnvironmentVariable("UAV_UXRCE_AGENT_REFS_FILE", default_value=""),
            ),
            DeclareLaunchArgument(
                "verbose_level",
                default_value=EnvironmentVariable("UAV_UXRCE_AGENT_VERBOSE", default_value=""),
            ),
            OpaqueFunction(function=_create_micro_xrce_agent_process),
        ]
    )
