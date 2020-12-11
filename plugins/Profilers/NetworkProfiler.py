import pyshark
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext

class NetworkProfiler:
    __cap = None
    __sniffer = None

    def start_sniffing_to_file_between_robot_and_remotepc(self, context: RobotRunnerContext, network_interface: str, robot_ip_addr: str, remotepc_ip_addr: str, output_file_name: str):
        display_filter = f"ip.addr=={robot_ip_addr} && ip.addr=={remotepc_ip_addr}"
        self.__start_sniffing(context, network_interface, display_filter, output_file_name)

    def start_sniffing_to_file_with_filter(self, context: RobotRunnerContext, network_interface: str, display_filter: str, output_file_name: str):
        self.__start_sniffing(context, network_interface, display_filter, output_file_name)

    def __start_sniffing(self, context: RobotRunnerContext, network_interface: str, display_filter: str, output_file_name: str):
        output_file = str(context.run_dir.absolute()) + f"/{output_file_name}"
        self.__cap = pyshark.LiveCapture(interface=network_interface, display_filter=display_filter, output_file=output_file)
        self.__sniffer = self.__cap.sniff_continuously()

    def stop_sniffing(self):
        self.__sniffer.close()