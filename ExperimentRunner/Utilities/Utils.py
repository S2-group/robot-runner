import psutil
import pathlib


class Utils:
    @staticmethod
    def check_process_running(processName: str):
        for proc in psutil.process_iter():
            try:
                # Check if process name contains the given name string.
                if processName.lower() in proc.name().lower():
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return False

    @staticmethod
    def create_dir(absolute_path: str):
        pathlib.Path(absolute_path).mkdir(parents=True, exist_ok=True)
