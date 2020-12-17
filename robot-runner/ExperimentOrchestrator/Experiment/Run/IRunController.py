from ProgressManager.Output.CSVOutputManager import CSVOutputManager
from pathlib import Path
from abc import ABC, abstractmethod
from multiprocessing import Event

from ConfigValidator.Config.RobotRunnerConfig import RobotRunnerConfig
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext

class IRunController(ABC):
    run_dir: Path = None
    current_run: int = None
    variation: tuple = None
    config: RobotRunnerConfig = None
    run_context: RobotRunnerContext = None
    data_manager: CSVOutputManager = None

    def __init__(self, variation: tuple, config: RobotRunnerConfig, current_run: int, total_runs: int):
        self.run_dir = Path(str(config.experiment_path.absolute()) + f"/{variation['__run_id']}")
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.variation = variation
        self.config = config
        self.current_run = current_run
        self.run_context = RobotRunnerContext(self.variation, self.current_run, self.run_dir)
        self.data_manager = CSVOutputManager(str(self.config.experiment_path.absolute()))

        self.run_completed_event = Event()

        print(f"\n-----------------NEW RUN [{current_run} / {total_runs}]-----------------\n")

    @abstractmethod
    def do_run(self):
        pass
