from pathlib import Path

class RobotRunnerContext:
    run_variation: dict
    run_nr:  int
    run_dir: Path

    def __init__(self, run_variation: dict, run_nr: int, run_dir: Path):
        self.run_variation = run_variation
        self.run_nr = run_nr
        self.run_dir = run_dir