# Distributed RR Examples

This is an example of RR running distributed. **Do not forget** to source your ROS distribution folder before the experiment.

First, run the RR locally:
```bash
    python3 robot-runner/ experiments/distributed_client/RobotRunnerConfig.py
```

Then, run the following command on the RR client machine (inside RR folder):

```bash
    PYTHONPATH=robot-runner/ python3 experiments/distributed_client/rr_client.py
```