
###     =========================================================
###     |                                                       |
###     |              ConfigValidationController               |
###     |       - Validate config.json and ConfigModel          |
###     |                                                       |
###     |       - Give comprehensible error messages to user    |
###     |       - including possible fix for error              |
###     |                                                       |
###     |       * Example of validation: env variable           |
###     |       * ROS_VERSION (1 or 2) can be checked if it     |
###     |       * exists and if the provided launch file is     |
###     |       * of the correct format (.launch or .launch.py) |
###     |                                                       |
###     =========================================================
class ConfigValidationController:
    pass
# TODO: Add ros_version env variable check etc. add comprehensible error messages