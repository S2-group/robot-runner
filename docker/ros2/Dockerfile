FROM ros:foxy
RUN mkdir /robot-runner
WORKDIR /robot-runner
COPY ../../ /robot-runner
RUN sudo apt update && sudo apt-get install -y python3-pip
RUN python3.8 -m pip install -r ros2_rr_requirements.txt
RUN export CLOUDSDK_PYTHON=python3.8