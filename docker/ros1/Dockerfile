FROM ros:melodic
RUN mkdir /robot-runner
WORKDIR /robot-runner
COPY ../../ /robot-runner
RUN sudo apt update
RUN sudo apt install -y software-properties-common
RUN sudo apt-get install -y python3
RUN sudo apt install -y python3.8
RUN sudo apt-get install -y python3.8-dev
RUN sudo apt update && sudo apt-get install -y python3-pip
RUN python3.8 -m pip install -r ros1_rr_requirements.txt
RUN export CLOUDSDK_PYTHON=python3.8