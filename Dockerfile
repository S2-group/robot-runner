FROM ros:foxy
RUN mkdir /robot_runner
WORKDIR /robot_runner
COPY . /robot_runner
RUN sudo apt update && sudo apt-get install -y python3-pip
RUN python3.8 -m pip install -r robot_runner_requirements.txt
RUN export CLOUDSDK_PYTHON=python3.8
#ENTRYPOINT [ "python3.8" ]