FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
COPY requirements.txt /requirements.txt
RUN pip3 install -r requirements.txt
ENV SHELL /bin/bash

CMD ["/bin/bash"]