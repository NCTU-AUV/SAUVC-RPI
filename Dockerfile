FROM ubuntu:latest

RUN locale  # check for UTF-8 && \
    \
    apt update && sudo apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

RUN locale  # verify settings

RUN apt install software-properties-common && \
    add-apt-repository universe
