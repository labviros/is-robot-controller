FROM ubuntu:16.04
MAINTAINER mendonca.felippe@gmail.com

WORKDIR /opt
ADD robot-controller .
ADD libs/ /usr/lib/