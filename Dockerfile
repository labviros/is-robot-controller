FROM ubuntu:16.04
LABEL maintainer mendonca.felippe@gmail.com

WORKDIR /opt
ADD service .
ADD parameters.yaml .
ADD libs/ /usr/lib/