FROM ubuntu:16.04
WORKDIR /ardupilot

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    software-properties-common \
    python-software-properties && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ADD Tools/environment_install/install-prereqs-ubuntu.sh /ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
ENV USER=root
RUN bash -c "Tools/environment_install/install-prereqs-ubuntu.sh -y && apt-get install gcc-arm-none-eabi -y" && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ADD . /ardupilot
ENV CCACHE_MAXSIZE=1G
ENV PATH /usr/lib/ccache:/ardupilot/Tools:${PATH}
