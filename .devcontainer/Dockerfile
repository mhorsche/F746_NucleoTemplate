# [Choice] Debian / Ubuntu version (use Debian 11, Ubuntu 18.04/22.04 on local arm64/Apple Silicon): debian-11, debian-10, ubuntu-22.04, ubuntu-20.04, ubuntu-18.04
ARG VARIANT=debian-11
FROM --platform=linux/amd64 mcr.microsoft.com/vscode/devcontainers/cpp:0-${VARIANT}

# [Optional] Uncomment this section to install additional packages.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
  #
  # Install C++ tools
  && apt-get -y install --no-install-recommends libncurses5 build-essential wget curl openocd \
  #
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

# Set up a development tools directory
WORKDIR /home/dev
ADD . /home/dev

# Install Arm GNU Toolchain and add to local PATH variable
RUN wget -qO- https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 | tar -xj
ENV PATH $PATH:/home/dev/gcc-arm-none-eabi-10.3-2021.10/bin

# Set working directory for application
WORKDIR /home/app