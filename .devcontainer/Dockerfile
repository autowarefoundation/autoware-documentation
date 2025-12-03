FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG USER=autoware
ENV HOME=/home/$USER
ENV PATH=$PATH:/home/$USER/.local/bin

# hadolint ignore=DL3008
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
      sudo \
      git \
      python3 \
      python3-pip \
      python-is-python3 \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

RUN groupadd work -g 1000 \
  && useradd -ms /bin/bash "$USER" -g 1000 -u 1000 \
  && printf "%s:%s" "$USER" "$USER" | chpasswd \
  && printf "%s ALL= NOPASSWD: ALL\n" "$USER" >> /etc/sudoers

USER $USER
WORKDIR $HOME

# hadolint ignore=DL3013
RUN python -m pip install --user --no-cache-dir pre-commit
