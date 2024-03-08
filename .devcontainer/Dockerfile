FROM ubuntu:22.04

ARG USER=autoware
ENV HOME /home/$USER
ENV PATH=$PATH:/home/$USER/.local/bin

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install \
      sudo \
      git \
      python3 \
      python3-pip \
      python-is-python3 \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

RUN groupadd work -g 1000 \
  && useradd -ms /bin/bash $USER -g 1000 -u 1000 \
  && printf "$USER:$USER" | chpasswd \
  && printf "$USER ALL= NOPASSWD: ALL\\n" >> /etc/sudoers

USER $USER
WORKDIR $HOME

RUN python -m pip install --user --no-cache-dir pre-commit
