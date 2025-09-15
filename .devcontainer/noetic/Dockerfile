
FROM introlab3it/rtabmap:focal

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -ex && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -a -G sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p /home/${USERNAME}/catkin_ws/src && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /usr/share/bash-completion/completions/git" >> /home/${USERNAME}/.bashrc
