
FROM introlab3it/rtabmap:noble-kilted

# remove ubuntu user
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN set -ex && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -a -G sudo ${USERNAME}

RUN mkdir -p /home/${USERNAME}/ros2_ws/src && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/ros2_ws

RUN echo "source /opt/ros/kilted/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /usr/share/bash-completion/completions/git" >> /home/${USERNAME}/.bashrc
