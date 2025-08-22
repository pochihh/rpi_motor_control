# Multi-arch Debian 12 (works on Apple Silicon and Intel Macs)
FROM debian:12-slim

# Create a non-root user for nicer VS Code integration
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo ca-certificates curl gnupg git pkg-config \
 && rm -rf /var/lib/apt/lists/*

# Create user with passwordless sudo
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/90-${USERNAME} \
 && chmod 0440 /etc/sudoers.d/90-${USERNAME}

USER ${USERNAME}
WORKDIR /workspaces