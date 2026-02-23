ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full
ARG BRANCH=rover
ARG USER=docker
RUN useradd -m -s /bin/bash ${USER} && usermod -aG sudo ${USER} && echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER} && chmod 0440 /etc/sudoers.d/${USER}

# Install Utilities
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo xterm init systemd snapd vim net-tools \
    curl wget git build-essential cmake cppcheck \
    gnupg libeigen3-dev libgles2-mesa-dev libcgal-dev libfftw3-dev \
    lsb-release pkg-config protobuf-compiler \
    python3-dbg python3-vcstool python3-pip python3-venv python3-pexpect \
    python-is-python3 python3-future python3-wxgtk4.0 python3-rosdep python3-colcon-common-extensions \
    qtbase5-dev ruby dirmngr gnupg2 nano xauth \
    software-properties-common htop libtool \
    x11-apps mesa-utils bison flex automake \
    && rm -rf /var/lib/apt/lists/

# Prereqs for Ardupilot - Ardurover
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true

ADD --chown=root:root --chmod=0644 https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list /etc/ros/rosdep/sources.list.d/00-gazebo.list
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |  tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    libgz-sim8-dev rapidjson-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libasio-dev \
    # mavproxy setting
    python3-dev python3-opencv python3-matplotlib python3-lxml \
    && rm -rf /var/lib/apt/lists/

# Install Ardupilot - Ardurover .docker/ardurover.dockerfile
USER docker
RUN wget -O /tmp/install.sh https://raw.githubusercontent.com/kmjeong000/dave/$BRANCH/extras/ardurover-ubuntu-install-local.sh
RUN chmod +x /tmp/install.sh && bash /tmp/install.sh

USER root
# Locale for UTF-8
RUN truncate -s0 /tmp/preseed.cfg && \
   (echo "tzdata tzdata/Areas select Etc" >> /tmp/preseed.cfg) && \
   (echo "tzdata tzdata/Zones/Etc select UTC" >> /tmp/preseed.cfg) && \
   debconf-set-selections /tmp/preseed.cfg && \
   rm -f /etc/timezone && \
   dpkg-reconfigure -f noninteractive tzdata
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get -y install --no-install-recommends locales tzdata \
    && rm -rf /tmp/*
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Install ROS-Gazebo framework
ADD https://raw.githubusercontent.com/kmjeong000/dave/$BRANCH/\
extras/ros-jazzy-gz-harmonic-install.sh install.sh
RUN bash install.sh

# Install mavros
RUN apt-get update && \
    apt-get -y install --no-install-recommends ros-jazzy-mavros* \
    && rm -rf /tmp/*
WORKDIR /opt/mavros_ws
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# Set up Dave workspace
ENV DAVE_UNDERLAY=/home/${USER}/dave_ws
WORKDIR $DAVE_UNDERLAY/src
RUN wget -O /home/${USER}/dave_ws/dave.repos -q https://raw.githubusercontent.com/kmjeong000/dave/$BRANCH/\
extras/repos/dave.$ROS_DISTRO.repos
RUN vcs import --shallow --input "/home/${USER}/dave_ws/dave.repos"

# Install dave dependencies
RUN apt-get update && \
    ( [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init ) && \
    rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# Compile Dave
WORKDIR $DAVE_UNDERLAY
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && colcon build
WORKDIR /

#Build wave sim
WORKDIR $DAVE_UNDERLAY/src/dave/gazebo/dave_gz_world_plugins/ocean-waves
RUN colcon build

#Build wave sim GUI plugin
WORKDIR $DAVE_UNDERLAY/src/dave/gazebo/dave_gz_world_plugins/ocean-waves/src/gui/plugins/waves_control
RUN mkdir build && cd build && cmake .. && make

## cable_ws
ENV CABLE_WS=/home/${USER}/cable_ws
WORKDIR /home/${USER}
RUN git clone --depth 1 https://github.com/kmjeong000/cable_ws.git $CABLE_WS
WORKDIR $CABLE_WS
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    . "$DAVE_UNDERLAY/install/setup.sh" && \
    colcon build

#Patch for wave sim
RUN ln -sf /opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/libOgreNextMain.so.2.3.3 /opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/libOgreNextMain.so.2.3.1

# Set up bashrc for root
USER docker
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /opt/gazebo/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/mavros/install/setup.bash" >> ~/.bashrc && \
    echo "source $DAVE_UNDERLAY/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/dave_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source $CABLE_WS/install/setup.bash" >> ~/.bashrc && \
    echo "export GEOGRAPHICLIB_GEOID_PATH=/usr/local/share/GeographicLib/geoids" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/gazebo/install/lib/python" >> ~/.bashrc && \
    echo "export PATH=/home/$USER/ardupilot_ws/ardupilot/Tools/autotest:\$PATH" >> ~/.bashrc && \
    echo "export PATH=/home/$USER/ardupilot_ws/ardupilot/build/sitl/bin:\$PATH" >> ~/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/${USER}/ardupilot_ws/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH:/home/docker/dave_ws/src/dave/gazebo/dave_gz_world_plugins/ocean-waves/install/wave/lib" >> ~/.bashrc && \
    echo "export GZ_GUI_PLUGIN_PATH=\$GZ_GUI_PLUGIN_PATH:/home/docker/dave_ws/src/dave/gazebo/dave_gz_world_plugins/ocean-waves/src/gui/plugins/waves_control/build" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/home/docker/dave_ws/src/dave/gazebo/dave_gz_world_plugins/ocean-waves/install/wave/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/home/${USER}/ardupilot_ws/ardupilot_gazebo/models:/home/${USER}/ardupilot_ws/ardupilot_gazebo/worlds:/home/docker/cable_ws/install/cable_sim/share:/home/docker/cable_ws/src:/home/docker/cable_ws/src/cable_sim:/home/docker/cable_ws/src/cable_sim/models:/home/docker/cable_ws/install:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc && \
    echo "\n\n" >> ~/.bashrc && \
    echo "if [ -d ~/HOST ]; then chown ${USER}:${USER} ~/HOST; fi" >> ~/.bashrc  && \
    echo "export PS1='\[\e[1;36m\]\u@DAVE_docker\[\e[0m\]\[\e[1;34m\](\$(hostname | cut -c1-12))\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '" >> ~/.bashrc

# Other environment variables
RUN echo "export XDG_RUNTIME_DIR=~/.xdg_log" >> ~/.bashrc && \
    echo "unset SESSION_MANAGER" >> ~/.bashrc

# Create and activate Python virtual environment
RUN python3 -m venv /home/docker/.venv && \
    . /home/docker/.venv/bin/activate && \
    pip install --upgrade pip setuptools wheel && \
    pip install PyYAML pygame mavproxy pexpect packaging urllib3 empy==3.3.4 websockets catkin_pkg future && \
    echo "alias venv='source /home/docker/.venv/bin/activate'" >> ~/.bashrc
ENV PATH="/home/docker/.venv/bin:$PATH"

USER root
RUN mkdir -p /usr/local/share/GeographicLib/geoids && \
    ln -sf /usr/share/GeographicLib/geoids/egm96-5.pgm /usr/local/share/GeographicLib/geoids/egm96-5.pgm && \
    chmod 644 /usr/share/GeographicLib/geoids/egm96-5.pgm
USER docker

# Remove sudo message
RUN touch /home/docker/.sudo_as_admin_successful

RUN mkdir -p /home/docker/.config/autostart && \
    touch ~/.dave_entrypoint && printf '\033[1;36m =====\n' >> ~/.dave_entrypoint && \
    printf '  ____    ___     _______      _                     _   _      \n' >> ~/.dave_entrypoint && \
    printf ' |  _ \  / \ \   / | ____|    / \   __ _ _   _  __ _| |_(_) ___ \n' >> ~/.dave_entrypoint && \
    printf ' | | | |/ _ \ \ / /|  _|     / _ \ / _` | | | |/ _` | __| |/ __|\n' >> ~/.dave_entrypoint && \
    printf ' | |_| / ___ \ V / | |___   / ___ | (_| | |_| | (_| | |_| | (__ \n' >> ~/.dave_entrypoint && \
    printf ' |____/_/   \_\_/  |_____| /_/   \_\__, |\__,_|\__,_|\__|_|\___|\n' >> ~/.dave_entrypoint && \
    printf ' __     ___      _               _     _____            _       \n' >> ~/.dave_entrypoint && \
    printf ' \ \   / (_)_ __| |_ _   _  __ _| |   | ____|_ ____   _(_)_ __  \n' >> ~/.dave_entrypoint && \
    printf '  \ \ / /| | `__| __| | | |/ _` | |   |  _| | `_ \ \ / | | `__| \n' >> ~/.dave_entrypoint && \
    printf '   \ V / | | |  | |_| |_| | (_| | |   | |___| | | \ V /| | |_   \n' >> ~/.dave_entrypoint && \
    printf '    \_/  |_|_|   \__|\__,_|\__,_|_|   |_____|_| |_|\_/ |_|_(_)  \n\033[0m' >> ~/.dave_entrypoint && \
    printf '\033[1;32m\n =====\n\033[0m' >> ~/.dave_entrypoint && \
    printf "\\033[1;32m 👋 Hi! This is Docker virtual environment for DAVE\n\\033[0m" \
    >> ~/.dave_entrypoint && \
    printf "\\033[1;33m\tROS2 Jazzy - Gazebo Harmonic (w ardupilot(ardusub) + mavros)\n\n\\033[0m" \
    >> ~/.dave_entrypoint && \
    printf "\\033[1;33m\tVirtual environment shortcut: 'venv' (type this command to activater the environment)\n\n\\033[0m" \
    >> ~/.dave_entrypoint && \
    echo 'cat ~/.dave_entrypoint' >> ~/.bashrc
    
# Autostart terminal
# hadolint ignore=SC3037
RUN echo "[Desktop Entry]\nType=Application" \
    > /home/docker/.config/autostart/terminal.desktop && \
    echo "Exec=gnome-terminal -- bash -c 'cat ~/.hi; exec bash'" \
    >> /home/docker/.config/autostart/terminal.desktop && \
    echo -e "X-GNOME-Autostart-enabled=true" \
    >> /home/docker/.config/autostart/terminal.desktop

WORKDIR /home/docker