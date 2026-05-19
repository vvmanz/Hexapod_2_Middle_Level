FROM microros/micro_ros_static_library_builder:foxy

# Закомментировать все ROS-репозитории, чтобы не мешали обновлению
RUN find /etc/apt/ -name '*.list' -exec sed -i 's|^deb .*packages.ros.org|# &|' {} \;

# Теперь apt-get update работает без ROS
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release

# Загрузить и установить новый GPG ключ ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Добавить ROS-репозиторий с новым ключом
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Теперь обновить пакеты с новым ключом
RUN apt-get update
