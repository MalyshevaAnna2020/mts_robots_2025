FROM ros:jazzy-ros-base
# FROM python:3.11-slim
# Используем официальный образ ROS 2 Jazzy

SHELL ["/bin/bash", "-c"]

# Устанавливаем pip
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*

# # Устанавливаем turtlesim
# RUN apt-get update && apt-get install -y ros-jazzy-turtlesim

# Инициализация rosdep (для установки зависимостей из package.xml)
# RUN rosdep init
RUN rosdep update

# Устанавливаем инструменты для сборки ros-пакетов
RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем GUI-инструменты (rqt, rviz2)
RUN apt-get update && \
    apt-get install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins\
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY . .

# Устанавливаем ROS-зависимости (без tf_transformations)
RUN cd /app/ros2_ws && \
    rosdep install --from-paths src --ignore-src -y

# Устанавливаем tf_transformations через pip (не входит в ROS)
# RUN pip3 install tf-transformations --break-system-packages
RUN apt-get update && \
    apt-get install -y \
    ros-jazzy-tf-transformations\
    && rm -rf /var/lib/apt/lists/*

# Собираем ros2 workspace
RUN cd /app/ros2_ws && \
    colcon build

# COPY requirements.txt ./
# RUN pip install --no-cache-dir -r requirements.txt

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "run", "controller", "controller"]
