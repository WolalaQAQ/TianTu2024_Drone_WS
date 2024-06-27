# 使用官方的Ubuntu 18.04基础镜像
FROM ubuntu:18.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive

# 安装基本依赖和工具
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 添加ROS源并安装ROS Melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -sSL 'http://packages.ros.org/ros.key' | apt-key add - \
    && apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# 初始化rosdep
RUN rosdep init && rosdep update

# 设置ROS环境变量
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# 安装一些常用的ROS工具和依赖
RUN apt-get update && apt-get install -y \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# 创建和初始化Catkin工作空间
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_init_workspace /root/catkin_ws/src"

# 编译工作空间（如果有需要预编译的包，可以在此添加）
# RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# 设置工作目录为catkin_ws
WORKDIR /root/catkin_ws

# 复制你的ROS包到工作空间（如果有的话）
# COPY ./your_ros_package /root/catkin_ws/src/your_ros_package

# 安装任何额外的系统依赖或Python包
# RUN apt-get update && apt-get install -y <additional-packages>
# RUN pip install <python-packages>

# 默认启动bash
CMD ["bash"]
