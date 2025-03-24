#!/bin/bash
MACHINE=`uname -m`

function create_user_account() {
  local user_name="$1"
  local uid="$2"
  local group_name="$3"
  local gid="$4"
  addgroup --gid "${gid}" "${group_name}"
  useradd ${user_name} -u ${uid} -g ${gid} -m
  echo "%sudo  ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/nopasswd
  usermod -aG sudo "${user_name}"
  usermod -aG video "${user_name}"
  chown -R "${user_name}":"${user_name}" /workspace
}

function setup_user_account_if_not_exist() {
  local user_name="$1"
  local uid="$2"
  local group_name="$3"
  local gid="$4"
  if grep -q "^${user_name}:" /etc/passwd; then
      echo "User ${user_name} already exist. Skip setting user account."
      return
  fi
  create_user_account "$@"
  # echo "127.0.0.1 in-dev-docker" >> /etc/hosts
}

function grant_device_permissions() {
  echo "todo: grant_device_permissions"
}

function setup_core_pattern() {
  if [[ -w /proc/sys/kernel/core_pattern ]]; then
      echo "/workspace/data/core/core_%e.%p" > /proc/sys/kernel/core_pattern
  fi
}

function init_ros2() {
  echo "source /opt/ros/${CUSTOM_ROS}/setup.bash" >> /home/$1/.bashrc
  # echo "source /workspace/scripts/setup.sh" >> /home/$1/.bashrc
  if [ "$MACHINE" == "x86_64" ]; then
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/algorithm/lib:/opt/dds/lib" >> /home/$1/.bashrc
    # echo "export PATH=\$PATH:/opt/third_party/protobuf/bin" >> /home/$1/.bashrc
  else
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/algorithm/lib:/opt/dds/lib" >> /home/$1/.bashrc
  fi
  echo "export AMENT_PREFIX_PATH=\$AMENT_PREFIX_PATH:/opt/algorithm" >> /home/$1/.bashrc
  echo "export PYTHONPATH=\$PYTHONPATH:/opt/algorithm/local/lib/python3.10/dist-packages" >> /home/$1/.bashrc
}

function main() {
  local user_name="$1"
  local uid="$2"
  local group_name="$3"
  local gid="$4"

  if [ "${uid}" != "${gid}" ]; then
    echo "Warning: uid(${uid}) != gid(${gid}) found."
  fi
  if [ "${user_name}" != "${group_name}" ]; then
    echo "Warning: user_name(${user_name}) != group_name(${group_name}) found."
  fi

  setup_user_account_if_not_exist "$@"
  # init_ros2 $user_name
  #grant_device_permissions "${user_name}"
  setup_core_pattern
  # add vtk & opencv & tensorrt lib
  # VERSION=$(lsb_release -rs | sed 's/\.//g')
  VERSION=$(grep VERSION_ID /etc/os-release | cut -d '"' -f 2 | tr -d '.')
  if [[ $VERSION == 2204 ]]; then
    init_ros2 "$user_name"  // TODO: source failed
    source /opt/ros/${CUSTOM_ROS}/setup.bash
  elif [[ $VERSION == 2004 ]]; then
    sudo apt install -y lsb-release > /dev/null 2>&1   # TODO: mv to docker images
  fi
  LOCAL_PATH="/opt/algorithm/lib"
  VTK_PATH="/workspace/third_party/x86/${VERSION}/VTK8/lib"
  OPENCV_PATH="/workspace/third_party/x86/${VERSION}/opencv-4.9/lib"
  TRT_PATH="/workspace/third_party/x86/TensorRT-8.4.1.5.Linux.x86_64-gnu.cuda-11.6.cudnn8.4/TensorRT-8.4.1.5/lib"
  PROTO_PATH="/workspace/third_party/x86/${VERSION}/protobuf-3.8.0/lib"
  CUDA_HOME=/usr/local/cuda
  echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${LOCAL_PATH}:${VTK_PATH}:${OPENCV_PATH}:${TRT_PATH}:${PROTO_PATH}:${CUDA_HOME}/lib64" >> /home/$1/.bashrc
  echo "export PATH=${CUDA_HOME}/bin:$PATH" >> /home/$1/.bashrc
  # install cv2
  python3 -m pip install -q /workspace/third_party/x86/${VERSION}/opencv-4.9/wheel/opencv-4.9.0-py3-none-any.whl
  PYTHON_VERSION="python$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')"
  sudo cp /workspace/third_party/x86/${VERSION}/opencv-4.9/wheel/config*.py /usr/local/lib/${PYTHON_VERSION}/dist-packages/cv2/

  echo "PS1='\[\033[01;32m\]\u@\[\033[01;35m\]\h\[\033[00m\]:\[\033[01;36m\]\w\[\033[00m\]$ '" >> /home/$1/.bashrc
  # vim utf-8
  echo "set encoding=utf-8" >> /home/$1/.vimrc
  echo "set fileencodings=utf-8" >> /home/$1/.vimrc
  # XDG_RUNTIME_DIR
  echo 'export XDG_RUNTIME_DIR=/run/user/$(id -u)' >> /home/$1/.bashrc
}

main "${DOCKER_USER}" "${DOCKER_USER_ID}" "${DOCKER_GRP}" "${DOCKER_GRP_ID}"
