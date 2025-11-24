# UGV_ROS_CUDA (windows)
## WSL ubuntu 24.04安裝 NVIDIA 相關套件
``` bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

``` bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
# 重新啟動 Docker
sudo systemctl restart docker
```

# 使用cuda映像
``` bash
docker pull nycusystemlab/ros_cuda:amd64
```
## 執行
``` bash
docker run -it --gpus all -e DISPLAY=host.docker.internal:0.0 -v /tmp/.X11-unix:/tmp/.X11-unix --name ros_cuda_container nycusystemlab/ros_cuda:amd64
```
## 進入容器終端
``` bash
docker exec -it ros_cuda_container bash
```
## 啟動/關閉容器
``` bash
docker start ros_cuda_container
docker stop ros_cuda_container
```
# ARM64
## 測試官方映像
``` bash
docker run -it --gpus all -e DISPLAY=host.docker.internal:0.0 -v /tmp/.X11-unix:/tmp/.X11-unix --platform linux/arm64 nvcr.io/nvidia/l4t-base:35.4.1
```
# 製作映像
## 建置 (AGX 本身就是 arm64，不需要 buildx)
``` bash
docker build --network=host -t nycusystemlab/l4t-noetic -f Dockerfile.l4t --push .
```
## 推送
```bash
docker push nycusysytmlab/l4t-noetic
```
## 建置
```bash
docker run -it --rm \
  --network=host \
  --privileged \
  --runtime=nvidia \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v /var/run/dbus:/var/run/dbus \
  -v /run/udev:/run/udev \
  -v /sys/class/gpio:/sys/class/gpio \
  -v /sys/devices:/sys/devices \
  -v /proc/device-tree:/proc/device-tree \
  -v /etc/localtime:/etc/localtime:ro \
  --name l4t-noetic \
  nycusystemlab/l4t-noetic \
  bash

# docker run -it --rm --network=host --privileged --runtime=nvidia -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev:/dev -v /var/run/dbus:/var/run/dbus -v /run/udev:/run/udev -v /sys/class/gpio:/sys/class/gpio -v /sys/devices:/sys/devices -v /proc/device-tree:/proc/device-tree -v /etc/localtime:/etc/localtime:ro nycusystemlab/l4t-noetic bash

```
