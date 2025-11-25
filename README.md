# ROS_CUDA (windows)
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
# Nvidia AGX Orin
## 導航
```bash
  # 建置映像並啟動容器
  docker compose up -d
  # 容器內編譯(只需一次)
  docker exec -it navigation /root/entrypoint.sh
  # 進入容器
  docker exec -it navigation bash
```