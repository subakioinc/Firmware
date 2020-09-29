# Docker Container
 * PX4 개발 toolchain을 위한 container
   * NuttX
   * Gazebo Simulation
   * ROS
## 사전 준비
 * Docker 설치
```bash
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
 * root로 실행하지 않기 위해
```bash
# docker group 생성 (may not be required)
sudo groupadd docker
# user를 docker group에 추가
sudo usermod -aG docker $USER
# docker 사용전에 다시 Log in/out
```
## Docker Container 사용
 * PX4 Firmware 받기
```bash
mkdir src
cd src
git clone https://github.com/PX4/Firmware.git
cd Firmware
```

## Helper Script(docker_run.sh)
 * docker_run.sh 스크립트를 이용한 container 사용
 * SITL build
```bash
./Tools/docker_run.sh 'make px4_sitl_default'
```
 * NuttX build
```bash
./Tools/docker_run.sh 'bash'
```

