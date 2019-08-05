# Simple C App
 * simple_c_app.c 파일 생성
 * CMakeLists.txt 생성
 * simple c App을 빌드 되도록 추가
 * Firmware/boards/px4/fmu-v3/default.cmake
   * 맨 마지막 줄에 simple_c_app 추가
# 확인
 * firmward upload
```console
jeyong@bar:~$ make px4_fmu-v3 upload
```
 * system console
```console
nsh> help
```
   * simple_c_app이 보이는지

 * simple_c_app 실행
```console
nsh> simple_c_app
Hello World!
```
