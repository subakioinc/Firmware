# Simple Cpp App
 * simple_cpp_app.cpp 파일 생성
 * CMakeList.txt 생성
 * simple Cpp App을 빌드 되도록 추가
 * Firmware/boards/px4/fmu-v3/default.cmake
   * 맨 마지막 줄에 simple_cpp_app 추가

 # 확인
 * firmward upload
```console
jeyong@bar:~$ make px4_fmu-v3 upload
```
 * system console
```console
nsh> help
```
   * simple_cpp_app이 보이는지

 * simple_cpp_app 실행
```console
nsh> simple_cpp_app
Hello World!
```