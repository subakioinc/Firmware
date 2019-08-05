# mypublish
 * mission_subak topic 생성
   * /msg/mission_subak.msg 위치시키기
   * /msg/CMakeLists.txt에 추가하기
 * examples/mymodule_subscribe 폴더 만들기
   * mymodule_subscribe.cpp 만들기
   * CMakeList.txt 생성
 * mypublish 모듈 빌드 되도록 추가
   * Firmware/boards/px4/fmu-v3/default.cmake
   * 맨 마지막 줄에 mymodule_subscribe 추가

# 확인
 * firmward upload
```console
jeyong@bar:~$ make px4_fmu-v3 upload
```
 * system console
```console
nsh> help
```
   * mymodule_subscribe이 보이는지

 * mymodule_subscribe 실행
```console
nsh> mymodule_subscribe start
.....
```
