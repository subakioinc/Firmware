# mypublish
 * mission_subak topic 생성
   * Firmware/msg/mission_subak.msg 생성하기
   * Firmware/msg/CMakeLists.txt에 추가하기
 * examples/mypublish 폴더 만들기
   * mypub_main.cpp 만들기
   * CMakeList.txt 생성
 * mypublish 모듈 빌드 되도록 추가
   * Firmware/boards/px4/fmu-v3/default.cmake
   * 맨 마지막 줄에 mypublish 추가

# 확인
 * firmward upload
```console
jeyong@bar:~$ make px4_fmu-v3 upload
```
 * system console
```console
nsh> help
```
   * mypub이 보이는지

 * mypub 실행
```console
nsh> mypub &
.....
```
