# simple_subscribe
 * examples/simple_subscribe 폴더 만들기
   * simple_subscribe.cpp 만들기
   * CMakeLists.txt 생성
 * simple_subscribe 모듈 빌드 되도록 추가
   * Firmware/boards/px4/fmu-v3/default.cmake
   * 맨 마지막 줄에 simple_subscribe 추가

# 확인
 * firmward upload
```console
jeyong@bar:~$ make px4_fmu-v3 upload
```
 * system console
```console
nsh> help
```
   * simple_subscribe 보이는지

 * simple_subscribe 실행
```console
nsh> simple_subscribe
출력 내용 확인!!!!
```
