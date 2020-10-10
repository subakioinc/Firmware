#!/bin/bash

set -e

# 인자 번호 설정 
sitl_bin="$1"
debugger="$2"
program="$3"
model="$4"
src_path="$5"
build_path="$6"
# The rest of the arguments are files to copy into the working dir.
# 나머지 인자들은 working 디렉터리에 복사 할 파일들이다

# 들어온 인자 echo로 출력 
echo SITL ARGS

echo sitl_bin: $sitl_bin
echo debugger: $debugger
echo program: $program
echo model: $model
echo src_path: $src_path
echo build_path: $build_path

# working 디렉터리 빌드시 생성 
rootfs="$build_path/tmp/rootfs" # this is the working directory
mkdir -p "$rootfs"

# 사용자 입력을 비활성화 할경우 즉,px4 shell사용 하지 않는 경우 
# To disable user input
if [[ -n "$NO_PXH" ]]; then
	no_pxh=-d
else
	no_pxh=""
fi

# 인자로 들어온 모델명이 있는 경우 
if [ "$model" != none ]; then
	# ps aux -> 모든 프로세스를 보여줌
	# grep -> 패턴을 찾을 때 사용 
		# grep java -> java라는 단어가 포함된 프로세스를 찾음 
		# grep "\-jar ...."  -> 해당 jar 파일이 포함된 프로세스 사녕ㄹ 
	# awk '{ print $2 }' 찾은 파일의 두번째 인자 출력 
	jmavsim_pid=`ps aux | grep java | grep "\-jar jmavsim_run.jar" | awk '{ print $2 }'`
	# jmavsim 프로세스가 생성되어 있는 경우 프로세스 죽임 
	if [ -n "$jmavsim_pid" ]; then
		kill $jmavsim_pid
	fi
fi

# 인자로 설정된 model의 값이 없거나 "none"으로 들어온경우 
if [ "$model" == "" ] || [ "$model" == "none" ]; then
	# model없으니 iris를 기본값으로 설정함 
	echo "empty model, setting iris as default"
	model="iris"
fi

# 들어온 인자의 수가 6개 미만일 경우 ehco로 사용방법 프린트 후 종료 
if [ "$#" -lt 6 ]; then
	echo usage: sitl_run.sh sitl_bin debugger program model src_path build_path
	echo ""
	exit 1
fi

# kill process names that might stil
# be running from last time

# 마지막 까지 실행 될수 있는 프로세스 (gazebo, px, px4_model)등을 kill명령으로 종료
pkill -x gazebo || true
pkill -x px4 || true
pkill -x px4_$model || true

# posix_lldbinit, posix.gdbinit 파일을 working 디렉터리에 복사   --> 두개의 파일의 무슨 파일인지???
cp "$src_path/Tools/posix_lldbinit" "$rootfs/.lldbinit"
cp "$src_path/Tools/posix.gdbinit" "$rootfs/.gdbinit"

# shift는 인자들을 삭제하는 기능 
# 인자로 들어온 이외의 파일들도 working 디렉터리에 복사 
shift 6
for file in "$@"; do
	cp "$file" $rootfs/
done

SIM_PID=0

# speed factor(속도계수)가 시뮬레이션 환경에서 설정되도록 허용
# Allow speed factor to bet set from environment.
if [[ -n "$PX4_SIM_SPEED_FACTOR" ]]; then
	speed_factor=$PX4_SIM_SPEED_FACTOR
else
	speed_factor=1
fi

# 프로그램이 jamvsim 이고, no_sim의 인자가 없는 경우  ????
if [ "$program" == "jmavsim" ] && [ ! -n "$no_sim" ]; then
	# Start Java simulator
	# Java 시뮬레이터를 시작한다 

	# bash scripts?  
	# -r : recursive , 
	# -f : 특정 file 지정 
	# -l : text를 나열한다?????

	"$src_path"/Tools/jmavsim_run.sh -r 250 -f $speed_factor -l &
	SIM_PID=`echo $!`
	# 인자로 들어온 프로그램이 gazebo 이고 no_sim인 경우 
elif [ "$program" == "gazebo" ] && [ ! -n "$no_sim" ]; then
	# -x : 실행파일 명세 
		# command -v gazebo : gazebo가 설치된 path 출력 됨 ("/usr/local/bin/gazebo")
		# gazebo 설치가 된경우 진행 
	if [ -x "$(command -v gazebo)" ]; then
		# -z : string의 길이가 zero
		# DONT_RUN이가 null인 경우 
		if  [[ -z "$DONT_RUN" ]]; then
			# Set the plugin path so Gazebo finds our model and sim
			# Gazebo가 model과 sim을 찾도록 Plugin path 설정한다 
			source "$src_path/Tools/setup_gazebo.bash" "${src_path}" "${build_path}"

			# gzserver로 world 실행 
			gzserver --verbose "${src_path}/Tools/sitl_gazebo/worlds/${model}.world" &
			SIM_PID=`echo $!`

			# HEADLESS : gazebo client(GUI)를 사용하지 않는 경우 
			if [[ -n "$HEADLESS" ]]; then
				echo "not running gazebo gui"
			else
				# gzserver needs to be running to avoid a race. Since the launch
				# is putting it into the background we need to avoid it by backing off
				
				# gzserver와 gzclient가 함께 실행 되는 것을 피해야 한다 
				# gzserver에 필요한 설정들을 먼저 해준다음에 gzclient를 실행햐아 함
				# 그래서 sleep을 걸어 주고 client 실행 함 
				sleep 3
				nice -n 20 gzclient --verbose &
				GUI_PID=`echo $!`
			fi
		fi
	else
		echo "You need to have gazebo simulator installed!"
		exit 1
	fi
fi

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
# 기존에 살아있는 프로세스를 모두 정리를 해야 하므로 실패시 종료 하지마시오
set +e

if [[ ${model} == test_* ]] || [[ ${model} == *_generated ]]; then
	sitl_command="\"$sitl_bin\" $no_pxh \"$src_path\"/ROMFS/px4fmu_test -s \"${src_path}\"/posix-configs/SITL/init/test/${model} -t \"$src_path\"/test_data"
else
	sitl_command="\"$sitl_bin\" $no_pxh \"$src_path\"/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t \"$src_path\"/test_data"
fi

echo SITL COMMAND: $sitl_command

export PX4_SIM_MODEL=${model}

# eval : 인자를 치환 할때 사용 

# DONT_RUN인자가 있는 경우 
if [[ -n "$DONT_RUN" ]]; then
	echo "Not running simulation (\$DONT_RUN is set)."
# lldb / gdb / ddd / valgrind / callgrind / ide 라는 디버거 사용시 
	# dlswkfmf 위에설정한 sitl_command로 인자 치환 
elif [ "$debugger" == "lldb" ]; then
	eval lldb -- $sitl_command
elif [ "$debugger" == "gdb" ]; then
	eval gdb --args $sitl_command
elif [ "$debugger" == "ddd" ]; then
	eval ddd --debugger gdb --args $sitl_command
elif [ "$debugger" == "valgrind" ]; then
	eval valgrind --track-origins=yes --leak-check=full -v $sitl_command
elif [ "$debugger" == "callgrind" ]; then
	eval valgrind --tool=callgrind -v $sitl_command
elif [ "$debugger" == "ide" ]; then
	echo "######################################################################"
	echo
	echo "PX4 simulator not started, use your IDE to start PX4_${model} target."
	echo "Hit enter to quit..."
	echo
	echo "######################################################################"
	read
else
	eval $sitl_command
fi

popd >/dev/null

# 인자로 DONT_RUN 들어온경우 
if [[ -z "$DONT_RUN" ]]; then
	# 해당 시뮬레이션(jmavsim/gazebo)해당 프로르세스 죽임 
	if [ "$program" == "jmavsim" ]; then
		pkill -9 -P $SIM_PID
		kill -9 $SIM_PID
	elif [ "$program" == "gazebo" ]; then
		kill -9 $SIM_PID
		if [[ ! -n "$HEADLESS" ]]; then
			kill -9 $GUI_PID
		fi
	fi
fi
