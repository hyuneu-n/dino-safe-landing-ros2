#!/usr/bin/env bash
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe GAZEBO_MASTER_URI=http://localhost:11346 ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh 2>/dev/null || true
( gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$HOME/gz_worlds/residential_delivery.world" > /tmp/t_gz.log 2>&1 & )
sleep 16
( source "$HOME/venv_ros/bin/activate"; cd "$HOME/safe_landing"; python3 landing_detector.py > /tmp/t_det.log 2>&1 & )
( cd "$HOME/safe_landing"; python3 mission_controller.py > /tmp/t_mc.log 2>&1 & )
sleep 66
echo "=== topics ==="
ros2 topic list 2>/dev/null | grep -E 'landing|mission|drone|chase' | sort
echo "=== /mission/status ==="
timeout 3 ros2 topic echo --once /mission/status 2>/dev/null
echo "=== /landing/target ==="
timeout 6 ros2 topic echo --once /landing/target 2>/dev/null || echo "(no target)"
echo "=== detector log (tail) ==="
tail -n 10 /tmp/t_det.log
echo "=== mission_controller log ==="
grep -iE 'Traceback|rror|이륙|협곡|스캔|착륙|재평가|장애물|보행자|고정' /tmp/t_mc.log | tail -n 22
echo "(mc log lines: $(wc -l < /tmp/t_mc.log))"
echo "=== gzserver log: errors/warnings ==="
grep -iE 'error|warn|err\]|cannot|fail|unable' /tmp/t_gz.log | head -n 15
echo "(gz log lines: $(wc -l < /tmp/t_gz.log))"
pkill -f 'localhost:11346' 2>/dev/null
pkill -f landing_detector 2>/dev/null
pkill -f mission_controller 2>/dev/null
pkill -f 'gzserver.*residential' 2>/dev/null
echo DONE
