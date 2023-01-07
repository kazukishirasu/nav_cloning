for i in `seq 1 40`
do
    echo "$i"
    # echo "$(($i - $(($i / 2))))"
    roslaunch nav_cloning nav_cloning_sim.launch script:="nav_cloning_test_mode.py" num:="$i"
sleep 10
done
