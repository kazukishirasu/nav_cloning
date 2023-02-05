for i in `seq 3 10`
do
    echo "$i"
    # echo "$(($i - $(($i / 2))))"
    roslaunch nav_cloning nav_cloning_sim.launch script:="nav_cloning_testonly.py" num:="$i"
sleep 10
done
