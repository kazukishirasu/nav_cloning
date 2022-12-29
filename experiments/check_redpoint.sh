for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
do
    echo "$i"
    # echo "$(($i - $(($i / 2))))"
    roslaunch nav_cloning nav_cloning_sim.launch num:="$i"
    sleep 15
done
