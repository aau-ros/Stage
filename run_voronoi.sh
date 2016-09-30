#!/bin/bash

### settings ###

# number of maps
maps=50

# size of square map (length of one side in pixels)
size=500

### settings ###

# function to print help page for this script
function usage(){
    echo -e "Usage: $0 <Options> "
    echo -e "Run the energy-aware exploration simulation in stage."
    echo -e "Make sure not to run multiple instances of this script with the exact same parameters!\n"
    echo -e "Options:"
    echo -e "\t-h, --help\t\tPrint this help."
    echo -e "\t-r, --robots\t\tNumber of robots, mandatory."
    echo -e "\t-d, --docking-stations\tNumber of docking stations, mandatory, at least 3."
    echo -e "\t-s, --strategy\t\tCoordination strategy, mandatory:\n\t\t\t\t0:\tmarked based\n\t\t\t\t1:\tgreedy\n\t\t\t\t2:\toptimal (not yet implemented)"
    echo -e "\t-p, --policy\t\tDocking station selection policy, mandatory:\n\t\t\t\t1:\tclosest\n\t\t\t\t2:\tvacant\n\t\t\t\t3:\topportune\n\t\t\t\t4:\tcurrent\n\t\t\t\t5:\tlonely\n\t\t\t\t6:\tconnected (not yet implemented)\n\t\t\t\t7:\tcombined (not yet implemented)"
    echo -e "\t-c, --connectivity\tConnectivity of rooms, mandatory, 0 (all rooms closed) <= connectivity <= 1 (no walls)."
    echo -e "\t-m, --map\t\tStart simulation from this map."
    echo
}

# define command line options
opts=$(getopt -o hr:d:s:p:c:m: --long help,robots:,docking-stations:,strategy:,policy:,connectivity:,map: -n 'run.sh' -- "$@")

# mandatory command line options
mandatory=(-r -d -s -p -c)

# print help for unknown option
if [ $? != 0 ]
    then usage
    exit 1
fi

# assign options to positional parameters
eval set -- "$opts"

# default starting map
map=1

# loop through all options
while true ; do
    # mark option as given
    for ((i = 0 ; i < ${#mandatory[@]} ; i++ )); do
        [[ $1 == ${mandatory[$i]} ]] && mandatory[$i]="-"
    done

    # extract option and its argument into a variable
    case "$1" in
        -r|--robots)
            robots=$2 ; shift 2 ;;
        -d|--docking-stations)
            ds=$2 ; shift 2 ;;
        -s|--strategy)
            strategy=$2 ; shift 2 ;;
        -p|--policy)
            policy=$2 ; shift 2 ;;
        -c|--connectivity)
            con=$2 ; shift 2 ;;
        -m|--map)
            map=$2 ; shift 2 ;;
        --) shift ; break ;;
        *) usage; exit 1 ;;
    esac
done

# check if all mandatory options where given
for ((i = 0 ; i < ${#mandatory[@]} ; i++ )); do
  if [[ ${mandatory[$i]} != '-' ]]; then
    echo "Option ${mandatory[$i]} was not given."
    usage
    exit 1
  fi
done

# repeat simulation for each map
for (( i=${map}; i<=${maps}; i++ ))
do
    # use python to create random distribution of docking stations
    # this creates worlds/robots.inc, worlds/dss.inc, and worlds/bitmaps/voronoi.png
    python voronoi.py ${i} ${size} ${ds} ${robots} ${con}

    # create content for world file
    world="include \"eae.inc\"\n\n"

    # add settings to world file
    world+="robots $robots\n"
    world+="docking_stations $ds\n"
    world+="strategy $strategy\n"
    world+="policy $policy\n"
    world+="connectivity $con\n"
    world+="run $i\n\n"

    # add robots to world file
    world+="include \"eae_robots.inc\"\n\n"

    # add docking stations to world file
    world+="include \"eae_dss.inc\"\n\n"

    # add floorplan to world file
    world+="floorplan\n"
    world+="(\n"
    world+="\tpose [0 0 0 0]\n"
    world+="\tsize [${size} ${size} 1]\n"
    world+="\tbitmap \"bitmaps/voronoi.png\"\n"
    world+=")\n"

    # world file path
    file="worlds/eae_${robots}_${ds}_${strategy}_${policy}_${con}.world"

    # write contents to world file
    echo -e ${world} > $file

    # output progress
    echo
    echo
    echo "--------------"
    echo " RUN ${i} OF ${maps}"
    echo "--------------"
    echo

    # run stage
    stage -g $file
done

