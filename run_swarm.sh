#!/bin/bash

# function to print help page for this script
function usage(){
    echo -e "Usage: $0 <Options> "
    echo -e "Run the energy-aware exploration simulation in stage."
    echo -e "Make sure not to run multiple instances of this script with the exact same parameters!\n"
    echo -e "Options:"
    echo -e "\t-h, --help\t\tPrint this help."
    echo -e "\t-p, --install-prefix\tInstall prefix, specifies the path for Stage installation, mandatory.\n\t\t\t\tMake sure the current user has write access. The following directories will be used within this path:\n\t\t\t\t<prefix>/bin: Executables, including the 'stage' program\n\t\t\t\t<prefix>/include: Stage header file\n\t\t\t\t<prefix>/lib: Compiled libraries, including libstage\n\t\t\t\t<prefix>/log: Stage log files\n\t\t\t\t<prefix>/share: Data resources, such as images or world files"
    echo -e "\t-s, --simulation\tUnique identifier for this simulation."
    echo -e "\t-r, --robots\t\tNumber of robots, max 8, mandatory."
    echo -e "\t-d, --docking-stations\tNumber of docking stations, max 9, mandatory."
    echo -e "\t-m, --map\t\tStart simulation from this map."
    echo -e "\t-g, --gui\t\tRun the simulation with graphical user interface."
    echo
}

# define command line options
opts=$(getopt -o hp:s:r:d:m:g --long help,install-prefix:,simulation:,robots:,docking-stations:,map:,gui -n 'run_swarm.sh' -- "$@")

# mandatory command line options
mandatory=(-p -s -r -d)

# print help for unknown option
if [ $? != 0 ]
    then usage
    exit 1
fi

# assign options to positional parameters
eval set -- "$opts"

# default starting map
map=1

# default no gui
gui=false

# loop through all options
while true ; do
    # mark option as given
    for ((i = 0 ; i < ${#mandatory[@]} ; i++ )); do
        [[ $1 == ${mandatory[$i]} ]] && mandatory[$i]="-"
    done

    # extract option and its argument into a variable
    case "$1" in
        -p|--install-prefix)
            prefix=$2 ; shift 2 ;;
        -s|--simulation)
            simulation=$2 ; shift 2 ;;
        -r|--robots)
            robots=$2 ; shift 2 ;;
        -d|--docking-stations)
            ds=$2 ; shift 2 ;;
        -m|--map)
            map=$2 ; shift 2 ;;
        -g|--gui)
            gui=true; shift ;;
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

# setup stage include paths
export LD_LIBRARY_PATH=${prefix}/lib
export STAGEPATH=${prefix}/lib
export CMAKE_INCLUDE_PATH=/opt/local/include
export CMAKE_LIBRARY_PATH=/opt/local/lib

# create log folder
mkdir ${prefix}/log

# compile swarm controller
make install

# define robots
robots[1]="robot( pose [ 1 0 0 0 ] )"
robots[2]="robot( pose [ -1 0 0 180 ] )"
robots[3]="robot( pose [ 0 1 0 90 ] )"
robots[4]="robot( pose [ 0 -1 0 270 ] )"
robots[5]="robot( pose [ 1 1 0 45 ] )"
robots[6]="robot( pose [ 1 -1 0 315 ] )"
robots[7]="robot( pose [ -1 1 0 135 ] )"
robots[8]="robot( pose [ -1 -1 0 225 ] )"

# define docking stations
dss[1]="docking_station( pose [ 0 0 0 0 ] fiducial_return 1 name \"ds1\" )"
dss[2]="docking_station( pose [ 25 0 0 0 ] fiducial_return 2 name \"ds2\" )"
dss[3]="docking_station( pose [ 50 0 0 0 ] fiducial_return 3 name \"ds3\" )"
dss[4]="docking_station( pose [ 75 0 0 0 ] fiducial_return 4 name \"ds4\" )"
dss[5]="docking_station( pose [ 100 0 0 0 ] fiducial_return 5 name \"ds5\" )"
dss[6]="docking_station( pose [ 125 0 0 0 ] fiducial_return 6 name \"ds6\" )"
dss[7]="docking_station( pose [ 150 0 0 0 ] fiducial_return 7 name \"ds7\" )"
dss[8]="docking_station( pose [ 175 0 0 0 ] fiducial_return 8 name \"ds8\" )"
dss[9]="docking_station( pose [ 200 0 0 0 ] fiducial_return 9 name \"ds9\" )"

# define number of maps
maps=1

# repeat simulation for each map
for (( i=${map}; i<=${maps}; i++ ))
do
    # create content for world file
    world="include \"swarm.inc\"\n\n"

    # add settings to world file
    world+="simulation \"$simulation\"\n"
    world+="robots $robots\n"
    world+="docking_stations $ds\n"
    world+="\n"

    # add robots to world file
    for (( j=1; j<=${robots}; j++ ))
    do
        world+="${robots[$j]}\n"
    done
    world+="\n"

    # add docking stations to world file
    for (( j=1; j<=${ds}; j++ ))
    do
        world+="${dss[$j]}\n"
    done
    world+="\n"

    # add floorplan to world file
    world+="floorplan\n"
    world+="(\n"
    world+="\tpose [100 0 0 0]\n"
    world+="\tsize [250 50 1]\n"
    world+="\tbitmap \"bitmaps/lines_${i}.png\"\n"
    world+="\tobstacle_return 1\n"
    world+=")\n"

    # world file path
    file="${prefix}/share/stage/worlds/swarm_${robots}_${ds}_${simulation}.world"

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
    if [ ${gui} = true ]
    then
        ${prefix}/bin/stage $file
    else
        ${prefix}/bin/stage -g $file
    fi
done

