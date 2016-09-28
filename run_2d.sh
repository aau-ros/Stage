#!/bin/bash

# function to print help page for this script
function usage(){
    echo -e "Usage: $0 <Options> "
    echo -e "Run the energy-aware exploration simulation in stage."
    echo -e "Make sure not to run multiple instances of this script with the exact same parameters!\n"
    echo -e "Options:"
    echo -e "\t-h, --help\t\tPrint this help."
    echo -e "\t-r, --robots\t\tNumber of robots, max 8, mandatory."
    echo -e "\t-d, --docking-stations\tNumber of docking stations, max 9, mandatory."
    echo -e "\t-s, --strategy\t\tCoordination strategy, mandatory:\n\t\t\t\t0:\tmarked based\n\t\t\t\t1:\tgreedy\n\t\t\t\t2:\toptimal (not yet implemented)"
    echo -e "\t-p, --policy\t\tDocking station selection policy, mandatory:\n\t\t\t\t1:\tclosest\n\t\t\t\t2:\tvacant\n\t\t\t\t3:\topportune\n\t\t\t\t4:\tcurrent\n\t\t\t\t5:\tlonely\n\t\t\t\t6:\tconnected (not yet implemented)\n\t\t\t\t7:\tcombined (not yet implemented)"
    echo -e "\t-m, --map\t\tStart simulation from this map."
    echo
}

# define command line options
opts=$(getopt -o hr:d:s:p:m: --long help,robots:,docking-stations:,strategy:,policy:,map: -n 'run.sh' -- "$@")

# mandatory command line options
mandatory=(-r -d -s -p)

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

# center
dss[1]="docking_station( pose [ 0 0 0 0 ] fiducial_return 1 name \"ds1\" )"

# square 1
# ring 1
dss[2]="docking_station( pose [ 25 0 0 0 ] fiducial_return 2 name \"ds2\" )"
dss[3]="docking_station( pose [ 0 25 0 0 ] fiducial_return 3 name \"ds3\" )"
dss[4]="docking_station( pose [ -25 0 0 0 ] fiducial_return 4 name \"ds4\" )"
dss[5]="docking_station( pose [ 0 -25 0 0 ] fiducial_return 5 name \"ds5\" )"
#ring 2
dss[6]="docking_station( pose [ 25 25 0 0 ] fiducial_return 6 name \"ds6\" )"
dss[7]="docking_station( pose [ -25 25 0 0 ] fiducial_return 7 name \"ds7\" )"
dss[8]="docking_station( pose [ -25 -25 0 0 ] fiducial_return 8 name \"ds8\" )"
dss[9]="docking_station( pose [ 25 -25 0 0 ] fiducial_return 9 name \"ds9\" )"

# square 2
# ring 1
dss[10]="docking_station( pose [ 50 0 0 0 ] fiducial_return 10 name \"ds10\" )"
dss[11]="docking_station( pose [ 0 50 0 0 ] fiducial_return 11 name \"ds11\" )"
dss[12]="docking_station( pose [ -50 0 0 0 ] fiducial_return 12 name \"ds12\" )"
dss[13]="docking_station( pose [ 0 -50 0 0 ] fiducial_return 13 name \"ds13\" )"
# ring 2
dss[14]="docking_station( pose [ 50 25 0 0 ] fiducial_return 14 name \"ds14\" )"
dss[15]="docking_station( pose [ 25 50 0 0 ] fiducial_return 15 name \"ds15\" )"
dss[16]="docking_station( pose [ -25 50 0 0 ] fiducial_return 16 name \"ds16\" )"
dss[17]="docking_station( pose [ -50 25 0 0 ] fiducial_return 17 name \"ds17\" )"
dss[18]="docking_station( pose [ -50 -25 0 0 ] fiducial_return 18 name \"ds18\" )"
dss[19]="docking_station( pose [ -25 -50 0 0 ] fiducial_return 19 name \"ds19\" )"
dss[20]="docking_station( pose [ 25 -50 0 0 ] fiducial_return 20 name \"ds20\" )"
dss[21]="docking_station( pose [ 50 -25 0 0 ] fiducial_return 21 name \"ds21\" )"
# ring 3
dss[22]="docking_station( pose [ 50 50 0 0 ] fiducial_return 22 name \"ds22\" )"
dss[23]="docking_station( pose [ -50 50 0 0 ] fiducial_return 23 name \"ds23\" )"
dss[24]="docking_station( pose [ -50 -50 0 0 ] fiducial_return 24 name \"ds24\" )"
dss[25]="docking_station( pose [ 50 -50 0 0 ] fiducial_return 25 name \"ds25\" )"

# square 3/4
# ring 1
dss[26]="docking_station( pose [ 75 0 0 0 ] fiducial_return 26 name \"ds26\" )"
dss[27]="docking_station( pose [ 0 75 0 0 ] fiducial_return 27 name \"ds27\" )"
dss[28]="docking_station( pose [ -75 0 0 0 ] fiducial_return 28 name \"ds28\" )"
dss[29]="docking_station( pose [ 0 -75 0 0 ] fiducial_return 29 name \"ds29\" )"
# ring 2
dss[30]="docking_station( pose [ 75 25 0 0 ] fiducial_return 30 name \"ds30\" )"
dss[31]="docking_station( pose [ 25 75 0 0 ] fiducial_return 31 name \"ds31\" )"
dss[32]="docking_station( pose [ -25 75 0 0 ] fiducial_return 32 name \"ds32\" )"
dss[33]="docking_station( pose [ -75 25 0 0 ] fiducial_return 33 name \"ds33\" )"
dss[34]="docking_station( pose [ -75 -25 0 0 ] fiducial_return 34 name \"ds34\" )"
dss[35]="docking_station( pose [ -25 -75 0 0 ] fiducial_return 35 name \"ds35\" )"
dss[36]="docking_station( pose [ 25 -75 0 0 ] fiducial_return 36 name \"ds36\" )"
dss[37]="docking_station( pose [ 75 -25 0 0 ] fiducial_return 37 name \"ds37\" )"
# ring 3
dss[38]="docking_station( pose [ 75 50 0 0 ] fiducial_return 38 name \"ds38\" )"
dss[39]="docking_station( pose [ 50 75 0 0 ] fiducial_return 39 name \"ds39\" )"
dss[40]="docking_station( pose [ -50 75 0 0 ] fiducial_return 40 name \"ds40\" )"
dss[41]="docking_station( pose [ -75 50 0 0 ] fiducial_return 41 name \"ds41\" )"
dss[42]="docking_station( pose [ -75 -50 0 0 ] fiducial_return 42 name \"ds42\" )"
dss[43]="docking_station( pose [ -50 -75 0 0 ] fiducial_return 43 name \"ds43\" )"
dss[44]="docking_station( pose [ 50 -75 0 0 ] fiducial_return 44 name \"ds44\" )"
dss[45]="docking_station( pose [ 75 -50 0 0 ] fiducial_return 45 name \"ds45\" )"
# ring 4 (square 4 ring 1)
dss[46]="docking_station( pose [ 100 0 0 0 ] fiducial_return 46 name \"ds46\" )"
dss[47]="docking_station( pose [ 0 100 0 0 ] fiducial_return 47 name \"ds47\" )"
dss[48]="docking_station( pose [ -100 0 0 0 ] fiducial_return 48 name \"ds48\" )"
dss[49]="docking_station( pose [ 0 -100 0 0 ] fiducial_return 49 name \"ds49\" )"
# ring 5 (square 4 ring 2)
dss[50]="docking_station( pose [ 100 25 0 0 ] fiducial_return 50 name \"ds50\" )"
dss[51]="docking_station( pose [ 25 100 0 0 ] fiducial_return 51 name \"ds51\" )"
dss[52]="docking_station( pose [ -25 100 0 0 ] fiducial_return 52 name \"ds52\" )"
dss[53]="docking_station( pose [ -100 25 0 0 ] fiducial_return 53 name \"ds53\" )"
dss[54]="docking_station( pose [ -100 -25 0 0 ] fiducial_return 54 name \"ds54\" )"
dss[55]="docking_station( pose [ -25 -100 0 0 ] fiducial_return 55 name \"ds55\" )"
dss[56]="docking_station( pose [ 25 -100 0 0 ] fiducial_return 56 name \"ds56\" )"
dss[57]="docking_station( pose [ 100 -25 0 0 ] fiducial_return 57 name \"ds57\" )"
# ring 6
dss[58]="docking_station( pose [ 75 75 0 0 ] fiducial_return 58 name \"ds58\" )"
dss[59]="docking_station( pose [ -75 75 0 0 ] fiducial_return 59 name \"ds59\" )"
dss[60]="docking_station( pose [ -75 -75 0 0 ] fiducial_return 60 name \"ds60\" )"
dss[61]="docking_station( pose [ 75 -75 0 0 ] fiducial_return 61 name \"ds61\" )"

# square 4
# ring 3
dss[62]="docking_station( pose [ 100 50 0 0 ] fiducial_return 62 name \"ds62\" )"
dss[63]="docking_station( pose [ 50 100 0 0 ] fiducial_return 63 name \"ds63\" )"
dss[64]="docking_station( pose [ -50 100 0 0 ] fiducial_return 64 name \"ds64\" )"
dss[65]="docking_station( pose [ -100 50 0 0 ] fiducial_return 65 name \"ds65\" )"
dss[66]="docking_station( pose [ -100 -50 0 0 ] fiducial_return 66 name \"ds66\" )"
dss[67]="docking_station( pose [ -50 -100 0 0 ] fiducial_return 67 name \"ds67\" )"
dss[68]="docking_station( pose [ 50 -100 0 0 ] fiducial_return 68 name \"ds68\" )"
dss[69]="docking_station( pose [ 100 -50 0 0 ] fiducial_return 69 name \"ds69\" )"
# ring 4
dss[70]="docking_station( pose [ 100 75 0 0 ] fiducial_return 70 name \"ds70\" )"
dss[71]="docking_station( pose [ 75 100 0 0 ] fiducial_return 71 name \"ds71\" )"
dss[72]="docking_station( pose [ -75 100 0 0 ] fiducial_return 72 name \"ds72\" )"
dss[73]="docking_station( pose [ -100 75 0 0 ] fiducial_return 73 name \"ds73\" )"
dss[74]="docking_station( pose [ -100 -75 0 0 ] fiducial_return 74 name \"ds74\" )"
dss[75]="docking_station( pose [ -75 -100 0 0 ] fiducial_return 75 name \"ds75\" )"
dss[76]="docking_station( pose [ 75 -100 0 0 ] fiducial_return 76 name \"ds76\" )"
dss[77]="docking_station( pose [ 100 -75 0 0 ] fiducial_return 77 name \"ds77\" )"
# ring 5
dss[78]="docking_station( pose [ 100 100 0 0 ] fiducial_return 78 name \"ds78\" )"
dss[79]="docking_station( pose [ -100 100 0 0 ] fiducial_return 79 name \"ds79\" )"
dss[80]="docking_station( pose [ -100 -100 0 0 ] fiducial_return 80 name \"ds80\" )"
dss[81]="docking_station( pose [ 100 -100 0 0 ] fiducial_return 81 name \"ds81\" )"

# define number of maps
maps=50

# repeat simulation for each map
for (( i=${map}; i<=${maps}; i++ ))
do
    # create content for world file
    world="include \"eae.inc\"\n\n"

    # add settings to world file
    world+="robots $robots\n"
    world+="docking_stations $ds\n"
    world+="strategy $strategy\n"
    world+="policy $policy\n\n"

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
    world+="\tpose [0 0 0 0]\n"
    world+="\tsize [250 250 1]\n"
    world+="\tbitmap \"bitmaps/lines2d_${i}.png\"\n"
    world+=")\n"

    # world file path
    file="worlds/eae_${robots}_${ds}_${strategy}_${policy}.world"

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

