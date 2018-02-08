# Desc: Definition of world, robots, and docking stations for energy-aware swarm exploration (swarm) with multiple robots.
# Author: Micha Rappaport
# Date: 2018-02-01

# include device definitions for pioneer robots
include "pioneer.inc"
include "sick.inc"
include "map.inc"

# start simulation in paused mode (gui only)
paused 1

# force simulation to quit after quit_time simulated seconds
quit_time 600

# resolution of map in meters / cell
resolution 1

# set up the gui window
window
(
    # window size
    size [ 800 600 ]

    # coordinates at center of window
    center [ 0 0 ]

    # rotation of view around x and y axis in degree
    rotate [ 0 0 ]

    # window zoom as ratio of world to pixel coordinates
    scale 20

    # show simulation data like way points
    show_data 1

    # show grid in the background
    show_grid 0
)

# definition of the charging connector of a robot
define charging_bump model
(
    # position relative to robot
    pose [0.240 0 -0.100 0 ]

    # size
    size [0.120 0.050 0.100]

    # color in the gui
    color "orange"

    # cannot collide with other obstacles
    obstacle_return 1

    # fiducial sensor
    fiducial(
        # position relative to charging connector
        pose [ 0 0 -0.050 0 ]

        # range of sensor
        range_max 9
        range_max_id 9

        # field of view of sensor
        fov 360
    )
)

# definition of a robot
define robot pioneer3dx_no_sonar
(
    # the controller to use (see ../examples/ctrl)
    ctrl "swarm"

    # mass in kg
    mass 9.0

    # initial battery charge in kJ
    kjoules 6

    # capacity of battery in kJ
    kjoules_capacity 6

    # localization method (can be gps or odom)
    localization "gps"

    # origin of the robot's coordinate system
    localization_origin [ 0 0 0 0 ]

    # cannot be detected by fiducial sensors
    fiducial_return 0

    # cannot collide with other obstacles
    obstacle_return 1

    # charging connector
    charging_bump()

    # wifi interface (see ../libstage/model_wifi.cc)
    wifi(
        essid "robotnet"
        model "simple"
        range 20
    )

    # lidar
    sickbase(
        sensor(
            samples 360 # angular resolution
            range [0 10] # range in meters
            fov 360 # field of view
        )
        color "blue"
        ranger_return 1
        watts 8
    )
)

# definition of a docking station
define docking_station model
(
    # size
    size [ 0.25 0.25 0.3 ]

    # color in the gui
    color "purple"

    # cannot be detected by a laser range finder
    ranger_return -1

    # cannot collide with other obstacles
    obstacle_return 0

    # amount of stored energy (-1 is infinite)
    kjoules -1

    # power output in W
    give_watts 250

    # provide power also without any explicit subscriber
    alwayson 1
)
