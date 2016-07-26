# Desc: Definition of world, robots, and docking stations for energy-aware exploration (eae) with multiple robots.
# Author: Micha Rappaport
# Date: 2016-07-21

# include device definitions for pioneer robots
include "pioneer.inc"
include "sick.inc"
include "map.inc"

# speed of simulation relative to real time
speedup 1

# start simulation in paused mode (gui only)
paused 1

# force simulation to quit after quit_time seconds
quit_time 18000

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
    show_grid 1
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
    obstacle_return 0

    # fiducial sensor
    fiducial(
        # position relative to charging connector
        pose [ 0 0 -0.050 0 ]

        # range of sensor
        range_max 20
        range_max_id 20

        # field of view of sensor
        fov 360
    )
)

# definition of a robot
define robot pioneer2dx_front_sonar
(
    # the controller to use (see ../examples/ctrl)
    ctrl "eae"

    # mass in kg
    mass 9.0

    # initial battery charge in kJ
    kjoules 5

    # capacity of battery in kJ
    kjoules_capacity 5

    # localization method (can be gps or odom)
    localization "gps"

    # origin of the robot's coordinate system
    localization_origin [ 0 0 0 0 ]

    # cannot be detected by fiducial sensors
    fiducial_return 0

    # can collide with other obstacles
    obstacle_return 1

    # charging connector
    charging_bump()

    # wifi interface (see ../libstage/model_wifi.cc)
    wifi(
        essid "robotnet"
        model "friis"
        power 30
        sensitivity -80
    )

    # lidar
    sickbase(
        sensor(
            samples 135 # angular resolution
            range [0 20] # range in meters (check also ../examples/ctrl/eae/eae.hh)
            fov 270 # field of view (check also ../examples/ctrl/eae/eae.hh)
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
    give_watts 1000

    # provide power also without any explicit subscriber
    alwayson 1
)