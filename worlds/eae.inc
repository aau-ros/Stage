
# World for energy-aware exploration (eae) with simplified assumptions:
#  - map has no obstacles
#  - robots move from cell to cell
#  - laser can see neighboring cells
# Authors: Micha Rappaport

include "pioneer.inc"

speedup 1
paused 1
quit_time 18000
resolution 1
threads 4

window
(
    size [ 800 600 ]
    center [ 0 0 ]
    rotate [ 0 0 ]
    scale 20
    show_data 1
    show_flags 1
    show_grid 1
)

define charging_bump model
(
    pose [0.240 0 -0.100 0 ]
    size [0.120 0.050 0.100]
    take_watts 1000.0
    color "orange"
    obstacle_return 0
    fiducial(
        range_max 5
        fov 360
        pose [ 0 0 -0.050 0 ]
    )
)

define robot pioneer2dx
(
    ctrl "eae"
    kjoules 10
    kjoules_capacity 10
    localization "gps"
    localization_origin [ 0 0 0 0 ]
    fiducial_return 0
    obstacle_return 0
    charging_bump()
    wifi(
        essid "robotnet"
        model "friis"
        power 30
        sensitivity -80
    )
)

define docking_station model
(
    size [ 0.25 0.25 0.3 ]
    color "purple"

    ranger_return 0
    obstacle_return 0

    kjoules -1  # provides infinite energy
    give_watts 1000
    alwayson 1 # so we give charge without any explicit subscriber
)