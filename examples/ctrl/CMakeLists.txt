SET( PLUGINS
  fasr
  sink
  source
  wander
  pioneer_flocking
  rasterize
  lasernoise
  dynamic
  wander_wifi
  idle
)

# create a library module for each plugin and link libstage to each

foreach( PLUGIN ${PLUGINS} )
  ADD_LIBRARY( ${PLUGIN} MODULE ${PLUGIN}.cc )
endforeach( PLUGIN )

ADD_LIBRARY( fasr2 MODULE fasr2.cc astar/findpath.cpp )

#ADD_LIBRARY( eae/eae MODULE eae/eae_coordination.cc eae/eae_ds.cc eae/eae_graph.cc eae/eae_gridmap.cc eae/eae_logoutput.cc eae/eae_robot.cc eae/eae_wifimessage.cc astar/findpath.cpp )
 
#ADD_LIBRARY( eae_frevo_simple/eae_frevo_simple MODULE eae_frevo_simple/eae_frevo_simple_gridmap.cc eae_frevo_simple/eae_frevo_simple_logoutput.cc eae_frevo_simple/eae_frevo_simple_robot.cc )

# add extras to the list of plugins
SET( PLUGINS ${PLUGINS} fasr2 )

set_source_files_properties( ${PLUGINS} PROPERTIES
  COMPILE_FLAGS "${FLTK_CFLAGS}"
)

foreach( PLUGIN ${PLUGINS} )
  TARGET_LINK_LIBRARIES( ${PLUGIN} stage ${OPENGL_LIBRARIES} )
endforeach( PLUGIN )

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( ${PLUGINS} PROPERTIES PREFIX "" )

# install in <prefix>/lib
install( TARGETS ${PLUGINS} DESTINATION ${PROJECT_PLUGIN_DIR})

