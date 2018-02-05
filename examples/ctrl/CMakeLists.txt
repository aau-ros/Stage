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
)

# create a library module for each plugin and link libstage to each

foreach( PLUGIN ${PLUGINS} )
  ADD_LIBRARY( ${PLUGIN} MODULE ${PLUGIN}.cc )
endforeach( PLUGIN )

ADD_LIBRARY( fasr2 MODULE fasr2.cc astar/findpath.cpp )

ADD_LIBRARY( eae MODULE eae/eae.cc eae/eae_coordination.cc eae/eae_ds.cc eae/eae_graph.cc eae/eae_gridmap.cc eae/eae_logoutput.cc eae/eae_robot.cc eae/eae_wifimessage.cc astar/findpath.cpp )

ADD_LIBRARY( swarm MODULE swarm/swarm.cc swarm/swarm_graph.cc swarm/swarm_gridmap.cc swarm/swarm_logoutput.cc swarm/swarm_robot.cc astar/findpath.cpp )
 
# add extras to the list of plugins
SET( PLUGINS ${PLUGINS} fasr2 )
SET( PLUGINS ${PLUGINS} eae )
SET( PLUGINS ${PLUGINS} swarm )

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

