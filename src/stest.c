/////////////////////////////////
// File: stest.c
// Desc: Stage library test program
// Created: 2004.9.15
// Author: Richard Vaughan <vaughan@sfu.ca>
// CVS: $Id: stest.c,v 1.5 2004-09-25 02:15:00 rtv Exp $
// License: GPL
/////////////////////////////////

#include "stage.h"

int main( int argc, char* argv[] )
{ 
  // initialize libstage
  stg_init( argc, argv );

  stg_world_t* world = stg_world_create_from_file( argv[1] );
  
  while( (stg_world_update( world,TRUE )==0) )
    {}
  
  stg_world_destroy( world );
  
  exit( 0 );
}