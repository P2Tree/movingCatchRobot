^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package costmap_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.0 (2016-05-20)
-------------------
* Reordered initializer list to match order of declarations.
  This avoids compiler warning with some compilers.
* Made update map threadsafe
  This is necessary for some plugins (e.g. VoxelLayer) that implement a
  thread unsafe updateBounds() function.
* Fix bug with resetting static layer
  If we don't have a new topic, consider our old data as if it were new.
* fix resource locations to fix tests
* Increase time-limit on failing test
* Merge pull request `#388 <https://github.com/ros-planning/navigation/issues/388>`_ from yujinrobot/jade_inflation_ghost_fix
  No more ghosts in the inflation layer
* Fixes the dynamic reconfigure segfault
  Doing a dynamic reconfigure of the inflation radius recreates
  the cached cost values without first locking a mutex, which causes
  a segfault. This breaks the reconfigure of inflation parameters into
  a separate function and adds a mutex lock.
* Merge pull request `#415 <https://github.com/ros-planning/navigation/issues/415>`_ from alexhenning/jade-fix-multiple-static-layers
  Fixes an issue with having multiple static layers
* Fixes an issue with having multiple static layers
  If you have a static layer in both the local and global costmaps that
  use the same map topic, there is a race condition that can cause the
  static layer to get stuck after printing `Requesting map....`. This race
  condition seems to be due to the call to shutdown in deactivate and how
  the NodeHandle handles multiple subscribers under the hood.
  This issue appears to happen about 1 in 1000 times in the setup I was
  testing. This fix has never failed in over 1000000 tests. Instead of
  calling activate and deactivate, the publisher is only recreated if the
  topic has changed. Otherwise, it reuses the old setup.
* fix related to issue `#408 <https://github.com/ros-planning/navigation/issues/408>`_ - With Rolling Window on, costmap_2d not properly updating bounds and costs in the static layer
* No more ghosts in the inflation layer
  Previous bounds would fit the sensor measurements, and the inflation layer would clear
  out to these, but leave 'ghosts' behind. These ghosts are from two sources - 1) the
  inflation radius and 2) whole obstacles left behind as the robot has moved from the last point.
  The modifications here remember the last bounds and set the new bounds so that a box at least
  large enough to incorporate the old bounds plus the inflation radius is generated.
* Contributors: Alex Henning, Daniel Stonier, Levon Avagyan, Michael Ferguson, palmieri

1.13.1 (2015-10-29)
-------------------
* Remove excessive canTransform spam.
* Fix for `#382 <https://github.com/ros-planning/navigation/issues/382>`_
* Republish costmap if origin changes
* Remove Footprint Layer
* Remove extra sign definition and use proper one when padding footprint
* fix plugin warnings on throw, closes `#205 <https://github.com/ros-planning/navigation/issues/205>`_
* initialize publisher variables
* Look for robot_radius when footprint is not set. `#206 <https://github.com/ros-planning/navigation/issues/206>`_
* Add a first_map_only parameter so we keep reusing the first received static map
* Merge pull request `#331 <https://github.com/ros-planning/navigation/issues/331>`_ from mikeferguson/static_layer_any_frame
* support rolling static map in any frame
* fix destructor of Costmap2D
* proper locking during costmap update
* do not resize static map when rolling
* Static layer works with rolling window now
* Contributors: Daniel Stonier, David Lu, Jihoon Lee, Michael Ferguson, Rein Appeldoorn, commaster90

1.13.0 (2015-03-17)
-------------------
* fixed issue with voxel_layer and obstacle_layer both deleting the same dynamic_reconfigure::Server and causing segfaults
* Fixing various memory freeing operations
* static_layer: Fix indexing error in OccupancyGridUpdate callback function.
* Contributors: Alex Bencz, David V. Lu!!, James Servos, Julse, Kaijen Hsiao

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------
* Add ARCHIVE_DESTINATION for static builds
* Contributors: Gary Servin

1.11.14 (2014-12-05)
--------------------
* added waitForTransform to bufferCloud to solve extrapolation into the future exception
* deallocate costmap_ before reallocating
* prevent div by zero in raytraceLine
* only prefix sensor_frame when it's not empty
* tf_prefix support in obstacle_layer
* remove undefined function updateUsingPlugins
* remove unused cell_data.h
* numerous style fixes
* Contributors: Andrzej Pronobis, David Lu, Jeremie Deray, Mani Monajjemi, Michael Ferguson, enriquefernandez

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------
* costmap_2d: export library layers
* Merge pull request `#198 <https://github.com/ros-planning/navigation/issues/198>`_ from kmhallen/hydro-devel
  Fixed costmap_2d clearing from service /move_base/clear_costmaps
* Costmap Layer comments
* Add destructors for all of the layers to remove the dynamic parameter clients
* Add method for removing static observations (for testing)
* Move testing_helper
* Initial Clearing Costmap parameter change
* Fixed costmap_2d clearing from service /move_base/clear_costmaps
* Contributors: David Lu!!, Kevin Hallenbeck, Michael Ferguson

1.11.11 (2014-07-23)
--------------------
* removes trailing spaces and empty lines
* Contributors: Enrique Fernández Perdomo

1.11.10 (2014-06-25)
--------------------
* Remove unnecessary colons
* Remove unused robot_radius parameter from dynamic_reconfigure
* Contributors: Daniel Stonier, David Lu!!

1.11.9 (2014-06-10)
-------------------
* fix hypot issues, add comments to tests from tracking this down
* dynamically reconfigure the previously uninitialised variable 'combination_method', closes `#187 <https://github.com/ros-planning/navigation/issues/187>`_.
* uses ::hypot(x, y) instead of sqrt(x*x, y*y)
* Contributors: Daniel Stonier, Michael Ferguson, Enrique Fernández Perdomo

1.11.8 (2014-05-21)
-------------------

1.11.7 (2014-05-21)
-------------------
* uses %u instead of %d for unsigned int
* update build to find eigen using cmake_modules
* inflation_layer: place .top() & .pop() calls together
* add parameter to configure whether full costmap is published each time
* Contributors: Michael Ferguson, Siegfried-A. Gevatter Pujals, agentx3r, enriquefernandez

1.11.5 (2014-01-30)
-------------------
* Better threading in inflation layer
* don't set initialized until updateMap is called
* check whether costmap is initalized before publishing
* New Overwrite Methods
  updateMap method
  Fix for `#68 <https://github.com/ros-planning/navigation/issues/68>`_
  Fix for inflation memory problems
  InfIsValid `#128 <https://github.com/ros-planning/navigation/issues/128>`_
  Static layer can recieve updates and accept non-lethal values
  Obstacle layer uses track_unknown_space parameter
  Footprint layer is not longer created as top-level layer (used as part of obstacle layer instead)
* Download test data from download.ros.org instead of willow
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Improve bounds checking 
* Reimplement Clear Costmaps Service by implementing reset functions in each layer
* Package URL Updates
* Additional static layer functionality for receiving updates
* Misc. Pointcloud fixes
* Improved eigen alignment problem on 32-bit arch.
* fixed costmap_2d tests
