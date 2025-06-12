^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_ros2_socketcan_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.4 (2025-06-12)
------------------
* Fixed using a global namespace.
* Match package name
* Fix typos in issue templates
* Contributors: Hilary, Tony Baltovski

2.1.3 (2025-05-21)
------------------
* Fix: Spin Timeout (`#13 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/13>`_)
  * Add timeout to spin to prevent script from waiting when node has failed
  * Update log to match appropriate action
* Contributors: luis-camero

2.1.2 (2025-05-20)
------------------
* Fix: Use script instead of OpaqueFunction (`#12 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/12>`_)
  * Use script instead of OpaqueFunction
  * Add missing line
  * Add license header
  * Add EOF line
  * Add retry in the event lifecycle service is not up yet
* Use arguments instead of perform(context) (`#11 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/11>`_)
* Reattempt transitions on failure (`#10 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/10>`_)
  * Reattempt lifecycle transitions on failure
  * Unique activator node names
  * Removed namespace from node name
  * Linting
* Fix: Multiple IncludeLaunchDescription (`#9 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/9>`_)
  * Use opaque function
  * Add import and context
  * Type performed context
* Contributors: Roni Kreinin, luis-camero

2.1.1 (2025-04-08)
------------------
* Increased timeout to 1 second (`#8 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/8>`_)
* Contributors: Roni Kreinin

2.1.0 (2025-01-31)
------------------
* Wait for interface to be up before launching node (`#7 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/7>`_)
  * Wait for interface to be UP before starting node
* Updated CI to Jazzy. (`#5 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/5>`_)
* Contributors: Roni Kreinin, Tony Baltovski

2.0.1 (2025-01-17)
------------------
* Lint launch files (`#4 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/4>`_)
* Prefix node names with can interface
* Add our own launch files that allow us to change namespaces
* Change CI to Humble.
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero

2.0.0 (2024-11-21)
------------------
1.0.0 (2024-11-21)
------------------
* Added README.
* Disabled copyright tests.
* Fixed linting.
* Added CI
* Added issue templates.
* Added codeowners.
* Updated package.xml
* Tx queue and wall timer
* Constructor with callback
* Small code styling to make default tests pass
* Add delay before publishing messages
* Initial add of clearpath_ros2_socketcan_interface
* Initial commit
* Contributors: Chris Iverach-Brereton, Luis Camero, Roni Kreinin, Tony Baltovski
