^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package beliefstate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2014-05-16)
------------------
* Annotate nested designators with their respective IDs, and publish them
* Removed obsolete code and replaced it with new function calls
* Improved workspace directory identification
* Moved designator publishing ensurance in dedicated function
  Hopefully, this didn't break the mechanism -- but now the oftenly used, important method of publishing designators and registering them in the plan log properly has its own function.
* Added prolog details to semantics descriptor file
* Support for with-theme-details designators
* Per-plugin configuration of lists now possible; experiment validation extensions can now be configured via the config file
* Improved detection and handling of prematurely ended contexts
* Automatically remove experiment that doesn't include an .owl file
  When shutting down the belief state system, the current directory will be removed in case it doesn't include an .owl file. Symlinks don't count. In case there is no .owl file in the directory, the ''current-experiment'' symlink will also be removed to denote that no current experiment is present.
* Reimplemented ability to limit PDF output by max detail level
  The max detail level now does not only limit the nodes that are displayed, but also lets existing subnodes of such nodes with a valid detail level still be displayed. Nodes that are not displayable due to a failed success/failure state are still not displayed (and neither are their children, disregarding their state).
* More checks for NULL
* Added checks for invalid pointers to find the problematic crash when exporting owl files
* Added informative output to owl exporter
* Create knowrob.owl symlink in experiment directory
* Added fixes
* Fixed package.xml and added a forgotten '')'' to version output
* Updated package.xml
* Default reaction to failed plugin loading is to invalidate startup altogether
  This option can be customized on the config file.
* Remove unloaded plugins from the index after deleting their instances
  Having the old plugin instance references still in the ''m_lstLoadedPlugins'' list results in segmentation faults due to the core system still trying to access their ''cycle()'' value.
* Core system now has a version number and proper output for it; also, restructured main.cpp a bit
* Command line output can be controlled via the config file
* Don't load plugins that failed to load before during the same run
* Security checks
* Make sure that all nodes have an end time
* Properly forward annotations, and don't reset failures when caught once
* Moved ''getTimeStamp'' into UtilityBase class and changed use where appropriate
  Also, supplied ''str'' functions for UtilityBase. This can now convert float, double, and int into strings (formerly done manually using sprintf or stringstream, mostly for timestamps). There is also a new function for directly outputting the string timestamp, i.e. ''getTimeStampStr'' (used a lot). Now, no unwanted thousand delimiter commas should show up in the timestamps anymore due to a centralized mechanism.
* Automatically set owl exporter version as metadata field when starting an experiment
* Notation of caught failures is now implemented
  Nodes now ''know'' whether they caught a failure, which failures it were, and which node emitted them. This is reflected in the command line output, as well as the resulting .owl logs. A new property, ''knowrob:caughtFailure'' now includes the reference to the failure individual (which, in turn, is also referenced by the emitting event individual). Event individuals / nodes can catch multiple failures in subsequent tries.
* Moved string replace function to UtilityBase class
* Don't unnecessarily wait for shutdown when starting the first experiment
* Don't subscribe to status messages for now, as it disturbs normal status output
* No usleeping when not necessary; also, made output clearer
* Moved a great deal of information from the ''CExporterOwl'' class into the semantics descriptor file
* Moved evaluation of designator annotation-to-purpose-tag into separate function
* Separated the pure beliefstate system capabilities from the ROS related parts
  The parts of the core beliefstate system that are ROS independent are now handled within the ''Beliefstate'' class. All ROS related enhancements (like handling the ROS workspace, and finding ROS package relative paths) are covered by the subclass ''BeliefstateROS''.
  This is independent from the ''bs_plugin_ros'', which handles ROS related initialization, communication, and shutdown.
* Added the infamous tf cache error to the semantics descriptor file
* Allow roslog'ging all status messages to a given topic
* Messages can now take up multiple lines while not interfering with lines after them
  Also, fast resizing does not crash the plugin (added mutexes at the right places).
* Fixed access violation between two threads when accessing the status message buffer
* Hide cursor
* Finally got the output for the console plugin right - plus resizing the window
* Gave plugins their output colors back
* Pay respect to with-policy and with-failure-handling when exporting owl
* Limit amoutn of screen output to buffer
* Fixed slight bug that made output in the console plugin ugly
* Made sure that output is being displayed; this resolves a bug introduced by the new message distribution strategy
* Removed ugly artifacts when outputting using the console plugin
* Subscribe and react to the 'resize-terminal-window' event, emitted for SIGWINCH
* Catch SIGWINCH signal and forward it as event into the event pipeline
* Finally got the garbled output from the console plugin fixed
  A mutex wasn't being paid attention to, and during redrawing of the interface, a memory corruption was the result.
* Moved the whole status message output distribution into the event system
  All messages that are being output onto the console are now events of type 'status-message'. If one or more plugins subscribe for this type of event, they will get a detailed message about the text to output, its color, boldness, and prefix label. If no plugins subscribes to this type, a default output inside the class 'Beliefstate' will do the 'old' way of just cout'ing the text stdout onto the console.
  The reason for this is, that a plugin might change the characteristics of the terminal (e.g. ncurses) and 'normal' output might interfer with this. If no such plugin is loaded, everything stays the same. Also, output could now be automatically be logged into a file by a fitting plugin.
* More ncurses code for the console plugin
* Added first version of the ncurses-driven console plugin
* Added a default config.cfg configuration to the configs directory
* Made parse error output more precise
* Added missing semicolons to the semantics descriptor file
* Added failure mapping for location-not-reached-failure
* More documentation
* Added doxygen output directory to .gitignore
* Added more source code documentation
* Added Doxyfile to .gitignore
* Added first patch of doxygen documentation strings
* Filled out and cleaned up package.xml
* Added BSD headers to all source and header files
* Add link to cram-systems.org documentation of beliefstate system
* Take additional default plugin search paths into account
* Take ROS_PACKAGE_PATH into account when doing directory token resolution
* Added utility function 'stripPostfix' to UtilityBase class
  This function optionally strips a given postfix from a given string if present. It returns the string otherwise.
* Removed obsolete commented out code
* Check for ROS availability before removing interactive objects from the server
* Allow manual override of workspace directory in config file
* Added support for holding image capture timepoints when images are added to the log
* Implemented loading of semantics descriptor files into CExporterOwl
  This might break operation that need the failure mappings in OWL classes for configurations where the workspace directory cannot be resolved. Better replace the dynamic path resolution (i.e. '${PACKAGE beliefstate}') in the config.cfg file by the absolute path if that problem comes up. The beliefstate core component should complain if this becomes a problem, so watch out for the warning messages.
* Prepared parser function for semantics descriptor files
* Moved fileExists function to UtilityBase
* Link to config++
* Get rid of ugly extra slash character in config file paths
* Created initial version of the CRAM/KnowRob semantics descriptor file
* Moved loading of semantics descriptor files to the proper plugin (owlexporter)
* Added configuration option for the ros plugin to control the number of asynchronous threads to start when spinning
* Added warning output when no workspace directory could be resolved
* Removed debug output and added todo for loading the semantics descriptor file
* Extended main example config file by plugin configuration options
* Added configuration options to the ros and symboliclog plugins
* Full support for recursive, individual configuration of plugins
  Plugins are now fully customizable from the main config file. Their options are read recursively into a designator structure, allowing to nest configuration groups.
* Introduced support for per-plugin configuration options
  Plugins can now be configured with individual options, directly from the main configuration file. Currently, only string-options are supported, which will in the future be extended to complete designator-like structures.
* Get rid of ugly ^C output when CTRL-C'ing the logger
* Finally get rid of the `packaging` directory in .tar.gz's
* Added command line options for files to check to consistency check python script
* Hotfix for stringstream number notation problem
* Fixed number notation for stringstream
* Allow to set date placeholders for experiment names
* Don't instantiate the belief state main class if only the help screen is displayed
* Added a bit of interface documentation
* Pay attention to the special designator annotation of type 'graspDetails'
* Let the supervisor set the experiment name upon startup
* Whitespace fix
* Link experiment-context to DesignatorIntegration
* Publish metadata to a topic when extracting files
* Use output capabilities of Beliefstate class instead of manual `cout`s
* Fixed a bug that would result in a publisher failure when unloading the `interactive` plugin
* Make sure designators are correctly associated with nodes even if they already exist
* Special treatment for designators annotated as 'goal-pose' or 'goal-location'
  This adds semantic information to base-movement actions. 'goal-location' should be a location-designator, holding the symbolic description of the pose to go to. 'goal-pose' is the actually resolved pose.
* Corrected config descriptions and changed a filename to a more appropriate one
* Added directory for custom config files, and an exemplary config file for only loading the 'interactive' plugin (which in turn automatically loads the 'ros' plugin as a dependency)
* Added option for loading custom config files via command line, and greatly enhanced error handling during loading of config files
  When information is omitted from config files, defaults will be assumed from now on. Also, missing information does not break config file loading, but is properly checked.
* Introduced changes to owl structure
* Go back to beginning of line when printing quit message
  This omits the ^C character shown in the console resulting from pressing CTRL+C
* Added missing space
* Free context IDs after the contexts ended. Also, more specific output.
* Set version strings for individual plugins and correct dev status
* Added ability to set an optional version string for plugins
* Interactive marker plugin is no longer a development plugin
* Before applying any changes to InteractiveObject instances, check whether ros is ok
* Initialize RNG with random seed
  The random number generator was always producing the same unique designator IDs. Since this could potentially lead to problems when intersecting multiple log instances, the random seed is now initialized properly (with `time(NULL)`).
  Also, more informative output for when equating designators.
* Added config options for plugin output colors and unhandled event messages
* Removed an old warning
* Fixed pose extraction for adding interactive objects
* Add default pick up object menu entry for objects added from beliefstate
* Pose extraction from added object designators for interactive objects
  When adding an object through the `add-object-to-active-node` beliefstate interface, it is now forwarded to the interactive object plugin (plus a proper pose if available in the object).
* Regular `usleep` in the main cycles of central `Beliefstate` class and plugins
  The main cycles of both instances were running at a very high fidelity, causing the CPU to be pretty busy with just this. Put `usleep(1000);`s in there to ease the processor down a bit (and since such high frequencies are not necessary here).
* Informative output for interactive objects
* Built full support for interactive objects (plus the respective interfaces)
* Equality check error for setting the experiment end time fixed
  The end time of experiments was not automatically set when exporting the planlog, due to an error in an equality check. Fixed this.
* Added object designator publishing when adding an object instance to a context
  The received designators of objects added to contexts were not published on the designated ROS topic again. This should be fixed now.
* Set success only if no failures are available in a node when ending its context
  This fixes a bug in which an `end-context` event would overwrite any `success = false` states in any node that was set by `add-failure` before. Now, the nodes are properly marked as successful or unsuccessful (also in the .dot output, marking unsuccessful nodes with red lines).
* Slightly changed the output of the `experiment-context` plugin
  The experiment start and end time tags are now called `<time-start>` and `<time-end>`, respectively. Also, the `experiment-shutdown` event will trigger saving the current time as `time-end`. If this was not called throughout the experiment before exporting, the export time will be used for this purpose instead.
* Added convenience method to find out whether individual nodes contain failures
* Added scripts for result packaging and consistency checks
* Add experiment start and end times in metadata when exporting logs
* Introduced forwarding of node characteristics from CRAM
  The functionality was missing and now supplies information about the current task node context in large extents (especially for goals when tried to achieve them). This includes more information in the exported .owl, and .dot files.
* Probably found the cause for missing designators in the published log topics
  When equating a designator that originated from `with-designators` and one made with `create-designator` or `make-designator` (so, not tracked when creating them), they show up in the symbolic log, but not in the database. This should be fixed now.
* Publish unique designators only once; correctly tearing down prematurely ended contexts
  Designators were published twice due to external calls. This is now fixed by taking the already known unique designators into account when publishing new ones. Also, prematurely ended contexts were not annotated with their success state and their end time correctly. This is fixed now (they get the same flags as the ended context ID that flagged them as prematurely ended).
* Made the experiment-context plugin properly accept data and export a meta file
  The metadata.xml file created by the plugin is now stored in the current experiment's folder. Its data fields are purely determined by what the plan execution entity sends to it.
* Added skeleton files for experiment context plugin
  The experiment context plugin shall hold information about
  a) what entities were part of the experiment at hand
  b) what was the intended purpose of the experiment
  c) additional notes about the current situation
  Also, the plugin should be able to export files containing this information (into a designated .xml file for example).
* Forgot to commit the header file for the UtilityBase class
* Renamed the DotExporter plugin class to its correct value
* Finally got a central mechanism for outputting text of different semantics
  The output of different system parts (the core beliefstate system, the plugin loading system, the individual helper classes) are now using the capabilities of a central `UtilityBase` class, which allows for outputting formatted, colored text. Also, the output messages generated this way are always marked with the name of the emitting entity, making backtracking of problem origins easier.
* Added more explaination to the config file
* Added function for removing an interactive marker object from the server
  By calling the appropriate function, a spawned instance of `InteractiveObject' can be removed from the interactive markers server again. Also, made this plugin a development plugin.
* Added capabilities for differentiating between normal and development plugins
  Plugins can now set the `bDevelopmentPlugin' flag in their constructor. If this flag is set, and the `load-development-plugins' flag in the config file is set to `false', those plugins will not be loaded. This serves the purpose of ignoring plugins that are not necessary for (or might interfere with) normal operation. Either way, the user will be notified when a development plugin is loaded, or when it is ignored.
* Made superclass destructors virtual, and introduced new `unimplemented' message
  Destructors of `Plugin' and `CExporter' classes are now virtual to prevent undefined behavior when deleting subclass instances. Also, introduced new output message type `unimplemented' (besides `info' and `warn') to be used for functionalities that are not fully implemented yet (more visiblity to the user/developer).
* Add annotations to designator events
* Creating designators and adding them are now two different atomic symbolic actions
* Changed event type name for semantic reasons (its just understandable far better this way)
* Implemented sending out added failures via events, and made adjacent changes to helper classes
* Enable to export a linear symbolic plan log path instead of only the whole tree
* Add experiment-knowledge plugin details and fix the config file so it works on older libconfig-versions
* Build skeleton experiment knowledge plugin
* Implemented a PLUGIN_CLASS macro to make plugin class name definition in source files easier
* Allow adding objects for interactive use through events; proper interactive callback handling through events; fixed a bug that would try to shutdown an (non-existing) experiment prior to the first one
* Subscribe to internal events; also, infrastructure for interpreting object add events and updating their pose is prepared
* Forgot to remove a faulty `break;'
* Simplified usage of the marker setup a bit; also, removal of menu entries working
* Added a lot of code for dynamic definition of interactive objects, and for dynamically populating the context menu for objects
* Added basic version of interactive markers for the new `interactive' plugin
  An interactive marker with a default menu entry (dummy) is generated and connected to a feedback function in the `interactive' plugin. The basic setup is there, now some functionality needs to go into it.
* Updated .gitignore to cover .rrd files
* Added symbolic event hook when equating designators (so other plugins can use this information)
* Moved the whole designator logging (publishing to /logged_designators) into an event driven function in the ROS plugin
  This was necessary to make sure that the unique id (which is generated for logged designators) is generated first, and the id'd designator is published afterwards. Works nicely now. Equation as well. The format of designator ids in the mongodb changed a bit, though (<id> -> designator_<id>).
* Fixed a cause for segfaults; added note in code about cause
* Fix and completely implement capturing images
* Trigger symbolic add image and set subcontext when respective plan events arrive
* Added .dot file format exporter
* Delete owl exporter instance after export
* Properly add image file references (image individuals) to event individuals in exported .owl files
* Add images from file to symbolic log
* Added extra (optional) parameter to owl class generator for prolog syntax output
* Merge branch 'master' of github.com:fairlight1337/beliefstate
* Forward symbolic events when new nodes were added to the symbolic log
* Added function for finding previous actions of nodes
* Removed `imagecapturer' as direct dependency from `symboliclog'
  The image capturer component is not a necessary component for the symbolic log. If no plugin is loaded to store images, it is just not done.
* Extended token parsing for config files when paths are defined dynamically
* Create README.md
  Added basic information about what the system does.
* Also, made the base data directory dynamic (i.e. using tokens) by supplying a global token replacement function.
  The `$HOME' token is now resolved to the current user's home directory (and can be used in the base data directory, and in search paths).
* Removed unnecessary comment.
* Replaced static lib search path by dynamically generated path
  The variable `$WORKSPACE' is now replaced by the currently active ROS workspace devel directory (in catkin, this might be /home/johndoe/catkin_ws/devel). If `$ROS_WORKSPACE' is set, it's value will be used. If not, the first (colon-separated) value in the list of paths in `$CMAKE_PREFIX_PATH' will be used. By default, the search path in the config.cfg file is now set to `$WORKSPACE/lib/' to take advantage of that.
* Correctly export generated OWL files in the current experiment directory
* Let imagecapturer save captured images to the current experiment directory
* Current experiment symlink name now configurable through config file
* Dynamic management of experiment spaces completed. Directories are created, symlink is set, and the global settings are updated accordingly when starting a new experiment through the supervisor.
* Introduced global events from the main beliefstate component, als extended the supervisor to start a new experiment when beliefstate startup is complete
* Eased use of open event request waiting; also, added supervisor plugin for starting new experimental environments
* Made all ROS communication asynchronous; also, fixed threaded internal communication (events, services)
  There were several blockers (mutexes, namely) within the thread communication code. These should be fixed for now. ROS services called from the outside can now block while the plugins in the beliefstate process the request asynchronously.
* Replaced the manually set base data directory by a much more convenient global settings structure
* Made all plugins threaded.
  This will help in asynchronous communication with components connecting to the beliefstate. All plugins are now executed in their own thread, and communicate with the `master' component via mutexed Result variables.
* Reintroduced republishing of captured images
* Changed c++ mode to c++0x from c++11. More compatible with other versions of cc1plus this way.
* Prepared structures to transport global config settings
* Correctly publish logged designators to a specified topic
* Nicified output of ROS plugin
* Remove entries from the list of plugins to load before loading a new config file
  In case a config file has begun to be loaded, and threw an exception, already existing entries in the list of plugins to load would have survived this (and, therefore, would be loaded when the next successful config file parse was done). This is fixed now.
* Made sure that plugins are only loaded once (based on their *real* name)
* Nicified output
* Removed development service from Gazebo plugin
* Made error messages a bit more meaningful; also, only output them it there was an actual error
* Also read experiment data settings from the config file (+ some notes in the source files)
* Support for loading config files, and finding config files at predefined places
* Added CImageCapturer worker class, and switched from precoded event identifiers to event names (identified by std strings)
* Introduced a first version of the config file as it will be used for configuring the beliefstate.
* Reimplemented recording of failures, designators, objects, and preliminarily even images. The images are not yet taken, though. Also, set the base data directory in all plugins.
* Correctly configuring OWL exporter and running it
  The designators, failures, etc. are not yet added (this needs to be reimplemented in PluginOwlExporter), but the basic functionality is back!
* Nicification
* Integrated formerly prepared OWL exporter class. Compiles, is included, but must still be wired into PluginOwlExporter
* Check for requested export filetype to actually be OWL in the OWL exporter
* Cleaned up, fixed services, added OWL exporter plugin, built pipeline for it
  When the ROS node is ordered to export the plan log into a file, the ROS plugin receives it and posts it to all plugins that understand this event. These plugins then request a service `symbolic-plan-tree' from plugins that support it. These plugins then send back their plan trees. In the end, the exporter plugin ends up in a function call, having the original event data from the outside request to export a file, plus the plan log tree data. ready. Three plugins involved, purely internal communication, very flexible and extensible. Yay!
* Fixed a missing initialization flag
* Reply to ROS service calls with the current id of a newly generated plan node
* Added color to output
* Added _actual_ node logging
* Fixed a few memory flaws; also, reintroduced context (plan) nodes and got first version of logging working again
* Service calls between plugins is in place and working nicely
  The service calls get deployed, collected, spread, and delivered correctly. The results are collected, and forwarded to the original caller.
* Plugins can now offer services by name
* Implemented skeleton methods for spreading service events
* Include services in cycle data deployment
* Prepared service infrastructure, and added convenience methods for simplifying code in plugins
* Plugins loadable by only their (short) names when they are in the search path
  The plugin filename must follow the naming convention for plugins (i.e. `libbs_plugin_<plugin-short-name>.so')
* Plugin dependency lists and automatic dependency loading complete; also, search paths
* Added skeleton Gazebo plugin
  This plugin also includes the showcase implementation of a plugin that uses ROS functionality parasitically. An other plugin initialized the ROS node and maintains the node handle, and this plugin is able to offer services on ROS without having to take care about the setup, and maintenance of the ROS connection.
* Added more plugin code (distribution working better now)
* Plugin infrastructure extended, distribution system extended, more plugins; also, bugfixes
* Initial commit
  This includes a fully functional plugin loading system and preliminary functionality for event distribution between loaded plugins. The available plugin_ros already inistalized, and controls the ROS interface for this node.
* Contributors: Jan Winkler
