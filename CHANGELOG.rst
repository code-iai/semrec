^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package semrec
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.2 (2014-10-14)
------------------
* Differentiate between mono and bgr
* Also allowed manually asserted times to produce floating point individuals
* Added gmon.out to .gitignore
* Changed the default list of plugins to load and added symobliclog time precision
* Added config option to the `symboliclog' plugin for floating point timepoints
  The `symboliclog' plugin now features an individual configuration option `time-precision' that controls with which floating point precision time strings are generated. If the value is left out or set to zero, no floating point precision is generated (i.e. timepoints are integers). If a number is given, that many decimals are used for precision.
* Removed debug output
* Added support for optional floating point precision into time string generation
* Fixed a build bug that would trigger pulling of 3rdparty code every time
  Even if the code shouldn't be rebuilt (i.e. when the `built_flat' file is present), the repository for 3rdparty/json-c code would still be updated. This isn't happening anymore now.
* Merge branch 'master' of github.com:fairlight1337/beliefstate
* Allow alter context command forwarding
* Add boolean flag to decide whether to set symlinks for knowrob owl files or not
* Added information on designators: Creation time and index designators
  `Index designators' are designators that are the head of a chain of created designators, and include at least one successor in the chain. Also, they are not a successor of another designator. All designators now have the `creationTime' field, which denotes the time they were issued for the first time.
* Got rid of commented-out code
* Added automatic checkout of current json-c lib, and using it in JSON class
* Added model generation logic files
* Updated OWL reader python class
* Finished decision tree inversion algorithms and query interface
  Given a set of features, the prediction plugin can now produce solutions describing which features need to be changed (and how) to get to a target result.
* Separation of branches by target result is working nicely.
* Further enhanced decision tree inversion, although its not complete yet
* Fixed memory leak
* Added copy constructor for Property class
* Allow registration of custom namespaces for OWL file export
* Properly handle ended contexts that are not active
* Adapted the .dot exporter plugin so that it can handle objects in multiple nodes
* Equipped symbolic log plugin with capabilities for concurrent timeline logging
* Make sure internal service events called from the ROS feeder plugin are waited for
* Removed unused functions
* Readjusted the knowrob namespace to the correct one
* Corrected KnowRob import IRI
* Object references to the same object now have the same object individual
  When objects are annotated to multiple contexts, the object IDs of these annotations are the same now. Also, no extra object individual (with unique ID) per object/context pair is created. This makes reasoning about what happened to a single object much easier and makes the resulting .owl files much more readable.
* Properly integrated usage of the new DecisionTree class into PluginPrediction
  The loaded decision tree models are now transparently used while predicting the outcome of a task, and the resulting relative failure occurrence (e.g. the DTree's result) is correctly fed into the prediction algorithm.
* Moved decision tree mechanism into its own class file and adapted predict plugin
  Since the decision tree mechanism is now a pretty general functionality, it was moved into its own class file (accompanied by a clean interface) such that it can be reused again easily. The prediction plugin uses this class for loading and evaluating decision tree models.
* Fixed compilation issue, eased informational output of missing features/operands
* Big update on decision trees: Loading and evaluating them works
  Decision trees created by algorithms such as Weka's J48 can now be loaded nicely into the prediction plugin (after they were converted into a suitable JSON format, see https://gist.github.com/fairlight1337/5366de28a6ae9316715d). Given a set of (dynamic) feature variables, the trees can be evaluated and can return any given leaf class value produced.
* Fixed a wrongly set `wait' flag that would deadlock the system
* Completely removed ROS dependency from prediction plugin and moved model loading mechanisms into new service event infrastructure
* Added support for tags
* Big fix: Adjusted idle time values, solved inconsistencies
  The idle time values were holding the system up where it was supposed to handle messages very quickly. On a conceptual level, this should not be a problem, but apparently ROS service requests get mixed up in their ordering when many of them are waiting for a ROS service to become available.
  This effect was messing up log files, resulting in broken trees and unusable data. Putting the values down to reasonably small values (still > 0) solves that issue AND keeps CPU load below 100% on all cores. Formerly, all CPU cores would go to 100% load. Now, this only happens for one of them while the logging system is under full steam.
* Added stack of nodes to make current stack depth easier to track
* Adjusted the knowrob OWL namespace to the new one
* Save JPG files rather than PNGs (reason: size)
* Fixe some issues with response events, and reset sequence numbers
* Properly register resetSequenceNumbers() function for global use
* Refactored prediction mechanism so that it actually works with the new models
* Also removed CMakeLists.txt references to C50
* Removed obsolete C50 implementation files
* Refactored how information is handled in the prediction plugin
* Safety checks
* Removed now-obsolete comment
* Greatly enhanced synchonization mechanisms for asynchronous messages between plugins
* Consider alter-node requests as service requests if the appropriate type flag ist set
* Greatly enhanced action synchronization through asynchronously running plugins
  Plugins can now effectively call each other's services __without__ using another cycle. Since all plugins run asynchronously, their can operate independently from each other.
  Also, replaced the prediction plugin's `predict' service by a more central and generic `/service' ROS service that allows to call arbitrary plugin services and receive the results.
* Enhanced the prediction model handling and de-/ascension
  The prediction model structure was changed and enhanced with several bits of data. The new structure is now being used by the prediction plugin for ascension and descension to track the active state.
* Reintroduced the global input lock for ROS service callbacks
  To ensure that no race conditions happen, a global access mutex to all data structures accessed from ROS service callbacks is now in place. Service callbacks can now only be active one at a time, of any type (begin, alter, end context).
* Added throw/catch failure counter for OWL export
  When the counter is != 0 after finishing an export, the number of failure throws didn't match the number of catches. This is a serious problem in semantics and will break the resulting output, so a warning in displayed in that case. If the warning isn't shown, everything is quietly assumed to be alright.
* Added ability to optionally set object properties (class, namespace, property)
* Fixed a wrongly assigned 'NamedIndividual' to 'namedIndividual'
  Under certain configurations, this would break reasoning as the named individuals don't all show up together. This is fixed now.
* Several additions for controlling CPU time hogging for different components
  The main cycle, the ROS spinner, and the PluginInstance class were using all CPU cores at 100% if possible. This fix should resolve that issue.
* Added profiling options to CMakeLists.txt
* Added explicit `taskSuccess' property to node individuals
* Properly add timestamps to .dot output nodes
* Hide internal values from .dot output
* Make sure the config value is only deleted once
* Fixed minor typo in fixed ExperimentMetaData individual name
* Compatibility fix for newer gcc compilers
* Additional to everything else, note down what parameter types were annotated per node
* Two enhancements: Don't annotate unsupported parameters, and note their types
  Custom parameter annotations in event individuals have certain types that are supported. These consist of strings and numbers at the moment. Unsupported types would up to now result in an empty annotation. This is fixed now, a warning is displayed, but no empty annotation is created in the resulting .owl file.
  Second, an `AnnotationInformation' individual is created now. All parameter types annotated throughout the whole experiment are denoted here, allowing easy comprehension of what to pay attention to when processing custom parameters.
* Pay attention to optionally set custom class names and namespaces when exporting
* Pay attention to optional start and end timestamps for beginning and ending contexts
* Updated .gitignore
* Updated config.cfg file
* More source code documentation
* Finally fixed loading config files from other default locations
* Removed Doxyfile file from .gitignore to be able to check it in
* Documented more type entitie
* Removed obsolete experiment-knowledge plugin
* Added Doxyfile
* Updated part of the code documentation
* Fixed designator ID specialization
  The mechanism for specializing designator IDs didn't actually set new type strings for the node name. Now, it does.
* Added first designator id specializers by designator type
* Added ability to decide on designator identifiers in unique ids
* Add message upon completing initialization
  This signals that initialization is over and logging can begin.
* Clear root nodes in the symbolic log when the experiment starts
* Refactored root node export to support multiple trees in one log
  The ExperimentMetaData individual can now hold multiple subAction properties in case multiple task trees are present in the same log.
* Include root node in meta data, unique meta data individual, version bump
  The meta data individual now has its own unique ID. This makes sense when multiple experiments get loaded (first, to not have clashes between experiment meta data individuals, and second, to distinguish experiments better).
  Also, the meta data now includes an own `knowrob:subAction' property that names the top-most parent node. This makes identification of the overall parent very easy, as opposed to scanning all individuals for being a subAction of any other node (which can get very time-consuming).
  Plus, version numbers were adjusted where appropriate.
* Made LogAnalyzer output more meaningful
* Corrected a left-over plugin name replacement by PLUGIN_CLASS
* Changed output of log analyzer to a more detailed version, incorporating new features
* Print one-time notification when first log context was started
  When suppressing messages (besides important ones), it's difficult to tell whether logging is actually active or not. Print one `important' message when the first context was begun to signal that logging is active.
* Version string bump
* Added more source code documentation for the main `beliefstate` class
* Introduced feature to suppress all unnecessary text output during logging
  The "only-display-important" config option now suppresses text output globally. This can be overridden by plugins for individual messages, and is overridden by the core altogether.
* Added explicit interface for finding objects
* Output readily calculated information about experiment statistics
  Prototyping new experiment analysis methods.
* Bugfix for OWL meta data export
* Implemented writing experiment meta data to exported OWL files
* Removed typo that invalidates the semantics descriptor file
* Added support for explicit belief state updates
* Added custom .gitignore for bstools
* Added explicit support for motion planning and execution processes
* Added explicit interface for object identity resolution events
* Fixed a few bugs, added output for time categories
* Finished first version of python bstools for analyzing logged memories
  The toolkit analyzes the generated log-OWL-files and creates two kinds of information: A proper task tree with all timing information for all children, plus a disc-like figure from that, and a sorted list on which tasks take how long.
* Added support for `type navigate` designator specialization, and fixed a bug
  A nasty bug was inserting `std::string` RDF classes in the XSD namespace. This resulted from copying `string` to `std::string` in complete files. Now, that should be fixed.
* Fixed a nasty bug that would prevent logging from continuing after taking images
  An open request ID was preventing the logging system from going on after it took an image and saved it to the current node. This commit fixes this.
* Fourth and last batch of major code overhaul
  Removed all `using namespace ...` instances to make the code less namespace-pollutant (and less polluted). All `for` loops were replaced by their respective `simpler` versions where applicable (so iterators are only used explicitly when `erase` was actually used on lists).
  Nicified lots of smaller code bits as well, making the overall code more readable.
* Greatly simplified algorithmic code of the OWL exporter class
* Third batch of major code cleanup
* Second batch of namespace wiping, code nicification, and general cleanup
* First batch of code cleanup, nicification, namespace wiping
  Removing all `using namespace ...` directives to make the code
  a) more compatible
  b) less polutant
  Also, replaced `for` loops with the correct versions when iterating over std STL containers, and removed old, unused (or commented-out) code pieces.
* Added action designator performance specializers
* Added functionality for properly loading timestamps of tasks, and optimize them
* Only predict when a model was loaded
  The prediction plugin would return errors when trying to predict without a model present. This fix circumvents this and ignores all prediction requests when no model is present, returning SUCCESS on all occasions (i.e. no failures).
* Added ArbitraryMappingsHolder intermediate class
  The class will hold arbitrary static configuration data, to be saved in arbitrary mapping files. These are configurable through the main config.cfg file per plugin. Also, cleaned up the linking mechanism to make linking new components easier and clearer.
* Made symboliclog depend on imagecapturer
  Right now, the system would block if an image is to be captured when no imagecapturer is loaded. This fixes that for now.
* Clean up
* Cleanup
* Got prediction running properly, based on fixed decision tree
  The fixed decision tree generated from training data (actually extracted from the very log files used here) properly predicts the upcoming plan errors based on active parameters provided. Plans now can predict the outcome of an action and reparameterize, until the prediction yields successful results.
  The next step is to integrate the decision tree gneration into the prediction plugin itself.
* Commented out unnecessary definition
* Added a great deal of failure handling details to the symoblic log plugin
  Failure handling (and rethrowing, in particular) was making serious problems during logging. This should, however, now be solved. Problems arose when failure handling nodes that previously were able to handle a failure tried to hand up the failure to a higher instance. The emitter/catcher mapping was then totally messed up, as the respective information was not updated accordingly.
* Fixed issues in a `switch` statement (missing `break`s)
* Removed unused parameter
* Greatly enhanced prediction performance by pre-computation when loading model
  The nodes/failures mappings (that are pretty much static throughout an experiment run) were calculated every time a prediction was triggered. With large trees, this can take up to several minutes. This is done in one step now when a new model is being loaded and is saved in a map for all future predictions, reducing the prediction time down to at most half a second.
* Allow event notifications for nodes that have been set active
* Set up new experiment space when the `start-new-experiment` event arises
  The symbolic log didn't pay attention to the `start-new-experiment` event up to now, but is now clearing and initializing all of its internal data to be ready for a new experiment instance.
* Added version of 3rdparty C5.0 algorithm for decision tree support
* Cleaned up and fixed a few tree linearization issues
  All probabilities are now generated correctly, plus the success rate. There still was an issue with long trees that weren't linearized correctly - and this is now solved.
* Slightly changed how knowrob tags are exported for annotated parameters
* Finally made predictions based on the actual probabilistic model work
  The joint probabilities of all nodes within a prediction branch are taken into account, and the respective failure rates vs. success rates are returned to the calling plan instance.
* Introduced support for manual parameter annotations
  Nodes can now be manually annotated with custom parameters. These can be used for e.g. the current distance between the robot and an object in question, the goal location to navigate to, ...
* Fixed prediction; found out why values weren't correct
  The sub-branch predictions were multiplied with the wrong success rate, always resulting in wrong probabilities. Also, the compiler seems to be invariant betwen interators of type map<string, int> and map<string, float>. So making mistakes here isn't noticed, and can result in loss in information. This is why the success rate never changed from 1.0.
* Predictions are happening, but something is not yet right with the values
  The prediction tree is correctly being walked, but the collection mechanism for failures and their individual probabilities still yield weird (not so say _wrong_) values.
* Failure deduction from node names in, prediction split up into branches.
  Still to do: walk through sub-branches when predicting.
* Added missing BSD headers
* Further refined prediction tree walking, and prepared actual prediction mechanism
* Ascending and descending the prediction tree works perfectly now
  Even stack protected. There were problems involving weird states in which the prediction stack can get when the executed plans involve (race-condition-prone) parallel execution code, but by introducing a wildcard class `*`, this can be gotten over with.
* Send symbolic-end-context event to all plugins for prematurely ended nodes
* Greatly enhanced prediction module, cleaned up, Owl classes in
  Ascent and descent inside the prediction tree/stack now works nicely. All classes inside the prediction track now refer to the correct Owl classes from the plan logs (and prediction models, thereafter).
* Prepared walking (ascending and descending) the prediction tree
  All consumable events are connected, and the mechanisms for accessing the prediction tree and stack are in place. Now, only accessing the proper ontology classes is missing (converting pure CRAM task names into ontology entries).
* Added JSON and `Property` support for bs_plugin_prediction
  JSON-based prediction models are now properly loaded from .json files and represented as `Property` data structures.
* Prepared everything for model loading and prediction.
  The actual format for prediction models must still be decided, but all
  services for loading and the actual prediction are set up.
* Extended skeleton files, filled service callbacks with more life
* Equipped prediction plugin with services
* Added skeleton files for prediction plugin to beliefstate
* More fixes to linking
  Apparently, the designator_integration/DesignatorIntegration link should
  not be done manually, but is handled by catkin completely. Removed the
  manually added references.
* Fixed linking errors
* Moved the ''findPrefixPath'' function from BeliefstateROS to Beliefstate
  The function is not ROS specific, so it goes into the superclass where it might be useful to other functionality as well.
* Contributors: Jan Winkler

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
