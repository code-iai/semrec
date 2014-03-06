beliefstate
===

The beliefstate package supplies a ROS driven system with recording capabilities on low (sensor messages), and on high (symbolic) data level. Its sole idea is to keep track of the contexts in which a high level plan execution system performs actions, record its parameters, and give a detailed output about what happened during execution.

Using low level timing information (timestamps), information from different sources can be combined and associated. Given a plan structure, for example images taken from a camera during specific times can be associated with symbolic events.

The beliefstate system is thought to be used on autonomous robot systems, keeping track of what they do, why they to it (situational context), and what happened during or after their performance.

The plugin based architecture allows for relatively simple extension of the system. Current plugins included and integrated into the system are:

 * ROS interface (connect to the ROS environment, and expose services for starting/altering/ending contexts)
 * Symbolic Log facility (build a task tree with timing information, based on contexts delivered through the ROS interface)
 * Supervisor component (to supervise the start/restart/end of experiments, in order to organize plan logs, and saved image data)
 * Exporters:
   * OWL Exporter (save the symbolic plan log in OWL format)
 * Image Capturer (save images from any given ROS topic, and store them in the experiment data space)

All necessary directories, and details of what to load and where to find plugins, is stored in a convenient config file (config.cfg). Directories are created automatically by the beliefstate system as needed, and data associated with an active experiment is always stored in its respective folder.

A more thorough documentation can be found here: http://cram-system.org/doc/logging/beliefstate