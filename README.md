Semantic Hierarchy Recorder [![Build Status](https://travis-ci.org/code-iai/semrec.svg?branch=master)](https://travis-ci.org/code-iai/semrec)
===

The semrec package supplies a ROS driven system with recording
capabilities on low (sensor messages), and on high (symbolic) data
level. Its sole idea is to keep track of the contexts in which a high
level plan execution system performs actions, record its parameters,
and give a detailed output about what happened during execution.

Using low level timing information (timestamps), information from
different sources can be combined and associated. Given a plan
structure, for example images taken from a camera during specific
times can be associated with symbolic events.

The semrec system is thought to be used on autonomous robot systems,
keeping track of what they do, why they to it (situational context),
and what happened during or after their performance.


Development Activity [![Stories in Ready](https://badge.waffle.io/code-iai/semrec.png?label=ready&title=Ready)](http://waffle.io/code-iai/semrec)
===

[![Throughput Graph](https://graphs.waffle.io/code-iai/semrec/throughput.svg)](https://waffle.io/code-iai/semrec/metrics) 


Plugin-based Extension
===

The plugin based architecture allows for relatively simple extension
of the system. Current plugins included and integrated into the system
are:

 * ROS interface (connect to the ROS environment, and expose services for starting/altering/ending contexts)
 * Symbolic Log facility (build a hierarchical task tree with timing information, based on contexts delivered through the ROS interface)
 * Supervisor component (to supervise the start/restart/end of experiments, in order to organize plan logs, and saved image data)
 * Exporters:
   * OWL Exporter (save the symbolic plan log in OWL format)
   * DOT Exporter (save the symbolic plan log in DOT format)
 * Image Capturer (save images from any given ROS topic, and store them in the experiment data space)
 * Prediction component (predict course of action given models of former plan executions)

All necessary directories, and details of what to load and where to find plugins, is stored in a convenient config file (config.cfg). Directories are created automatically by the Semantic Hierarchy Recorder system as needed, and data associated with an active experiment is always stored in its respective folder.

A more thorough documentation can be found here: http://cram-system.org/doc/logging/beliefstate


### Immediate Usage Instructions

To compile `semrec`, you will need to install the following (Ubuntu) dependencies:
```bash
$ sudo apt-get install libncurses5-dev automake autoconf
```

Besides setup and operation of the recorder, you will find yourself in the situation that you want to nicely package the created logs and adjacent data.
After first running the recorder and acquiring the logs you need, copy the files `package.sh` and `makedot.sh` from the `scripts` subfolder into your local experiment data folder (by default, this is `~/sr_experimental_data`). To package the most current experiment, now just run

```bash
$ ./package.sh
```

and the script will generate a `.tar.gz` file for you, denoting the current time and date, including the generated `.owl` file, possibly captured images, and the contents of your `mongodb` log (`roslog.tf`, `roslog.logged_designators` databases). If needed, adjust the script's inner workings to your needs. Per default, `makedot.sh` generates `.pdf` files from the logged `.dot` files if you have activated the `dotexporter` plugin. If not, this will print a failure, but will move on in packaging your logs.

Afterwards, you will have two additional folders in your experiment data folder:

 * `packaging` holds the raw files archived into the target `.tar.gz` file
 * `results` holds all packaged logs so far
