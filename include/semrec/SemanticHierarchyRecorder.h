/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Institute for Artificial Intelligence,
 *  Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Jan Winkler */


#ifndef __SEMANTIC_HIERARCHY_RECORDER_H__
#define __SEMANTIC_HIERARCHY_RECORDER_H__


// System
#include <iostream>
#include <cstdlib>
#include <list>
#include <string>
#include <libconfig.h++>
#include <mutex>

// Private
#include <ForwardDeclarations.h>
#include <Types.h>
#include <PluginSystem.h>
#include <UtilityBase.h>
#include <Config.h>


/*! \brief Main recorder system class
  
  This class manages the distribution of messages and service events,
  triggers the loading of plugins, and loads configuration files. */
namespace semrec {
  class SemanticHierarchyRecorder : public UtilityBase {
  private:
    /*! \brief Internal pointer to the PluginSystem class instance
        used for loading and managing plugin instances. */
    PluginSystem* m_psPlugins;
    /*! \brief Flag representing the current run state of the main
        loop cycle. If true, the main loop will continue to run; if
        false, will stop execution of the main loop and quit. */
    bool m_bRun;
    /*! \brief Internal copy of the global argc parameter holding the
        number of command line parameters supplied when starting the
        system. This parameter, together with m_argv, is handed to all
        plugins during initialization. */
    int m_argc;
    /*! \brief Internal copy of the global argv parameter holding the
        command line parameter values supplied when starting the
        system. This parameter, together with m_argc, is handed to all
        plugins during initialization. */
    char** m_argv;
    /*! \brief List of plugins to load. This list is populated by the
        plugins/load configuration section in the configuration
        file. */
    std::list<std::string> m_lstPluginsToLoad;
    /*! \brief List of currently available global events to
      process. The entries in this list only last one cycle of the
      main loop. They are dropped afterwards, but are handed to all
      plugins that subscribed to the respective event type
      beforehand. This is the main communication mechanism between
      the belief state system and loaded plugins. */
    std::list<Event> m_lstGlobalEvents;
    /*! \brief In case a static workspace directory was specified in
      the configuration file, its value is stored here. This value,
      if not empty, takes precedence over the dynamically resolved
      return value of workspaceDirectory(). */
    std::list<std::string> m_lstWorkspaceDirectories;
    /*! \brief Mutex that controls whether terminal window resizes are
      currently being processed.
      
      Terminal resize events can be received very qickly after
      another. When one of these events is being processed, it should
      not be interrupted by a new event following it, as system calls
      and non-thread-safe operations are involved.
     */
    std::mutex m_mtxTerminalResize;
    /*! \brief Flag that signals whether the terminal window is
      currently being resized. */
    bool m_bTerminalWindowResize;
    /*! \brief Flag that signals whether command line output should be
      printed or not. */
    bool m_bCommandLineOutput;
    /*! \brief Flag that signals whether all, or only important
      messages should be printed to the console. */
    bool m_bOnlyDisplayImportant;
    /*! \brief Variable holding the current version of the belief state
      system core software. */
    std::string m_strVersion;
    bool m_bDisplayConfigurationDetails;
    
  protected:
    std::list<std::string> m_lstConfigFileLocations;
    
  public:
    /*! \brief Constructor of the main belief state system class.
      
      \param argc The main argc variable supplied to main(), holding
      the number of available command line parameters issues when
      running the system.
      
      \param argv The main argv variable supplied to main(), holding
      the actual available command line parameters issues when running
      the system. */
    SemanticHierarchyRecorder(int argc, char** argv);
    /*! \brief Destructor of the main belief state system class.
      
      Deletes all instances of internal objects. */
    ~SemanticHierarchyRecorder();
    
    /*! \brief Returns the current version of the belief state system software.
      
      The version string can contain a version number, and additional
      content, describing the exact version present.
    
      \return Software version string */
    std::string version();
    
    /*! \brief Main initialization method for the belief state system.
      
      Initializes the default search paths for configuration files,
      creates instances for the PluginSystem and triggers the loading
      of configuration files and plugins (as configured).
      
      \param strConfigFile Overrides the default plugin search
      mechanism and uses the file specified in this parameter. If this
      fails, the function falls back to the default mechanism.
    
      \return Result construct, describing the outcome of the operation */
    Result init(std::string strConfigFile = "");
    
    /*! \brief Deinitialization method for the belief state system
      
      Dummy deinitialization method, does not deinitialize any
      components. Implemented for the sake of completeness and to make
      subclassing easier in which actual deinitialization mechanisms
      might occur.
    
      \return Result construct, describing the outcome of the operation */
    Result deinit();
    
    /*! \brief Loads the specified configuration file
      
      Loads the libconfig-styled configuration file specified in the
      parameter. Will update all components in the main belief state
      system instance that are affected by the configured options.
      
      \param strConfigFile Configuration file path to open for loading
      configuration
    
      \return Boolean that signals whether loading of the specified
      configuration file succeeded or failed. */
    bool loadConfigFile(std::string strConfigFile);
    
    /*! \brief Loads the plugin-specific configuration portion of the configuration file
      
      Every named plugin can be configured individually from the main
      configuration file. This function (recursively) evaluates the
      specified options to allow for arbitrary tree-like configuration
      of every plugin.
      
      \param sBranch The libconfig Setting which denotes the current
      configuration group for the current plugin.
      
      \param ckvpInto The KeyValuePair instance into which the loaded
      configuration branch should be parsed.
      
      \param strConfigPath The current path in the nested
      configuration groups. This is for displaying nice output only.
      
      \param bIgnorePluginField Ignore the 'plugin' field in the
      current branch. This is only used on the first level, as that
      level holds the plugin name of the plugin to configure.
    
      \return Boolean that signals whether loading of the individjal
      plugin configuration branch succeeded or failed. */
    bool loadIndividualPluginConfigurationBranch(libconfig::Setting &sBranch, KeyValuePair* ckvpInto, std::string strConfigPath = "", bool bIgnorePluginField = false);
    
    /*! \brief Spreads incoming system events to all subscribing plugins.
      
      System events are being accumulated in the system event queue
      within the main SemanticHierarchyRecorder class instance. They are then
      distributed among the plugins that subscribed to their specific
      event type.
      
      \param evEvent The event to spread among subscribers
      
      \return Boolean signalling whether at least one subscriber
      consumes the event. */
    bool spreadEvent(Event evEvent);
    
    /*! \brief Spread incoming system service events to all subscribing plugins.
      
      System service events are special events that require a reply
      from potential receivers. This function spreads a service event
      request, and its corresponding reply, to the respective plugin
      instances.

      \param seServiceEvent The service event to spread among
      subscribers (and receivers of replies). */
    void spreadServiceEvent(ServiceEvent seServiceEvent);
    
    /*! \brief Main cycle method, maintaining core functionality for the system.
      
      The main cycle function triggers processing of event pipelines,
      and forwards messages between plugins, utilizing functions
      present in the SemanticHierarchyRecorder class. This function is being called
      from outside the function in a loop.
      
      \return Boolean signallig whether the current cycle completed
      successfully, or if there was a problem that needs further
      inspection. */
    bool cycle();
    
    /*! \brief Triggers the recorder system shutdown
      
      Sets the m_bRun class member to false, effectively letting the
      main loop cycle() method stop execution and return from its
      blocking call. */
    void triggerShutdown();
    
    /*! \brief Triggers the processing of a terminal resize event. */
    void triggerTerminalResize();
    
    void setBaseDataDirectory(std::string strBaseDataDirectory);
    std::string baseDataDirectory();
    std::list<std::string> resolveDirectoryTokens(std::string strPath);
    
    /*! \brief Returns the current set workspace directory
      
      The workspace directory is a globally set path that can be used
      as a reference point for directory tokens. It is referenced as
      $WORKSPACE. In this function, the directory manually set in the
      config.cfg file is returned, or an empty string if it is not set
      there. */
    virtual std::list<std::string> workspaceDirectories();
    
    /*! \brief Returns the current user's home directory
      
      Returns whatever is stored in the environmental variable
      '${HOME}'. */
    std::string homeDirectory();
    
    virtual std::list<std::string> findTokenReplacements(std::string strToken);
    bool handleUnhandledEvent(Event evEvent);
    
    std::list<std::string> findPrefixPaths(std::string strPathList, std::string strMatchingSuffix, std::string strDelimiter = ":");
    
    virtual std::vector<std::string> getAdditionalSearchPaths();
  };
}


#endif /* __SEMANTIC_HIERARCHY_RECORDER_H__ */
