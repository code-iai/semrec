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


#ifndef __NODE_H__
#define __NODE_H__


// System
#include <string>

// Other
#include <designators/KeyValuePair.h>


using namespace designator_integration;


namespace semrec {
  /*! \brief Class describing a single node entity
    
    Nodes are the basic building block of task trees. They can have
    (meta-)properties, a title, a unique ID, and sub nodes. They also
    know about their parent node to ease tree processing. */
  class Node {
  private:
    /*! \brief The string title of the current node
      
      Mostly representing its task- or goal-context. */
    std::string m_strTitle;
    /*! \brief Unique ID for this node
      
      Mostly important when exporting tree data (such as .owl files)
      in order to give every individual node a unique identity. Is
      set automatically by the system. */
    std::string m_strUniqueID;
    /*! \brief Internal counter ID
      
      This represents this node's identity during the current run of
      the system. This is a volatile value and should not be used for
      external node reference. */
    int m_nID;
    
    /*! \brief Pointer to the node's parent node instance */
    Node* m_ndParent;
    /*! \brief List of description key/value-pairs
     
     This description is information set by the external system that
     triggers logging mechanisms. This is custom data and its fields
     can be set via the designator interface. */
    std::list<KeyValuePair*> m_lstDescription;
    /*! \brief List of this node's sub-nodes */
    std::list<Node*> m_lstSubnodes;
    /*! \brief This nide's meta information
     
     This meta-information includes when a node context was started
     and ended (timestamps), what its success value was, and similar
     meta-information. This is automatically set by the system. */
    KeyValuePair* m_ckvpMetaInformation;
    
    /*! \brief List of failures caught by this node, and their emitters
      
      Nodes can act as failure handling nodes. When the external plan
      system signals that a node caught a formerly thrown failure, the
      respective emitter and the failure type are denoted here. */
    std::list< std::pair<std::string, Node*> > m_lstCaughtFailures;
    
    /*! \brief Central initialization method for the Node class */
    void init();
    
    /*! \brief Deletes the internal description cache for this node
      
      This is a clean-up method and is called by the system
      automatically. */
    void clearDescription();
    /*! \brief Deletes the internal sub-node references for this node
      
      This is a clean-up method and is called by the system
      automatically. */
    void clearSubnodes();
    
  public:
    /*! \brief Constructur for generating an empty node */
    Node();
    /*! \brief Constructor for generating a title'd node */
    Node(std::string strTitle);
    /*! \brief Constructur for generating a node given an existing
        description */
    Node(std::list<KeyValuePair*> lstDescription);
    /*! \brief Destructor, cleaning up the internal node data */
    ~Node();
    
    /*! \brief Sets this node's description
      
      The formerly present description is replaced by the one given as
      lstDescription.
      
      \param lstDescription The description to replace the current one with */
    void setDescription(std::list<KeyValuePair*> lstDescription);
    /*! \brief Returns the node's current description
      
      \return List of key/value-pairs denoting the node's current description fields. */
    std::list<KeyValuePair*> description();
    
    /*! \brief Set this node's title
      
      The (string) title of a node can for example be its task- or
      goal-context.
      
      \param strTitle The title to assign to this node */
    void setTitle(std::string strTitle);
    /*! \brief Return this node's title
      
      \return The node's current title string */
    std::string title();
    
    /*! \brief Set this node's pointer to its parent node
      
      \param ndParent The node to set as this node's parent instance */
    void setParent(Node* ndParent);
    /*! \brief Return this node's parent node
      
      \return The pointer to this node's parent node instance */
    Node* parent();
    Node* relativeWithID(int nID, bool bIgnoreSelf = false);
    
    /*! \brief Add a sub-node to this node
      
      To build up a tree structure, nodes can hold sub-nodes to
      reflect branching. This function adds a Node instance to the
      sub-branches of this node.
      
      \param ndAdd The node instance to add as branch */
    void addSubnode(Node* ndAdd);
    /*! \brief Return the list of sub-nodes for this node
      
      \return List of Node instances that are held as branching nodes for this Node instance */
    std::list<Node*> subnodes();
    
    void setUniqueID(std::string strUniqueID);
    std::string uniqueID();
    
    void setID(int nID);
    int id();
    
    int highestID();
    
    bool includesUniqueID(std::string strUniqueID);
    
    KeyValuePair* metaInformation();
    
    void setPrematurelyEnded(bool bPrematurelyEnded);
    bool prematurelyEnded();
    
    KeyValuePair* addDescriptionListItem(std::string strDomain, std::string strPrefix);
    std::string addImage(std::string strOrigin, std::string strFilename, std::string strTimestamp);
    std::string addObject(std::list<KeyValuePair*> lstDescription);
    std::string addFailure(std::string strCondition, std::string strTimestamp);
    std::string catchFailure(std::string strFailureID, Node* ndEmitter, std::string strTimestamp);
    void removeCaughtFailure(std::string strFailureID);
    Node* emitterForCaughtFailure(std::string strFailureID, std::string strEmitterID, std::string strTimestamp);
    bool hasFailures();
    void addDesignator(std::string strType, std::list<KeyValuePair*> lstDescription, std::string strUniqueID, std::string strAnnotation = "");
    
    void setSuccess(bool bSuccess);
    void setSuccess(int nSuccess);
    bool success();
    
    Node* previousNode();
    
    void ensureProperty(std::string strKey, std::string strDefaultValue);
  };
}


#endif /* __NODE_H__ */
