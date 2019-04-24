
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://utnuclearroboticspublic.github.io/temoto2
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include "temoto_nlp/base_task/base_task.h" // The base task
#include <class_loader/class_loader.h>      // Class loader includes
#include "temoto_component_manager/component_manager_interface.h"

/* 
 * ACTION IMPLEMENTATION of TaOpenTerminal 
 */
class TaOpenTerminal : public temoto_nlp::BaseTask
{
public:
  /* REQUIRED BY TEMOTO */
  TaOpenTerminal()
  {
    // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
    TEMOTO_INFO("TaOpenTerminal constructed");
  }

  /* REQUIRED BY TEMOTO */
  void startTask(temoto_nlp::TaskInterface task_interface)
  {
    input_subjects = task_interface.input_subjects_;
    switch (task_interface.id_)
    {

    // Interface 0
    case 0:
      startInterface_0();
      break;
    }
  }

  /* REQUIRED BY TEMOTO */
  std::vector<temoto_nlp::Subject> getSolution()
  {
    return output_subjects;
  }

  ~TaOpenTerminal()
  {
    TEMOTO_INFO("TaOpenTerminal destructed");
  }

  /********************* END OF REQUIRED PUBLIC INTERFACE *********************/

private:
  // Nodehandle
  ros::NodeHandle n_;

  // Create sensor manager interface object for accessing sensor manager
  temoto_component_manager::ComponentManagerInterface<TaOpenTerminal> cpmi_;

  /*
   * Interface 0 body
   */
  void startInterface_0()
  {
    /* EXTRACTION OF INPUT SUBJECTS */
    temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
    std::string what_0_word_in = what_0_in.words_[0];

    cpmi_.initialize(this);

    std::string temoto_namespace = temoto_core::common::getTemotoNamespace();
    ComponentTopicsReq requested_topics;
    // requested_topics.addOutputTopic("chatter_data", "/" + temoto_namespace + "/human_chatter");
    requested_topics.addOutputTopic("chatter_data", "/human_chatter");

    TEMOTO_INFO("Trying to open the human chatter terminal");

    ComponentTopicsRes responded_topics = cpmi_.startComponent("human chatter", requested_topics);
    std::string chatter_topic = responded_topics.getOutputTopic("chatter_data");

    TEMOTO_INFO_STREAM("Receiving human chatter on topic '" << chatter_topic << "'");
  }

}; // TaOpenTerminal class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaOpenTerminal, temoto_nlp::BaseTask);
