/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Task that calls the "index_tasks" service of the Task Handler
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "temoto_nlp/base_task/base_task.h"                  				 // The base task
#include <class_loader/class_loader.h>               // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "ros/package.h"
#include "temoto_nlp/IndexTasks.h"


// First implementaton
class TaskIndex: public temoto_nlp::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

TaskIndex()
{
    index_tasks_publisher_ = n_.advertise<temoto_nlp::IndexTasks>("index_tasks", 10);
    ROS_INFO("TaskIndex constructed");
}

/*
 * startTask
 */
void startTask(temoto_nlp::TaskInterface task_interface)
{
// * AUTO-GENERATED, DO NOT MODIFY *
    input_subjects = task_interface.input_subjects_;

    switch(task_interface.id_)
    {
    // Interface 0
    case 0:
        startInterface_0();
    break;
    }
// * AUTO-GENERATED, DO NOT MODIFY *
}

/*
 * Interface 0 body
 */
void startInterface_0()
{
    // < AUTO-GENERATED, DO NOT MODIFY >

    // Extracting input subjects
    temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
    std::string  what_0_word_in = what_0_in.words_[0];

    // </ AUTO-GENERATED, DO NOT MODIFY >

// --------------------------------< USER CODE >-------------------------------

    std::cout << "  TaskIndex: Indexing the tasks '" << what_0_word_in << "'\n";

    // Create a service message
    temoto_nlp::IndexTasks index_tasks_msg;
    index_tasks_msg.directory = ros::package::getPath(ROS_PACKAGE_NAME) + "/..";

    // Publish
    index_tasks_publisher_.publish(index_tasks_msg);

// --------------------------------</ USER CODE >-------------------------------
}

std::string getStatus()
{
    std::string str = "healthy";
    return str;
}

std::vector<temoto_nlp::Subject> getSolution()
{
    return output_subjects;
}

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented / END
 * * * * * * * * * * * * * * * * * * * * * * * * */

~TaskIndex()
{
    ROS_INFO("[TaskIndex::~TaskIndex] TaskIndex destructed");
}

private:

    ros::NodeHandle n_;
    ros::Publisher index_tasks_publisher_;
};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaskIndex, temoto_nlp::BaseTask);
