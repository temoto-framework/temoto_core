/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *	Sample Task class that utilizes the Temoto 2.0 architecture.
 *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Things that have to be included
#include "temoto_nlp/base_task/base_task.h"                 				 // The base task
#include <class_loader/class_loader.h>                                   // Class loader includes

// Task specific includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// First implementaton
class TaAddNumbers: public temoto_nlp::BaseTask
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * *
 * Inherited methods that have to be implemented /START
 * * * * * * * * * * * * * * * * * * * * * * * * */

TaAddNumbers()
{
    // Do something here if needed
    TASK_INFO("TaAddNumbers constructed");
}

// startTask with arguments
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
    temoto_nlp::Subject numeric_0_in = temoto_nlp::getSubjectByType("numeric", input_subjects);
    std::string  numeric_0_word_in = numeric_0_in.words_[0];
    float        numeric_0_data_0_in = boost::any_cast<float>(numeric_0_in.data_[0].value);

    temoto_nlp::Subject numeric_1_in = temoto_nlp::getSubjectByType("numeric", input_subjects);
    std::string  numeric_1_word_in = numeric_1_in.words_[0];
    float        numeric_1_data_0_in = boost::any_cast<float>(numeric_1_in.data_[0].value);

    std::string  numeric_0_word_out;
    float        numeric_0_data_0_out;

    // </ AUTO-GENERATED, DO NOT MODIFY >

// -------------------------------< USER CODE >-------------------------------

    numeric_0_data_0_out = numeric_0_data_0_in + numeric_1_data_0_in;
    numeric_0_word_out = numeric_0_word_in;

    TASK_INFO("TaAddNumbers MOD_5: %f %s + %f %s = %f %s", numeric_0_data_0_in, numeric_0_word_in.c_str()
                                              , numeric_1_data_0_in, numeric_1_word_in.c_str()
                                              , numeric_0_data_0_out, numeric_0_word_out.c_str());

// -------------------------------</ USER CODE >-------------------------------

    // < AUTO-GENERATED, DO NOT MODIFY >

    temoto_nlp::Subject numeric_0_out("numeric", numeric_0_word_out);
    numeric_0_out.markComplete();
    numeric_0_out.data_.emplace_back("number", boost::any_cast<float>(numeric_0_data_0_out));
    output_subjects.push_back(numeric_0_out);

    // </ AUTO-GENERATED, DO NOT MODIFY >
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

~TaAddNumbers()
{
    TASK_INFO("TaAddNumbers destructed");
}


};

// Dont forget that part, otherwise this class would not be loadable
CLASS_LOADER_REGISTER_CLASS(TaAddNumbers, temoto_nlp::BaseTask);
