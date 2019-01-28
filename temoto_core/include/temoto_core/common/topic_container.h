#ifndef TEMOTO_CORE__TOPIC_CONTAINER_H
#define TEMOTO_CORE__TOPIC_CONTAINER_H

#include "diagnostic_msgs/KeyValue.h"

namespace temoto_core
{

typedef std::pair<std::string, std::string> StringPair;

/**
 * @brief The AlgorithmTopics class
 */
class TopicContainer
{
public:

  void addInputTopic(const std::string& type, const std::string& topic)
  {
    input_topics_.emplace_back(std::make_pair(type, topic));
  }

  void addInputTopicType(const std::string& type)
  {
    addInputTopic(type, "");
  }

  void clearInputTopics()
  {
    input_topics_.clear();
  }


  void addOutputTopic(std::string type, std::string topic)
  {
    output_topics_.emplace_back(std::make_pair(type, topic));
  }

  void addOutputTopicType(const std::string& type)
  {
    addOutputTopic(type, "");
  }

  void clearOutputTopics()
  {
    output_topics_.clear();
  }


  std::vector<StringPair> getInputTopics() const
  {
    return input_topics_;
  }

  std::vector<StringPair> getOutputTopics() const
  {
    return output_topics_;
  }

  // Get input topic
  std::string getInputTopic(const std::string& type) const
  {
    return getTopicByType(type, input_topics_);
  }

  // Get output topic
  std::string getOutputTopic(const std::string& type) const
  {
    return getTopicByType(type, output_topics_);
  }

  // Returns input topics as a key/value vector
  std::vector<diagnostic_msgs::KeyValue> inputTopicsAsKeyValues() const
  {
    return asKeyValues(input_topics_);
  }

  // Returns output topics as a key/value vector
  std::vector<diagnostic_msgs::KeyValue> outputTopicsAsKeyValues() const
  {
    return asKeyValues(output_topics_);
  }

  // Set input topics by key/value vector
  void setInputTopicsByKeyValue(std::vector<diagnostic_msgs::KeyValue>& v_kv)
  {
    setTopicsByKeyValue(v_kv, input_topics_);
  }

  // Set output topics by key/value vector
  void setOutputTopicsByKeyValue(std::vector<diagnostic_msgs::KeyValue>& v_kv)
  {
    setTopicsByKeyValue(v_kv, output_topics_);
  }

  // Set input topics by string pair
  void setInputTopics(std::vector<StringPair> input_topics)
  {
    input_topics_ = input_topics;
  }

  // Set output topics by string pair
  void setOutputTopics(std::vector<StringPair> output_topics)
  {
    output_topics_ = output_topics;
  }

private:
  std::vector<StringPair> input_topics_;
  std::vector<StringPair> output_topics_;

  // Get topic by type
  std::string getTopicByType(const std::string& type, const std::vector<StringPair>& topics) const
  {
    // Loop over the topics and check the type match. Return the topic if the types amatch
    for(auto&topic : topics)
    {
      if(topic.first == type)
      {
        return topic.second;
      }
    }

    return std::string();
  }

  // Return stringpair type topic vector as key/value vector
  std::vector<diagnostic_msgs::KeyValue> asKeyValues(const std::vector<StringPair>& v_sp) const
  {
    std::vector<diagnostic_msgs::KeyValue> v_kv;

    for (auto& sp : v_sp)
    {
      diagnostic_msgs::KeyValue kv;
      kv.key = sp.first;
      kv.value = sp.second;
      v_kv.push_back(std::move(kv));
    }

    return v_kv;
  }

  // Set the stringpair type topic vector by key/value vector
  void setTopicsByKeyValue(std::vector<diagnostic_msgs::KeyValue>& v_kv, std::vector<StringPair>& v_sp)
  {
    // Clear the vector before adding new elements
    v_sp.clear();

    for (auto& kv : v_kv)
    {
      v_sp.push_back(StringPair{kv.key, kv.value});
    }
  }

};

} // temoto_core namespace

#endif