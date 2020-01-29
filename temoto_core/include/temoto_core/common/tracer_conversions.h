#ifndef TEMOTO_CORE__TRACER_CONVERSIONS_H
#define TEMOTO_CORE__TRACER_CONVERSIONS_H

#include <diagnostic_msgs/KeyValue.h>
#include <unordered_map>
#include <vector>
#include <string>

namespace temoto_core
{

typedef std::unordered_map<std::string, std::string> StringMap;
typedef std::vector<diagnostic_msgs::KeyValue> KeyValues;

/**
 * @brief 
 * 
 * @param string_map 
 * @return KeyValues 
 */
KeyValues unorderedMapToKeyValues(const StringMap& string_map);

/**
 * @brief 
 * 
 * @param key_values 
 * @return StringMap 
 */
StringMap keyValuesToUnorderedMap(const KeyValues& key_values);
} // temoto_core namespace

#endif