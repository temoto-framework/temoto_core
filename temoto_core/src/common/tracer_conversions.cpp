#include "temoto_core/common/tracer_conversions.h"

namespace temoto_core
{

KeyValues unorderedMapToKeyValues(const StringMap& string_map)
{
  KeyValues key_values;
  for (const auto& string_map_entry : string_map)
  {
    diagnostic_msgs::KeyValue kv;
    kv.key = string_map_entry.first;
    kv.value = string_map_entry.second;
    key_values.push_back(kv);
  }
  return key_values;
}

StringMap keyValuesToUnorderedMap(const KeyValues& key_values)
{
  StringMap string_map;
  for (const auto& key_value : key_values)
  {
    string_map.insert({key_value.key, key_value.value});
  }
  return string_map;
}
} // temoto_core namespace