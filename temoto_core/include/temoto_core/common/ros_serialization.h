
#include <ros/serialization.h>
#include <vector>
#include <iostream>

namespace ser = ros::serialization;

namespace temoto_core
{
/**
 * @brief Serialize a ROS msg class into a uint8_t vector
 * 
 * @tparam payload 
 * @param payload 
 * @return std::vector<uint8_t> 
 */
template <class PayloadType>
std::vector<uint8_t> serializeROSmsg(const PayloadType& payload)
{
  // Serialize the payload
  uint32_t payload_size = ros::serialization::serializationLength(payload);
  boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);
  ser::OStream stream(buffer.get(), payload_size);
  ser::serialize(stream, payload);

  // Create a byte array
  std::vector<uint8_t> payload_byte_array;

  // Fill out the byte array
  for (uint32_t i=0; i<payload_size; i++)
  {
    payload_byte_array.push_back(buffer.get()[i]);
  }
  return payload_byte_array;
}

/**
 * @brief Deserialize an uint8_t vector into a ROS message
 * 
 * @tparam payload 
 * @param payload 
 * @return std::vector<uint8_t> 
 */
template <class PayloadType>
PayloadType deserializeROSmsg(std::vector<uint8_t> payload_in)
{
  uint32_t payload_size = payload_in.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);

  // Fill buffer with the serialized payload
  for (uint32_t i=0; i<payload_size; i++)
  {
    (buffer.get())[i] = payload_in[i];
  }

  // Convert the serialized payload to msg
  ser::IStream stream(buffer.get(), payload_size);
  ser::deserialize(stream, payload);
  return payload;
}
}