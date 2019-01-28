#ifndef TEMOTO_CORE__RELIABILITY_H
#define TEMOTO_CORE__RELIABILITY_H

#include <array>

namespace temoto_core
{
class Reliability
{
public:
  // Constructor
  Reliability();

  /**
   * \brief Reset reliability
   * \param reliability Sets the initial values for the reliability moving average filter.
   * The value has to be in range [0-1], 0 being not reliable at all and 1.0 is very
   * reliable.
   */
  void resetReliability(float reliability = 0.8);

  /**
   * \brief Adjust reliability
   * \param reliability the new reliability contribution to the moving average filter. The value has
   * to be in range [0-1], 0 being not reliable at all and 1.0 is very
   * reliable.
   */
  void adjustReliability(float reliability = 1.0);


  /**
   * @brief Get the filtered reliability.
   * @return Filtered reliability.
   */
  float getReliability() const
  {
    return reliability_;
  }
  
private:

  /**
   * @brief Reliability ratings of the robot.
   */
  std::array<float, 100> reliabilities_;

  /**
   * @brief Average reliability.
   */
  float reliability_;

  /**
   * @brief Reliability rating of the robot.
   */
  unsigned int reliability_idx_;

};
} // temoto_core namespace
#endif
