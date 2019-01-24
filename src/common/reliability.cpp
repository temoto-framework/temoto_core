#include "temoto_core/common/reliability.h"

namespace temoto_core
{

Reliability::Reliability()
{
  resetReliability(0.8);
}

// Adds a reliability contribution to a moving average filter
void Reliability::adjustReliability(float reliability)
{
  reliability = std::max(std::min(reliability, 1.0f), 0.0f);  // clamp to [0-1]
  ++reliability_idx_ %= reliabilities_.size();                 // rolling index

  // take out the last reliability
  reliability_ -= reliabilities_[reliability_idx_] / (float)reliabilities_.size();

  // insert new reliability value
  reliabilities_[reliability_idx_] = reliability;
  reliability_ += reliability / (float)reliabilities_.size();
}

void Reliability::resetReliability(float reliability)
{
  // Fill array with initial reliability values;
  reliabilities_.fill(reliability);  // buffer for instantaneous reliability values
  reliability_ = reliability;    // Filtered reliability is kept here
  reliability_idx_ = 0;
}

} // temoto_core namespace