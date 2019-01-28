#ifndef TEMOTO_CORE__TEMOTO_ID_H
#define TEMOTO_CORE__TEMOTO_ID_H

#include <stdlib.h>
#include <chrono>

namespace temoto_core
{
namespace temoto_id
{

typedef int ID;
const ID UNASSIGNED_ID = 0;

class IDManager
{
public:
  IDManager()
  {
    // generate random seed for more reliable testing
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    srand (ms.count());
    current_ID_ = rand() % 1000;
  }
  /**
   * @brief checkID
   * @param ID_in
   * @return
   */
  ID checkID(ID ID_in)
  {
    if (ID_in == UNASSIGNED_ID)
    {
      return generateID();
    }
    else
    {
      return ID_in;
    }
  }

  /**
   * @brief generateID
   * @return
   */
  ID generateID()
  {
    incrementID();
    return current_ID_;
  }

private:
  /**
   * @brief current_ID_
   */
  ID current_ID_ = 2150;

  /**
   * @brief incrementID
   */
  void incrementID()
  {
    // Increment the id and if it is "0" then increment again
    if (++current_ID_ == UNASSIGNED_ID)
      ++current_ID_;
  }
};

} // temoto_id namespace
} // temoto_core namespace

#endif
