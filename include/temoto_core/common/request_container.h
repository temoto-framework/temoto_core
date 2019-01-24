#ifndef TEMOTO_CORE__REQUEST_CONTAINER_H
#define TEMOTO_CORE__REQUEST_CONTAINER_H

#include "temoto_core/common/temoto_id.h"
#include <vector>
#include <string>
#include <utility>

namespace temoto_core
{

template <class T> class RequestContainer
{
public:

    /**
     * @brief request_
     */
    T request_;

    /**
     * @brief IDs_
     */
    std::vector <temoto_core::temoto_id::ID> IDs_;

    /**
     * @brief RequestContainer
     * @param request
     * @param ID
     */
    RequestContainer (T request, temoto_core::temoto_id::ID ID)
    {
        request_ = request;
        addID(ID);
    }

    /**
     * @brief addID
     * @param ID
     */
    void addID (temoto_core::temoto_id::ID ID)
    {
        IDs_.push_back(ID);
    }

    /**
     * @brief removeID
     * @param ID
     */
    void removeID (temoto_core::temoto_id::ID ID)
    {
        // Find the index of the ID
        for (auto id=IDs_.begin() ; id < IDs_.end(); id++)
        {
            if (*id == ID)
            {
                IDs_.erase(id);
                break;
            }
        }
    }

    /**
     * @brief getReq
     * @return
     */
    typename T::Request getReq()
    {
        return request_.request;
    }

    /**
     * @brief getRes
     * @return
     */
    typename T::Response getRes()
    {
        return request_.response;
    }
};

/*
template <class T> class ContainedRequests
{
public:

    ContainedRequests(){};

private:

    std::vector <RequestContainer<T>> contained_requests_;
};
*/

} // temoto_core namespace

#endif
