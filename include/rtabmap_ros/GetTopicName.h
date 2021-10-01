/*
 * GetTopicName.h
 *
 *  Created on: Oct 1, 2021
 *      Author: mathieu
 */

#ifndef INCLUDE_RTABMAP_ROS_GETTOPICNAME_H_
#define INCLUDE_RTABMAP_ROS_GETTOPICNAME_H_

#include <string>

template<class T>
auto getTopicNameImpl(T const& obj, int)
    -> decltype(obj->get_topic_name(), std::string())
{
   return obj->get_topic_name();
}

template<class T>
auto getTopicNameImpl(T const& obj, long)
    -> decltype(obj.getTopic(), std::string())
{
  return obj.getTopic();
}

template<class T>
auto getTopicName(T const& obj)
    -> decltype(getTopicNameImpl(obj, 0), std::string())
{
  return getTopicNameImpl(obj, 0);
}


#endif /* INCLUDE_RTABMAP_ROS_GETTOPICNAME_H_ */
