/*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file   thread_safe_map.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jun 16 09:58:59 2011
 *
 *
 * @brief  We needed a threadsafe hash map, with the possibility of multiple
 * readers accessing the map at the same time, but only one writer at a time.
 *
 *
 */

#ifndef _THREAD_SAFE_MAP_HPP_
#define _THREAD_SAFE_MAP_HPP_

#include <boost/smart_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/mutex.hpp>

#include <map>
#include <vector>
#include <iostream>
#include <utility>
#include <string>

namespace threadsafe
{
template<class T>
class Map
{
public:
  Map()
  {
    mutex_ = boost::shared_ptr<boost::shared_mutex>(new boost::shared_mutex());
    map_ = boost::shared_ptr<InternalMap>(new InternalMap());
  };

  ~Map()
  {
  };

  T find(std::string first)
  {
    boost::shared_lock<boost::shared_mutex> lock(*(mutex_.get()));

    typename InternalMap::iterator it = map_->find(first);
    if (it != map_->end())
      return it->second;
    else
      return T();
  }

  bool insert(const std::string &first, const T &value)
  {
    if (!mutex_->timed_lock(boost::posix_time::microseconds(lock_wait_time)))
    {
      return false;
    }

    keys_.push_back(first);
    map_->insert(std::pair<std::string, T>(first, value));
    mutex_->unlock();
    return true;
  }

  bool update(const std::string &first, const T &value)
  {
    if (!mutex_->timed_lock(boost::posix_time::microseconds(lock_wait_time)))
    {
      return false;
    }

    (*map_)[first] = value;
    mutex_->unlock();
    return true;
  }

  std::vector<std::string> keys()
  {
    return keys_;
  }

private:
  static const int lock_wait_time = 100;

  typedef std::map<std::string, T> InternalMap;

  boost::shared_ptr<InternalMap> map_;

  boost::shared_ptr<boost::shared_mutex> mutex_;
  std::vector<std::string> keys_;
};
}  // namespace threadsafe

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
