#ifndef COMMON_MAKE_UNIQUE_H_
#define COMMON_MAKE_UNIQUE_H_

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace common{

template <class T>
struct _Unique_if
{
  typedef std::unique_ptr<T> _Single_object;
};

template <class T, class... Args>
typename _Unique_if<T>::_Single_object make_unique(Args&&... args) 
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}//namespace common

#endif