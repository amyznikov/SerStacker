/*
 * run-loop.h
 *
 *  Created on: Apr 11, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __run_loop_h__
#define __run_loop_h__

#include <atomic>
#include <utility>
#include <vector>

// #undef HAVE_TBB
#if HAVE_TBB
#include <tbb/tbb.h>

template<typename T, typename Func>
static inline void parallel_for(T start, T end, Func && f)
{
  tbb::parallel_for(tbb::blocked_range<int>(start, end), std::forward<Func>(f),
      tbb::static_partitioner());
}
template<typename T, typename Func>
static inline void no_parallel_for(T start, T end, Func && f)
{
  for ( ; start < end; ++start ) {
    (std::forward<Func>(f)(tbb::blocked_range<T>(start, start + 1)));
  }
}
static inline int rbegin(const tbb::blocked_range<int> & range)
{
  return range.begin();
}
static inline int rend(const tbb::blocked_range<int> & range)
{
  return range.end();
}

/**
* @brief Abstract wrapper for parallel reduction (TBB branch)
*
* @tparam RangeType Index type (int, size_t)
* @tparam Reducer User-defined reducer class implementing the TBB interface:
*                 - Split constructor: Reducer(Reducer&, tbb::split)
*                 - Join method: void join(const Reducer&)
*                 - Function call operator: void operator()(const tbb::blocked_range<int>&)
*/
using cv_reduction_splitter = tbb::split;

template<typename RangeType, typename Reducer>
static inline void parallel_reduce(RangeType start, RangeType end, Reducer& reducer)
{
  tbb::parallel_reduce(tbb::blocked_range<RangeType>(start, end), reducer, tbb::static_partitioner());
}


#else
#include <opencv2/opencv.hpp>

template<typename T, typename Func>
static inline void parallel_for(T start, T end, Func && f)
{
  cv::parallel_for_(cv::Range(start, end), f);
}
template<typename T, typename Func>
static inline void no_parallel_for(T start, T end, Func && f)
{
  for ( ; start < end; ++start ) {
    (std::forward<Func>(f)(cv::Range(start, start + 1)));
  }
}
static inline int rbegin(const cv::Range & range)
{
  return range.start;
}
static inline int rend(const cv::Range & range)
{
  return range.end;
}

// Placeholder type for OpenCV
struct cv_reduction_splitter {};

// Simulate the tbb::blocked_range structure for OpenCV so that the reducer accepts the same range type.
template<typename T>
class cv_blocked_range {
public:
  cv_blocked_range(T begin, T end) : _begin(begin), _end(end) {}
  T begin() const { return _begin; }
  T end() const { return _end; }
private:
  T _begin, _end;
};

template<typename T> static inline
T rbegin(const cv_blocked_range<T> & range)
{
  return range.begin();
}
template<typename T> static inline
T rend(const cv_blocked_range<T> & range)
{
  return range.end();
}

/**
* @brief Abstract wrapper to simulate TBB parallel reduction with OpenCV threads
*/
template<typename RangeType, typename Reducer>
static inline void parallel_reduce(RangeType start, RangeType end, Reducer & rootReducer)
{
  const int maxThreads = cv::getNumThreads();
  std::vector<Reducer*> localReducers(maxThreads, nullptr);
  cv_reduction_splitter splitter;

  cv::parallel_for_(cv::Range(start, end), [&](const cv::Range & range) {
    const int threadIdx = cv::getThreadNum();
    Reducer* currentReducer = nullptr;

    if (threadIdx == 0 || threadIdx >= maxThreads) {
      currentReducer = &rootReducer;
    }
    else {
      if (localReducers[threadIdx] == nullptr) {
        localReducers[threadIdx] = new Reducer(rootReducer, splitter);
      }
      currentReducer = localReducers[threadIdx];
    }

    (*currentReducer)(cv_blocked_range<RangeType>(range.start, range.end));
  });

  // Sequential join
  for( int i = 0; i < maxThreads; ++i ) {
    if( localReducers[i] != nullptr ) {
      rootReducer.join(*localReducers[i]);
      delete localReducers[i];
    }
  }
}
#endif

template<typename T, typename Func>
static inline void parallel_loop(T start, T end, Func&& f)
{
#if HAVE_TBB
    tbb::parallel_for(start, end, std::forward<Func>(f), tbb::static_partitioner());
#else
    cv::parallel_for_(cv::Range(start, end), [&](const cv::Range & range) {
      for (int idx = range.start; idx < range.end; ++idx) {
        f(idx);
      }
    });
#endif
}


template<typename... Fs>
void parallel_invoke(Fs&&... fs) {
#if HAVE_TBB
  tbb::parallel_invoke(std::forward<Fs>(fs)...);
#else
    // C++17 Fold Expression
    // Sequentially expands and calls each passed function object.
   (std::forward<Fs>(fs)(), ...);
#endif
}



#endif /* __run_loop_h__ */
