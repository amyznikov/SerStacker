/*
 * run-loop.h
 *
 *  Created on: Apr 11, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __run_loop_h__
#define __run_loop_h__

#if HAVE_TBB
#include <tbb/tbb.h>

template<typename T, typename Func>
static inline void parallel_for(T start, T end, Func && f)
{
  tbb::parallel_for(tbb::blocked_range<int>(start, end), std::forward<Func>(f),
      tbb::static_partitioner());
}
static inline int rbegin(const tbb::blocked_range<int> & range)
{
  return range.begin();
}
static inline int rend(const tbb::blocked_range<int> & range)
{
  return range.end();
}

#else
#include <opencv2/opencv.hpp>

template<typename T, typename Func>
static inline void parallel_for(T start, T end, Func && f)
{
  cv::parallel_for_(cv::Range(start, end), f);
}
static inline int rbegin(const cv::Range & range)
{
  return range.start;
}
static inline int rend(const cv::Range & range)
{
  return range.end;
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
