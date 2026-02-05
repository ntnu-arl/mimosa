// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <chrono>

class ChronoStopwatch
{
private:
  std::chrono::time_point<std::chrono::steady_clock> start_;
  std::chrono::time_point<std::chrono::steady_clock> ticked_;

public:
  ChronoStopwatch() { reset(); }

  /**
   * @brief Get the elapsed time since the last reset in seconds
   */
  float elapsed()
  {
    std::chrono::time_point<std::chrono::steady_clock> current = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed_seconds = current - start_;
    return elapsed_seconds.count();
  }

  /**
   * @brief Get the elapsed time since the last reset in milliseconds
   */
  float elapsedMs() { return elapsed() * 1000.0f; }

  /**
   * @brief Reset the stopwatch and return the elapsed time since the last reset.
   */
  float lap()
  {
    std::chrono::time_point<std::chrono::steady_clock> stop = std::chrono::steady_clock::now();
    std::chrono::duration<float> lap_seconds = stop - start_;
    start_ = stop;
    return lap_seconds.count();
  }

  float lapMs() { return lap() * 1000.0f; }

  /**
   * @brief Get the elapsed time since the last tick in seconds
   */
  float tick()
  {
    std::chrono::time_point<std::chrono::steady_clock> current = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed_seconds = current - ticked_;
    ticked_ = current;
    return elapsed_seconds.count();
  }

  float tickMs() { return tick() * 1000.0f; }

  void reset()
  {
    start_ = std::chrono::steady_clock::now();
    ticked_ = start_;
  }
};

#include <omp.h>

class OMPStopwatch
{
private:
  double start_;
  double ticked_;

public:
  OMPStopwatch() { reset(); }
  ~OMPStopwatch() {}

  /**
   * @brief Get the elapsed time since the last reset in seconds
   */
  double elapsed() { return omp_get_wtime() - start_; }
  double elapsedMs() { return elapsed() * 1000.0; }
  double lap()
  {
    double stop = omp_get_wtime();
    double lap_seconds = stop - start_;
    start_ = stop;
    return lap_seconds;
  }
  double lapMs() { return lap() * 1000.0; }
  double tick()
  {
    double current = omp_get_wtime();
    double elapsed_seconds = current - ticked_;
    ticked_ = current;
    return elapsed_seconds;
  }
  double tickMs() { return tick() * 1000.0; }
  void reset()
  {
    start_ = omp_get_wtime();
    ticked_ = start_;
  }
};

#if USE_OPENMP
using Stopwatch = OMPStopwatch;
#else
using Stopwatch = ChronoStopwatch;
#endif
