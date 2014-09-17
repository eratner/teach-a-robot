#ifndef PERFORMANCE_H
#define PERFORMANCE_H

#include <sys/types.h>
#include <sys/sysinfo.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/times.h>
#include <sys/vtimes.h>
#include <fstream>

namespace demonstration_visualizer
{

class ProcessInfo
{
public:
  static std::ofstream STATS_OUT_FILE;

  static void writeStats(const std::string &out, bool flush = false);

  /**************************************************/
  /***************** Virtual memory *****************/
  /**************************************************/
  /// @brief Returns the total virtual memory available
  static long long getTotalVirtualMemory();

  /// @brief Returns the total amount of virutal memory currently in use
  static long long getUsedVirtualMemory();

  /// @brief Returns the amount of virtual memory currently in use by the calling process
  static int getProcessVirtualMemory();

  /**************************************************/
  /**************** Physical memory *****************/
  /**************************************************/
  /// @brief Returns the total physical memory available
  static long long getTotalPhysicalMemory();

  /// @brief Returns the total amount of physical memory currently in use
  static long long getUsedPhysicalMemory();

  /// @brief Returns the amount of physical memory currently in use by the calling process
  static int getProcessPhysicalMemory();

  /**************************************************/
  /********************** CPU ***********************/
  /**************************************************/
  /// @brief Should be called before any CPU computations
  static void initCPU();

  /// @brief Returns the total percentage of CPU currently in use
  static double getTotalCPU();

  /// @brief Returns the percentage of CPU currently in use by the calling process
  static double getProcessCPU();

private:
  /// For total CPU usage
  static unsigned long long last_total_user;
  static unsigned long long last_total_user_low;
  static unsigned long long last_total_sys;
  static unsigned long long last_total_idle;

  /// For current process CPU usage
  static int num_processors;
  static clock_t last_cpu;
  static clock_t last_sys_cpu;
  static clock_t last_user_cpu;

  ProcessInfo();

  static int parseLine(char *line);

  static int getProcessValue(const char *s, int n);

};

} // namespace demonstration_visualizer

#endif // PERFORMANCE_H
