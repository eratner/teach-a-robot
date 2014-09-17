#ifndef PERFORMANCE_H
#define PERFORMANCE_H

#include <sys/types.h>
#include <sys/sysinfo.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/times.h>
#include <sys/vtimes.h>

namespace demonstration_visualizer
{

class ProcessInfo
{
public:
  /**************************************************/
  /***************** Virtual memory *****************/
  /**************************************************/
  /// @brief Returns the total virtual memory available
  static long long getTotalVirtualMemory()
  {
    struct sysinfo memory_info;
    sysinfo(&memory_info);
    long long total_virtual_mem = memory_info.totalram;
    total_virtual_mem += memory_info.totalswap;
    total_virtual_mem *= memory_info.mem_unit;
    return total_virtual_mem;
  }

  /// @brief Returns the total amount of virutal memory currently in use
  static long long getUsedVirtualMemory()
  {
    struct sysinfo memory_info;
    sysinfo(&memory_info);
    long long used_virtual_mem = memory_info.totalram - memory_info.freeram;
    used_virtual_mem += (memory_info.totalswap - memory_info.freeswap);
    used_virtual_mem *= memory_info.mem_unit;
    return used_virtual_mem;
  }

  /// @brief Returns the amount of virtual memory currently in use by the calling process
  static int getProcessVirtualMemory()
  {
    return getProcessValue("VmSize:", 7);
  }

  /**************************************************/
  /**************** Physical memory *****************/
  /**************************************************/
  /// @brief Returns the total physical memory available
  static long long getTotalPhysicalMemory()
  {
    struct sysinfo memory_info;
    sysinfo(&memory_info);
    long long total_physical_mem = memory_info.totalram;
    total_physical_mem *= memory_info.mem_unit;
    return total_physical_mem;
  }

  /// @brief Returns the total amount of physical memory currently in use
  static long long getUsedPhysicalMemory()
  {
    struct sysinfo memory_info;
    sysinfo(&memory_info);
    long long used_physical_mem = memory_info.totalram - memory_info.freeram;
    used_physical_mem *= memory_info.mem_unit;
    return used_physical_mem;
  }

  /// @brief Returns the amount of physical memory currently in use by the calling process
  static int getProcessPhysicalMemory()
  {
    return getProcessValue("VmRSS:", 6);
  }

  /**************************************************/
  /********************** CPU ***********************/
  /**************************************************/
  /// @brief Should be called before any CPU computations
  static void initCPU()
  {
    // For total CPU usage
    FILE* file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %Ld %Ld %Ld %Ld", &last_total_user, &last_total_user_low, &last_total_sys, &last_total_idle);
    fclose(file);

    // For current process CPU usage
    struct tms time_sample;
    char line[128];

    last_cpu = times(&time_sample);
    last_sys_cpu = time_sample.tms_stime;
    last_user_cpu = time_sample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    num_processors = 0;
    while(fgets(line, 128, file) != NULL)
    {
      if(strncmp(line, "processor", 9) == 0) num_processors++;
    }
    fclose(file);
  }

  /// @brief Returns the total percentage of CPU currently in use
  static double getTotalCPU()
  {
    double percent;
    FILE *file;
    unsigned long long total_user, total_user_low, total_sys, total_idle, total;

    file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %Ld %Ld %Ld %Ld", &total_user, &total_user_low, &total_sys, &total_idle);
    fclose(file);

    if (total_user < last_total_user || total_user_low < last_total_user_low ||
        total_sys < last_total_sys || total_idle < last_total_idle)
    {
      // Overflow detection; skip this value
      percent = -1.0;
    }
    else
    {
      total = (total_user - last_total_user) + (total_user_low - last_total_user_low) + (total_sys - last_total_sys);
      percent = total;
      total += (total_idle - last_total_idle);
      percent /= total;
      percent *= 100;
    }

    last_total_user = total_user;
    last_total_user_low = total_user_low;
    last_total_sys = total_sys;
    last_total_idle = total_idle;

    return percent;
  }

  /// @brief Returns the percentage of CPU currently in use by the calling process
  static double getProcessCPU()
  {
    struct tms time_sample;
    clock_t now;
    double percent;

    now = times(&time_sample);
    if (now <= last_cpu || time_sample.tms_stime < last_sys_cpu || time_sample.tms_utime < last_user_cpu)
    {
      // Overflow detection; skip this value
      percent = -1.0;
    }
    else
    {
      percent = (time_sample.tms_stime - last_sys_cpu) + (time_sample.tms_utime - last_user_cpu);
      percent /= (now - last_cpu);
      percent /= num_processors;
      percent *= 100;
    }

    last_cpu = now;
    last_sys_cpu = time_sample.tms_stime;
    last_user_cpu = time_sample.tms_utime;

    return percent;
  }

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

  ProcessInfo()
  {
  }

  static int parseLine(char *line)
  {
    int i = strlen(line);
    while(*line < '0' || *line > '9')
      line++;
    line[i - 3] = '\0';
    i = atoi(line);
    return i;
  }

  static int getProcessValue(const char *s, int n)
  {
    FILE *file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while(fgets(line, 128, file) != NULL)
    {
      if(strncmp(line, s, n) == 0)
      {
        result = parseLine(line);
        break;
      }
    }

    fclose(file);
    return result;
  }

};

unsigned long long ProcessInfo::last_total_user = 0;
unsigned long long ProcessInfo::last_total_user_low = 0;
unsigned long long ProcessInfo::last_total_sys = 0;
unsigned long long ProcessInfo::last_total_idle = 0;

int ProcessInfo::num_processors = 0;
clock_t ProcessInfo::last_cpu;
clock_t ProcessInfo::last_sys_cpu;
clock_t ProcessInfo::last_user_cpu;

} // namespace demonstration_visualizer

#endif // PERFORMANCE_H
