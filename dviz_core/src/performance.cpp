#include <dviz_core/performance.h>

namespace demonstration_visualizer
{

std::ofstream ProcessInfo::STATS_OUT_FILE;

void ProcessInfo::writeStats(const std::string &out, bool flush)
{
  if(!STATS_OUT_FILE.is_open())
    STATS_OUT_FILE.open("/home/eratner/demonstrations/stats/stats.txt", std::ofstream::out | std::ofstream::app);

  STATS_OUT_FILE << out;

  if (flush)
    STATS_OUT_FILE.flush();
}

long long ProcessInfo::getTotalVirtualMemory()
{
  struct sysinfo memory_info;
  sysinfo(&memory_info);
  long long total_virtual_mem = memory_info.totalram;
  total_virtual_mem += memory_info.totalswap;
  total_virtual_mem *= memory_info.mem_unit;
  return total_virtual_mem;
}

long long ProcessInfo::getUsedVirtualMemory()
{
  struct sysinfo memory_info;
  sysinfo(&memory_info);
  long long used_virtual_mem = memory_info.totalram - memory_info.freeram;
  used_virtual_mem += (memory_info.totalswap - memory_info.freeswap);
  used_virtual_mem *= memory_info.mem_unit;
  return used_virtual_mem;
}

int ProcessInfo::getProcessVirtualMemory()
{
  return getProcessValue("VmSize:", 7);
}

long long ProcessInfo::getTotalPhysicalMemory()
{
  struct sysinfo memory_info;
  sysinfo(&memory_info);
  long long total_physical_mem = memory_info.totalram;
  total_physical_mem *= memory_info.mem_unit;
  return total_physical_mem;
}

long long ProcessInfo::getUsedPhysicalMemory()
{
  struct sysinfo memory_info;
  sysinfo(&memory_info);
  long long used_physical_mem = memory_info.totalram - memory_info.freeram;
  used_physical_mem *= memory_info.mem_unit;
  return used_physical_mem;
}

int ProcessInfo::getProcessPhysicalMemory()
{
  return getProcessValue("VmRSS:", 6);
}

void ProcessInfo::initCPU()
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

double ProcessInfo::getTotalCPU()
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

double ProcessInfo::getProcessCPU()
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

ProcessInfo::ProcessInfo()
{
}

int ProcessInfo::parseLine(char *line)
{
  int i = strlen(line);
  while(*line < '0' || *line > '9')
    line++;
  line[i - 3] = '\0';
  i = atoi(line);
  return i;
}

int ProcessInfo::getProcessValue(const char *s, int n)
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

unsigned long long ProcessInfo::last_total_user = 0;
unsigned long long ProcessInfo::last_total_user_low = 0;
unsigned long long ProcessInfo::last_total_sys = 0;
unsigned long long ProcessInfo::last_total_idle = 0;

int ProcessInfo::num_processors = 0;
clock_t ProcessInfo::last_cpu;
clock_t ProcessInfo::last_sys_cpu;
clock_t ProcessInfo::last_user_cpu;

} // namespace demonstration_visualizer
