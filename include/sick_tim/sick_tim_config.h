
#ifndef SICK_TIM_CONFIG_
#define SICK_TIM_CONFIG_

#include <string>
namespace sick_tim
{

// Dynamic Reconfigure
struct SickTimConfig
{
  bool publish_datagram;
  double min_ang;
  double max_ang;
  bool intensity;
  int skip;
  std::string frame_id;
  double time_offset;
  bool auto_reboot;
};

} /* namespace sick_tim */
#endif /* SICK_TIM_CONFIG_ */