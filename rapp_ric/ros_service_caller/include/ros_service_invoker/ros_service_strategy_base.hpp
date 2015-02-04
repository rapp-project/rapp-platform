#ifndef ROS_SERVICE_STRATEGY_BASE_HPP
#define ROS_SERVICE_STRATEGY_BASE_HPP

#include <ros_service_invoker/ros_service_includes.hpp>

/** 
 * @class BaseStrategy
 * @brief This class is the base Strategy class, providing set and get strategy
 * functions. Every invoker must implement its own strategy class, inheriting
 * this one. The <class T> template parameter concerns the specific invoker's
 * strategies, included in its own enum. This class is not directly used from
 * the invokers, but via the IRosServiceInvoker.
 */
template <class S> 
class BaseStrategy
{
  private:
    S strategy; // The current strategy
  
  public:
    // Returns the current strategy
    S get_strategy(){ 
      return strategy; 
    }
    // Changes the current strategy
    void change_strategy(S strategy){ 
      this->strategy = strategy; 
    }
};

#endif // ROS_SERVICE_STRATEGY_BASE_HPP
