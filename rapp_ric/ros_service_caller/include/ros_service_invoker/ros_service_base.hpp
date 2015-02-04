#ifndef ROS_SERVICE_BASE_HPP
#define ROS_SERVICE_BASE_HPP

#include <ros_service_invoker/ros_service_strategy_base.hpp>

/**
 * @class IRosServiceInvoker
 * @details Templated abstract class to be inherited by the corresponding 
 * invokers. The template arguments are 2:
 * <class T> which determines the setup input type
 * <class S> which determines the setup strategy for a specific input type
 * Each invoker must inherit this class and implement the setup and
 * call_service pure virtual functions
 */
template <class T, class S>
class IRosServiceInvoker
{
  private:
    // The corresponding invoker's strategy, specialized by the invokers
    // strategies enum 
    BaseStrategy<S> strategy;

  public:

    /**
     * @brief Setter for the BaseStrategy's change_strategy function
     * @param strategy [S] The corresponding invoker's strategy enum element
     */
    void set_strategy(S strategy){ 
      this->strategy.change_strategy(strategy); 
    }

    /**
     * @brief Getter for the BaseStrategy's get_strategy function
     * @param strategy [S] The corresponding invoker's strategy enum element
     */
    S get_strategy(void) { 
      return strategy.get_strategy(); 
    }

    /**
     * @brief Pure virtual function, uptaking the task of setting up the
     * service request.
     * @details This function must be implemented for each type in each
     * invoker.
     */
    virtual void setup(T s) = 0;

    /**
     * @brief Pure virtual function that calls the service the invoker was
     * created for.
     */
    virtual std::string call_service() = 0;

    /**
     * Virtual destructor
     */
    virtual ~IRosServiceInvoker(){}
};

#endif // ROS_SERVICE_BASE_HPP
