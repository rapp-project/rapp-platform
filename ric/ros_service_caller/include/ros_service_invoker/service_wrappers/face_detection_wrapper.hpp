#include <ros_service_invoker/ros_service_base.hpp>


enum FaceDetectionStrategies
{
  INT_1,
  INT_2
};


template <class T, class S>
class FaceDetectionWrapper : public IRosServiceInvoker<T, S>
{
  public:
    FaceDetectionWrapper(){}
    void setup(T s){}
    void setup_strategy_int_1(int s);
    std::string call_service(){return std::string();}
};

template<>
void FaceDetectionWrapper<int, FaceDetectionStrategies>::setup_strategy_int_1(int s){
  std::cout << "Called int 1\n";
}

template <>
void FaceDetectionWrapper<int, FaceDetectionStrategies>::setup(int s){
  if(get_strategy() == INT_1){
    setup_strategy_int_1(s);
  }
  else
  {
    std::cout << "Called int_2\n";
  }
}


template <>
void FaceDetectionWrapper<float, FaceDetectionStrategies>::setup(float s){
  std::cout << "Called float\n";
}



