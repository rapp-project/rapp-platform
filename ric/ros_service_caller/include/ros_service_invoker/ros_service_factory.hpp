#include <ros_service_invoker/service_wrappers/face_detection_wrapper.hpp>

enum ServiceTypes
{
  FACE_DETECTION
};

typedef IRosServiceInvoker<int, FaceDetectionStrategies> faceDetectorCaller;

class RosInvokerFactory
{
  public:
    template <class T, class S>
    static IRosServiceInvoker<T,S>* getInvoker(ServiceTypes type)
    {
      switch(type)
      {
        case FACE_DETECTION:
          return new FaceDetectionWrapper<T,S>();
          break;
      }
    }
};
