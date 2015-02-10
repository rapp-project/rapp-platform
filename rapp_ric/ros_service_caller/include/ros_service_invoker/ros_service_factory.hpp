#ifndef ROS_SERVICE_FACTORY_HPP
#define ROS_SERVICE_FACTORY_HPP

#include <ros_service_invoker/service_wrappers/face_detection_wrapper.hpp>
#include <ros_service_invoker/service_wrappers/qr_detection_wrapper.hpp>
#include <ros_service_invoker/service_wrappers/speech_detection_google_wrapper.hpp>

/**
 * @enum ServiceTypes
 * @brief The enumeration for the different RPSs (RAPP Platform Services) Each 
 * new service must be declared here as a different enumeration element
 */
enum ServiceTypes
{
  FACE_DETECTION,
  QR_DETECTION,
  SPEECH_DETECTION_GOOGLE
};

/**
 * @class RosInvokerFactory
 * @brief A typical factory software pattern that returns a ros service invoker.
 */
class RosInvokerFactory
{
  public:
    /**
     * @brief The getInvoker is a static function returning an invoker
     * @details This function is templated by two arguments. The first, <class T>
     * specifies the type of the input variable of the invoker's setup function.
     * The second, <class S> determines the strategy (if one exists) in the case
     * that invoker sets up the service message with the same input argument 
     * type, but in a different way.
     * @param type [ServiceTypes] The invoker type. 
     */
    template <class T, class S>
    static IRosServiceInvoker<T,S>* getInvoker(ServiceTypes type)
    {
      /* Switch on the types of invokers. For a new invoker, this switch must 
       * be expanded */
      switch(type)      
      {
        case FACE_DETECTION: // Face Detection case
          return new FaceDetectionWrapper<T,S>();
          break;
        case QR_DETECTION:
          return new QrDetectionWrapper<T,S>();
          break;
        case SPEECH_DETECTION_GOOGLE:
          return new SpeechDetectionGoogleWrapper<T,S>();
          break;
        default: ;
          // Throw exception
      }
    }
};

#endif // ROS_SERVICE_FACTORY_HPP
