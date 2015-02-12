#ifndef _RAPP_FACE_DETECTOR_
#define _RAPP_FACE_DETECTOR_

#include <rapp_server/faceDetector/Includes.hxx>

namespace rapp {
namespace cloud {

class faceDetector: public serviceHandler
{
  private:
    // Typedef of the already existent Face Detection ROS service, from the
    // ros_nodes/face_detection ROS node
    typedef rapp_platform_ros_communications::FaceDetectionRosSrv Srv; 
    Srv srv; // The ROS service object
    std::string service_url; // The ROS service URL
    ros::NodeHandle nh_; // The ROS node handle

    /**
     * @brief The service caller function
     * @param img_url [std::string] The image for which the face detection
     * algorithm will be deployed
     * @return std::string JSON string with the results
     */
    std::string invoke_ros_service(std::string img_url)
    {
      srv.request.imageFilename = img_url;

      std::string ret;
      // Check for the face detection service's existence
      if(!ros::service::exists(service_url, true)){
        // Throw exception
        ROS_ERROR("Service %s does not exist", service_url.c_str());
      }

      // Calls the ROS face detection service
      if(ros::service::call(service_url, srv))
      {
        // Prepares the response
        int n = 0;
        n = srv.response.faces_up_left.size();
        if(n > 0)
        {
          // Prepare the JSON response
          ret += "{\"faces\":[";
          for(unsigned int i = 0 ; i < n ; i++)
          {
            ret += "{\"top_left_x\":\"";
            ret += TOSTR(srv.response.faces_up_left[i].point.x);
            ret += "\",\"top_left_y\":\"";
            ret += TOSTR(srv.response.faces_up_left[i].point.y);
            ret += "\",\"bottom_right_x\":\"";
            ret += TOSTR(srv.response.faces_down_right[i].point.x);
            ret += "\",\"bottom_right_y\":\"";
            ret += TOSTR(srv.response.faces_down_right[i].point.y);
            ret += "\"},";
          }
          ret.erase(ret.size() - 1);
          ret += "]}";
        }
        else
        {
          return "{\"faces\":[]}";
        }
      }
      else
      {
        // Throw exception
      }
      return ret;

    }

  public:

    faceDetector(void)
    {
      if(!nh_.getParam("/face_detection_topic", service_url))
      {
        ROS_ERROR("Face detection service parameter not found");
      }
    }
    
    typedef char byte;
      
    std::string process ( const std::vector<byte> & bytearray )
    {
      std::vector<byte> imagebytes;

      // Search for the `<IMG!>` delimiter - then copy from that position, 
      // and up to the position of <EOF!>
      for ( unsigned int i = 0; i < bytearray.size(); i++ )
      {
        if ( (i + 5) < ( bytearray.size()-7 ) )
        {
          // NOTE: In order to avoid copying 5 bytes into string, maybe test  
          // first char is `<`?
          if ( std::string( &bytearray[i], 5 ) == "<IMG>" )    // Find <IMG>
          {
            std::copy ( bytearray.begin() + i + 5,          // length of <IMG>
                        bytearray.end() - 7,                // length of </EOF!>
                        std::back_inserter( imagebytes ) );
            break;
          }
        }   
      }

      // Copy Image Bytes to a file on Disk
      std::cout << "Image bytes: " << imagebytes.size() << std::endl;
      
      std::ofstream os ( "/home/etsardou/copy_of_picture.png", 
        std::ios::out | std::ofstream::binary );
      
      std::copy( imagebytes.begin(), imagebytes.end(), 
        std::ostreambuf_iterator<byte>( os ) );
      
      os.close();        

      /* NOTE: 
       * I am assuming this is a blocking call here ?
       * If that is the case, then the method faceDetector::process, should be 
       * in a thread?
       * Manos: yes, thats blocking. A non blocking version exists that uses
       * a callback. The thread solution can work too.
       */
      return invoke_ros_service("/home/etsardou/copy_of_picture.png") ;
    } 
};

}
}
#endif
