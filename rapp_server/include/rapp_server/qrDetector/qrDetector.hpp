#ifndef _RAPP_QR_DETECTOR_
#define _RAPP_QR_DETECTOR_

#include <rapp_server/qrDetector/Includes.hxx>

namespace rapp {
namespace cloud {

class qrDetector: public serviceHandler
{
  private:
    // Typedef of the already existent ROS service, from the
    // respective ROS node
    typedef rapp_platform_ros_communications::QrDetectionRosSrv Srv; 
    Srv srv; // The ROS service object
    std::string service_url; // The ROS service URL
    ros::NodeHandle nh_; // The ROS node handle

    /**
     * @brief The service caller function
     * @param img_url [std::string] The image for which the qr detection
     * algorithm will be deployed
     * @return std::string JSON string with the results
     */
    std::string invoke_ros_service(std::string img_url)
    {
      srv.request.imageFilename = img_url;

      std::string ret;
      // Check for the ROS service's existence
      if(!ros::service::exists(service_url, true)){
        // Throw exception
        ROS_ERROR("Service %s does not exist", service_url.c_str());
      }

      // Calls the ROS service
      if(ros::service::call(service_url, srv))
      {
        // Prepares the response
        int n = 0;
        n = srv.response.qr_centers.size();
        if(n > 0)
        {
          // Prepare the JSON response
          ret += "{\"qrs\":[";
          for(unsigned int i = 0 ; i < n ; i++)
          {
            ret += "{\"qr_center_x\":\"";
            ret += TOSTR(srv.response.qr_centers[i].point.x);
            ret += "\",\"qr_center_y\":\"";
            ret += TOSTR(srv.response.qr_centers[i].point.y);
            ret += "\",\"qr_message\":\"";
            ret += TOSTR(srv.response.qr_messages[i]);
            ret += "\"},";
          }
          ret.erase(ret.size() - 1);
          ret += "]}";
        }
        else
        {
          return "{\"qrs\":[]}";
        }
      }
      else
      {
        // Throw exception
      }
      return ret;

    }

  public:

    qrDetector(void)
    {
      if(!nh_.getParam("/qr_detection_topic", service_url))
      {
        ROS_ERROR("QR detection service parameter not found");
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
      
      std::ofstream os ( "/home/alex/copy_of_picture.png", 
        std::ios::out | std::ofstream::binary );
      
      std::copy( imagebytes.begin(), imagebytes.end(), 
        std::ostreambuf_iterator<byte>( os ) );
      
      os.close();        

      /* 
       * I am assuming this is a blocking call here ?
       * If that is the case, then the method faceDetector::process, should be 
       * in a thread?
       * 
       * Manos: yes, thats blocking. A non blocking version exists that uses
       * a callback. The thread solution can work too.
       * 
       * WARNING -
       * If this is blocking, then either the rapp_server must become threaded, or we have to use a non-blocking call here.
       * Otherwise, the server will be able to process only one request at a time!
       */
      return invoke_ros_service("/home/alex/copy_of_picture.png") ;
    } 
};

}
}
#endif
