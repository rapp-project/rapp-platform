#ifndef RAPP_SERVICE_CLOUD_GLOBALS
#define RAPP_SERVICE_CLOUD_GLOBALS

namespace rapp
{
namespace services
{
namespace cloud
{

/**
 * A few global strings needed internally
 * @version 1
 * @date 10-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 */

/// api.rapp.cloud
constexpr char server_address[] = "127.0.0.1";

/// This assumes PHP - Not HOP ! @NOTE: this is used only for testing, FIX
constexpr char face_detect_uri[] = "/hop/faceDetection";

}
}
}
#endif