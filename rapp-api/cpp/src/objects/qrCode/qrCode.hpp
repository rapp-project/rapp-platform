#ifndef RAPP_OBJECT_QRCODE
#define RAPP_OBJECT_QRCODE
#include "Includes.ihh"

namespace rapp
{
namespace objects
{

/**
 * @class qrCode
 * @brief class which should encapsulate a QR code
 * @version 0
 * @date 3-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * TODO: Examine how exactly the qr Code will be represented.
 *       I can think of a few ways: 
 *       using an OpenCV matrix
 *       Using a square coordinates (x1,y1, to x2, y2)
 *       Using some for of a unique identifier
 *       Using some kind of string/url/uri
 *       etc...
 */

class qrCode
{
  public:
    
    qrCode ( ) = default;
    
};
}
}
#endif