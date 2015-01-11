#ifndef RAPP_OBJECT_FACE
#define RAPP_OBJECT_FACE
#include "Includes.ihh"

namespace rapp
{
namespace object
{

/**
 * @class face
 * @brief class which should somehow encapsulate a face (as polygons or image or wtv)
 * @version 0
 * @date 3-January-2015
 * @author Alex Gkiokas <a.gkiokas@ortelio.co.uk>
 * 
 * TODO: Examine how exactly the face will be represented.
 *       I can think of a few ways: using an OpenCV matrix
 *       Using some form of polygons
 *       Using some for of a unique identifier, based on murmur hash of the polygons
 *       Using some kind of string
 *       etc...
 */

class face
{
  public:
    
    face ( ) = default;
    
};
}
}
#endif