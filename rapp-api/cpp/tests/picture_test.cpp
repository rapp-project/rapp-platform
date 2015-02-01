#include "../src/objects/picture/picture.hpp"
#include <memory>

int main ( int argc, char * argv[] )
{
    // Open picture.
    auto pic = std::make_shared<rapp::object::picture> ( "picture.jpg" );
    
    pic->save( "copy_of_picture.jpg" );
    
    return 0;
}