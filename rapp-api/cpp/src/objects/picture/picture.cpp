#include "picture.hpp"

namespace rapp {
namespace object {

picture::picture ( const std::string filepath )
{
    bytestream_.open ( filepath, std::ios::in | std::ios::binary | std::ios::ate );
    if ( !bytestream_.is_open() )
        throw std::runtime_error ( "picture: could not open bytestream for " + filepath );
    
    picture::openCopy_ ( );
}

picture::picture ( std::ifstream & bytestream )
{
    bytestream_.copyfmt( bytestream );                                  // copy stream members
    bytestream_.clear( bytestream.rdstate() );                          // copy state of stream
    bytestream_.basic_ios<byte>::rdbuf( bytestream.rdbuf() );           // copy contents

    picture::openCopy_ ( );
}

picture::picture ( std::vector<byte> bytearray )
{
    bytearray_ = bytearray;
}

picture::~picture ( )
{
    if ( bytestream_.is_open() )
        bytestream_.close();
}

std::vector<picture::byte> picture::bytearray ( ) const
{
    return bytearray_;
}

bool picture::save ( const std::string filepath )
{
    std::ofstream os ( filepath, std::ios::out | std::ofstream::binary );
    if ( os.is_open() )
    {
        std::copy( bytearray_.begin(), bytearray_.end(), std::ostreambuf_iterator<byte>( os ) );
        os.close();
        return true;
    }
    return false;
}

bool picture::operator== ( const picture & rhs ) const
{
    return ( this->bytearray_ == rhs.bytearray_ );
}

void picture::echo ( ) const
{
    for ( const auto & byte : bytearray_ )
        std::cout << byte;
    std::cout << std::endl;
}

void picture::openCopy_ ( )
{
    bytestream_.seekg( 0, std::ios_base::end);
    std::streampos fileSize = bytestream_.tellg();
    bytearray_.resize( fileSize );
    bytestream_.seekg( 0, std::ios_base::beg );
    bytestream_.read( &bytearray_[0], fileSize );
}



}
}
