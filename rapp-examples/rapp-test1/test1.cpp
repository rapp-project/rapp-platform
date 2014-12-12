/* 
 * We assume that STL is present on the robot - 
 * ANY other header/library must be imported explicitly in the RAPP Submission Service
 */
#include <iostream>

// RAPP API
#include "rapp.hpp"

// this is just a mock-up paradigm of a RAPP written in pure C++
int main ( void )
{
    std::cout << rapp::services::FooSync( "foo" ) << std::endl;
    std::cout << rapp::services::FooAsync( "foo", []( void ){ std::cout << "async cb" << std::endl;} ) << std::endl;
    std::cout << rapp::core::Foo( "foo" ) << std::endl;
}

