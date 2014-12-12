/**
 * This is just a mock-up of the RAPP::API for C++
 * It serves to demonstrate the usage of calling a service on the cloud,
 * or calling a core agent method.
 * 
 * I've taken a procedural/functional approach. Ideally, it should be done in an Object-Oriented manner.
 */ 

#ifndef RAPP_HPP
#define RAPP_HPP
#include <functional>

namespace rapp
{
    // Services are to be found ONLY on the cloud
    namespace services
    {
        /**
         * Normally, when calling a service, we would assume a socket opens to the cloud.
         * This is a remote socket call, to http:://api.rapp.cloud/FooSync
         * 
         * NOTE: I won't go into the actual demonstration of how to do this, unless you want me to!
         *       My assumption is we'd use boost for C++.
         * 
         * For me, a blocking socket is of little use in robotics, so I doubt we will need this
         */
        std::string FooSync ( const std::string str )
        {
            return "bar";
        }
        
        /**
         * This time, we're calling an asynchronous service.
         * Again, this is a remote socket call, to http:://api.rapp.cloud/FooAsync
         * 
         * We pass as @param str the 1st parameter, which could be an object, a string, a stream or an image
         * We pass as @param cb the callback that will handle the completion of FooAsync
         * The callback can be a lamda, a function pointer or a function object.
         * 
         * If we take an Objet-Oriented approach, then I would use polymorphism-ala-boost to showcase
         * completion, success and error handling.
         */
        std::string FooAsync ( const std::string str, std::function<void(void)> cb )
        {
            cb();
            return "bar";
        }
        
    }
    
    // Core Methods are to be found ONLY on the Robot
    namespace core
    {
        /**
         * This is a method on the robot, part of the CORE Agent API.
         * If the user for example wants to run some standard core functionality found on the robot
         * Then he/she calls Foo directly.
         * 
         * There is no sync/async here, its up to the user how this is controlled (threaded vs non-threaded)
         * 
         * Again, in order to ensure atomicity and uniqueness, maybe this should be a singleton object,
         * rather than a method/function.
         */
        std::string Foo ( const std::string str )
        {
            return "bar";
        }
        
    }
}

#endif /* RAPP_HPP */