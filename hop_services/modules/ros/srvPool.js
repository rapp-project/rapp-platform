/*!
 * @file srvChooser
 * @brief Choose from availabe ros service list.
 */

/**
 *  MIT License (MIT)
 *
 *  Copyright (c) <2014> <Rapp Project EU>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  Authors: Konstantinos Panayiotou
 *  Contact: klpanagi@gmail.com
 *
 */

function SrvPool ( rosSrv, poolSize ){
  this.stackSize = poolSize || 10;
  this.base = rosSrv;
  this.cache = [];
  this.clients = [];
  // Initialize all as available (0)
  for(var ii = 0; ii < this.stackSize; ii++){this.cache[ii] = 0};


  /*!
   * @brief Allocate for service with this index.
   */
  this.allocate = function( index )
  {
    this.cache[index] = 1; // Allocate for this index
  };


  /*!
   * @brief DeAllocate for service with this index.
   */
  this.deallocate = function( index )
  {
    this.cache[index] = 0; // Allocate for this index
  };


  /*!
   * @brief Checks if is currently allocated or released.
   * @param index Index number from 0 to stackSize.
   */
  this.isAllocated = function( index )
  {
    if( this.cache[index] ){return true;}
    return false;
  };
};


SrvPool.prototype.getAvailable = function()
{
  /**
   * @brief Simple implementation. Iterate through cache[] and return
   * the first available (not allocated).
   */
  var srvName = this.base;
  var clientAssigned = false
  for( var ii = 0; ii < this.stackSize; ii++ )
  {
    if(!this.isAllocated(ii))
    {
      srvName += '_' + ii;
      //Allocate
      clientAssigned = true;
      this.allocate(ii);
      break;
    }
  }
  if(!clientAssigned)
  {
    var randomPickup = Math.floor(Math.random() * (this.stackSize -1));
    this.allocate(randomPickup);
    srvName += '_' + randomPickup;
  }

  // If all are avilable then return to base srv. this.base
  return srvName;
};


SrvPool.prototype.release = function(srvName)
{
  var idx = srvName[srvName.length - 1];
  this.deallocate(idx);
};

/**
 * Module exports
 */
module.exports = SrvPool;
