var path = require('path');
var util = require('util');
var hopService = require('hop').Service;


function WebService(svcImpl, options)
{
  this.svc_ = new hopService( svcImpl );
  this.name_ = options.name;
  this.urlName_ = options.urlName;
  this.workerName_ = options.workerName;
  this.anonymous_ = options.anonymous || false;
  this.namespace_ = options.namespace || "";

  if( ! this.anonymous_ ){
    // Set HOP Service name
    this.svc_.name = (this.namespace_) ?
      util.format("%s/%s", this.namespace_, this.urlName_) :
      this.urlName_;
  }

}


WebService.prototype.register = function()
{
  // Register service to service handler.
  var msg = {
    request: "svc_registration",
    svc_name: this.name_,
    worker_name: this.workerName_,
    svc_frame: this.svc_,
    svc_path: this.svc_.path
  };
  postMessage(msg);

};

module.exports = WebService;
