
function Router(callback, onerror) {
  this.clb = callback;
  this.onerror = onerror;
}

Router.prototype.run = function(req, resp, ros) {
  var that = this;

  this.clb(req, resp, ros)
}
