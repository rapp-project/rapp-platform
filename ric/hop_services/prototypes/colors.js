


function ColorsMap() {
  this.map = new Object(); // or var map = {};
  this.map["red"] = "\033[01;31m";
  this.map["green"] = "\033[01;32m";
  this.map["yellow"] = "\033[01;33m";
  this.map["blue"] = "\033[01;34m";
  this.map["magenta"] = "\033[01;35m";
  this.map["cyan"] = "\033[01;36m";
};

function getColor(color) {
  var _colors = new ColorsMap();
  return _colors.map[color];
}
  

exports.getColor = getColor; 
