import service my_service (n);

function run () {
  var tmp1;
  var tt = my_service(1).post(
    function (value)
    {
      console.log ("direct use: ", value * value);
      tmp1 = value * value;
      return value * value;
    },
    {
       host: "localhost",asynchronous: false, port: "9001" 
    }
  );
  console.log ("tmp1: ", tmp1);
  return tt;
}

var t = run()
console.log ("t: ", t);

