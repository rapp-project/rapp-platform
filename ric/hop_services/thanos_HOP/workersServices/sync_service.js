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
      { host: "localhost",asynchronous: false, port: "9000" }
    );
    console.log ("tmp1: ", tmp1);
    return tt;
	}

var t = run()
console.log ("t: ", t);
//timeout to let the slave worker some time to initiate

//~ rapp +d0d8b7c7d5c993c02f6b87972f690c28

