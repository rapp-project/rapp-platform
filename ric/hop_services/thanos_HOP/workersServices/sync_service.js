//var w = new Worker ("./slave.js");

//import service my_service (n);

service my_service (x){
    console.log(aaa);
    return (x + 1);    
}

var ls=0;
//function run () {
//    var tmp1=0;
    //var tmp2 = my_service(1).post(//function (value)
    //{
	   //// console.log ("direct use: ", value * value);
	   //// ls = value * value+1;	    return value * value;
      //}
      //,	{asynchronous:false});
  
  
    //console.log ("tmp1: ", ls);
    //console.log ("tmp2: ", tmp2);
    //ls=1;
    var start = new Date().getTime();
    var t=0;
    while(t==0)
    {
      
      var start2 = new Date().getTime();
      if((start2-start)>3000)
      {
        break;
      }
      
    }
    console.log(start2-start);
    
   //// ls=1;
  ////  return ls;
    
	//}
  
//function run () {
  
  ////return takis;
  //ls=2;
//}
//ls=run(ls);
//setTimeout (run, 10);
//timeout to let the slave worker some time to initiate
//ls=setTimeout (run, 10);
//ls=run();
//ls=run(ls);
console.log(ls);
