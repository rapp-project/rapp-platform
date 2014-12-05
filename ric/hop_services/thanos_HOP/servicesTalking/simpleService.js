var hop = require( "hop" );
//service svc1( {a: "", b: "" } ) { //{ a: 10, b: 11, c: 12 }
   
   //console.log(a);
   //console.log(b);
   //a=a+"changed";
   //b=b+"changed";
   
   //return {"a":a , "b":b}
   

  //// return {"s" : "sss"};

//}

service svc1( {path : ""}) {
  
 console.log(path);
 return hop.HTTPResponseFile( path );
 //return "aaa";
}
