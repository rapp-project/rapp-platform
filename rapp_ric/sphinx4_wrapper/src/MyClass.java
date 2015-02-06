/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
//package myclass;

import java.io.*;
import java.net.URL;
import java.net.URLClassLoader;
 
public class MyClass {
    public static void main(String[] args) {
        

ClassLoader cl = ClassLoader.getSystemClassLoader();
 
        URL[] urls = ((URLClassLoader)cl).getURLs();
 
        for(URL url: urls){
        	//System.out.println(url.getFile());
        }


        try {
            BufferedReader bufferRead = new BufferedReader(new InputStreamReader(System.in));
            //PrintWriter writer = new PrintWriter("result.txt", "UTF-8");
            String s = bufferRead.readLine();
            while(s.equals("x")==false) {
                //writer.println(s);
		//System.out.println(s);
		//System.out.println("x");
                //s = bufferRead.readLine();
            }
            

        System.out.println("TAKE IT");
        System.out.println("x");
		System.out.println("Terminate transmit");
            //writer.close();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }
}

