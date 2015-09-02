#!/bin/bash

url=$HOME/Development/rapp_ws/src/rapp-platform/scripts/testing/testing_output
url_err=$HOME/Development/rapp_ws/src/rapp-platform/scripts/testing/report

source ~/.bashrc
cd $HOME/Development/rapp_ws && catkin_make run_tests -j1 2>&1 | tee $url #Now we have the output

error_found=false
error_lines=0

rm $url_err ; touch $url_err
cat $url | while read line; do
    # Find sum of tests here..

    # Get the lines from the prev error (if exists)
    if [ $error_found == true ] && [ $error_lines -le "5" ]; then
      let "error_lines=error_lines+1"
      [ -z "$line" ] && continue # skip empty lines
      echo $line >> $url_err
      continue
    else
      error_found=false
      error_lines=0
    fi
    # Find the actual test error in line
    if echo $line | grep -q "FAIL:" ; then
      echo $line >> $url_err
      error_found=true
    fi
done

rm $url
 
