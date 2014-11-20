if (CATKIN_ENABLE_TESTING)
  #catkin_add_nosetests(test/mysql_wrapper/t1.py)
  add_rostest(../../test/mysql_wrapper/db_test.launch)
endif()
