-- MySQL dump 10.13  Distrib 5.5.40, for debian-linux-gnu (x86_64)
--
-- Host: localhost    Database: RappStore
-- ------------------------------------------------------
-- Server version	5.5.40-0ubuntu0.14.04.1

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `tblDep`
--

DROP TABLE IF EXISTS `tblDep`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblDep` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `name` varchar(16) NOT NULL,
  `version` varchar(5) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblDep`
--

LOCK TABLES `tblDep` WRITE;
/*!40000 ALTER TABLE `tblDep` DISABLE KEYS */;
INSERT INTO `tblDep` VALUES (1,'boost','1.54'),(2,'opencv','3.0'),(3,'eigen3','3.0');
/*!40000 ALTER TABLE `tblDep` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblLibrary`
--

DROP TABLE IF EXISTS `tblLibrary`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblLibrary` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `dep` int(12) NOT NULL,
  `filename` varchar(12) NOT NULL,
  `path` varchar(128) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblLibrary`
--

LOCK TABLES `tblLibrary` WRITE;
/*!40000 ALTER TABLE `tblLibrary` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblLibrary` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblNews`
--

DROP TABLE IF EXISTS `tblNews`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblNews` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `title` varchar(128) NOT NULL,
  `text` longtext NOT NULL,
  `timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblNews`
--

LOCK TABLES `tblNews` WRITE;
/*!40000 ALTER TABLE `tblNews` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblNews` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblPackage`
--

DROP TABLE IF EXISTS `tblPackage`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblPackage` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `dep` int(12) NOT NULL,
  `name` varchar(32) NOT NULL,
  `lib_flag` varchar(64) NOT NULL,
  `inc_flag` varchar(64) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=82 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblPackage`
--

LOCK TABLES `tblPackage` WRITE;
/*!40000 ALTER TABLE `tblPackage` DISABLE KEYS */;
INSERT INTO `tblPackage` VALUES (1,2,'OpenCV','OpenCV_LIBS','OpenCV_INCLUDE_DIRS'),(2,3,'Eigen3','','EIGEN3_INCLUDE_DIR'),(3,1,'Boost','Boost_LIBRARIES','Boost_INCLUDE_DIRS'),(43,1,'mpi_python','Boost_MPI_PYTHON_LIBRARY',''),(44,1,'date_time','Boost_DATE_TIME_LIBRARY',''),(45,1,'wserialization','Boost_WSERIALIZATION_LIBRARY',''),(46,1,'python-py27','Boost_PYTHON-PY27_LIBRARY',''),(47,1,'coroutine','Boost_COROUTINE_LIBRARY',''),(48,1,'iostreams','Boost_IOSTREAMS_LIBRARY',''),(49,1,'filesystem','Boost_FILESYSTEM_LIBRARY',''),(50,1,'timer','Boost_TIMER_LIBRARY',''),(51,1,'graph','Boost_GRAPH_LIBRARY',''),(52,1,'unit_test_framework','Boost_UNIT_TEST_FRAMEWORK_LIBRARY',''),(53,1,'program_options','Boost_PROGRAM_OPTIONS_LIBRARY',''),(54,1,'prg_exec_monitor','Boost_PRG_EXEC_MONITOR_LIBRARY',''),(55,1,'wave','Boost_WAVE_LIBRARY',''),(56,1,'signals','Boost_SIGNALS_LIBRARY',''),(57,1,'chrono','Boost_CHRONO_LIBRARY',''),(58,1,'math_c99l','Boost_MATH_C99L_LIBRARY',''),(59,1,'python-py34','Boost_PYTHON-PY34_LIBRARY',''),(60,1,'serialization','Boost_SERIALIZATION_LIBRARY',''),(61,1,'mpi','Boost_MPI_LIBRARY',''),(62,1,'math_c99f','Boost_MATH_C99F_LIBRARY',''),(63,1,'thread','Boost_THREAD_LIBRARY',''),(64,1,'log_setup','Boost_LOG_SETUP_LIBRARY',''),(65,1,'random','Boost_RANDOM_LIBRARY',''),(66,1,'mpi_python-py34','Boost_MPI_PYTHON-PY34_LIBRARY',''),(67,1,'mpi_python-py27','Boost_MPI_PYTHON-PY27_LIBRARY',''),(68,1,'log','Boost_LOG_LIBRARY',''),(69,1,'regex','Boost_REGEX_LIBRARY',''),(70,1,'locale','Boost_LOCALE_LIBRARY',''),(71,1,'exception','Boost_EXCEPTION_LIBRARY',''),(72,1,'context','Boost_CONTEXT_LIBRARY',''),(73,1,'math_tr1','Boost_MATH_TR1_LIBRARY',''),(74,1,'math_c99','Boost_MATH_C99_LIBRARY',''),(75,1,'math_tr1f','Boost_MATH_TR1F_LIBRARY',''),(76,1,'system','Boost_SYSTEM_LIBRARY',''),(77,1,'graph_parallel','Boost_GRAPH_PARALLEL_LIBRARY',''),(78,1,'test_exec_monitor','Boost_TEST_EXEC_MONITOR_LIBRARY',''),(79,1,'atomic','Boost_ATOMIC_LIBRARY',''),(80,1,'math_tr1l','Boost_MATH_TR1L_LIBRARY',''),(81,1,'python','Boost_PYTHON_LIBRARY','');
/*!40000 ALTER TABLE `tblPackage` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblUserDir`
--

DROP TABLE IF EXISTS `tblUserDir`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblUserDir` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `username` varchar(32) NOT NULL,
  `lang` varchar(3) CHARACTER SET ascii NOT NULL,
  `path` varchar(64) CHARACTER SET utf8 COLLATE utf8_bin NOT NULL,
  `timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `unique_id` (`username`,`path`)
) ENGINE=InnoDB AUTO_INCREMENT=15 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblUserDir`
--

LOCK TABLES `tblUserDir` WRITE;
/*!40000 ALTER TABLE `tblUserDir` DISABLE KEYS */;
INSERT INTO `tblUserDir` VALUES (3,'alex','CPP','navigator-0.1','2015-05-12 19:39:05'),(12,'admin','ROS','rostest-0.1','2015-05-19 21:43:54'),(13,'admin','JS','hoptest-0.1','2015-05-19 21:50:41'),(14,'admin','CPP','pathfinder-0.1','2015-05-19 22:56:49');
/*!40000 ALTER TABLE `tblUserDir` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblAppsRobots`
--

DROP TABLE IF EXISTS `tblAppsRobots`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblAppsRobots` (
  `app_id` int(12) NOT NULL,
  `robot_id` int(12) NOT NULL,
  PRIMARY KEY (`app_id`,`robot_id`),
  KEY `robot_id` (`robot_id`),
  CONSTRAINT `tblAppsRobots_ibfk_1` FOREIGN KEY (`app_id`) REFERENCES `tblRapp` (`id`),
  CONSTRAINT `tblAppsRobots_ibfk_2` FOREIGN KEY (`robot_id`) REFERENCES `tblRobot` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblAppsRobots`
--

LOCK TABLES `tblAppsRobots` WRITE;
/*!40000 ALTER TABLE `tblAppsRobots` DISABLE KEYS */;
INSERT INTO `tblAppsRobots` VALUES (1,1);
/*!40000 ALTER TABLE `tblAppsRobots` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblEmail`
--

DROP TABLE IF EXISTS `tblEmail`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblEmail` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `username` varchar(32) NOT NULL,
  `password` varchar(32) NOT NULL,
  `server` varchar(128) NOT NULL,
  `email` varchar(128) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uid` (`username`,`password`,`server`,`email`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblEmail`
--

LOCK TABLES `tblEmail` WRITE;
/*!40000 ALTER TABLE `tblEmail` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblEmail` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblModel`
--

DROP TABLE IF EXISTS `tblModel`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblModel` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `model_str` varchar(128) NOT NULL,
  `manufacturer` varchar(128) NOT NULL,
  `version` decimal(18,9) NOT NULL,
  `arch` varchar(12) NOT NULL,
  `os` varchar(12) NOT NULL,
  `picture` varchar(128) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uid` (`model_str`,`manufacturer`,`version`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblModel`
--

LOCK TABLES `tblModel` WRITE;
/*!40000 ALTER TABLE `tblModel` DISABLE KEYS */;
INSERT INTO `tblModel` VALUES (1,'extreme','bioLabs',1.100000000,'1.5','ubuntu 140.0','something'),(2,'extreme2','antaIndustries',1.100000000,'1.5','ubuntu 14.04','noth1'),(3,'extreme3','antaIndustries',1.110000000,'1.6','ubuntu 14.04','roth2');
/*!40000 ALTER TABLE `tblModel` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblRapp`
--

DROP TABLE IF EXISTS `tblRapp`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblRapp` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `rapp` varchar(64) NOT NULL,
  `version` decimal(18,9) NOT NULL,
  `arch` varchar(5) NOT NULL,
  `lang` varchar(3) NOT NULL,
  `owner` int(12) NOT NULL,
  `directory` varchar(128) NOT NULL,
  `enabled` tinyint(1) NOT NULL DEFAULT '0',
  `timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  UNIQUE KEY `id` (`id`),
  UNIQUE KEY `uid` (`rapp`,`version`,`arch`),
  KEY `fk_owner` (`owner`),
  KEY `id_2` (`id`),
  CONSTRAINT `tblRapp_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `tblUser` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblRapp`
--

LOCK TABLES `tblRapp` WRITE;
/*!40000 ALTER TABLE `tblRapp` DISABLE KEYS */;
INSERT INTO `tblRapp` VALUES (1,'uberApp1',1.010000000,'15','1',0,'homethanos',0,'2014-11-23 06:04:13'),(2,'uberApp4',1.910000000,'15','1',0,'homethanos',0,'2014-11-23 07:04:13'),(3,'uberApp2',1.810000000,'16','1',0,'homethanos',0,'2014-11-23 05:04:13');
/*!40000 ALTER TABLE `tblRapp` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblRobot`
--

DROP TABLE IF EXISTS `tblRobot`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblRobot` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `macddr` bigint(8) NOT NULL,
  `model` int(12) NOT NULL,
  `owner` int(12) NOT NULL,
  `timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uid` (`macddr`,`model`),
  KEY `fk_owner` (`owner`),
  KEY `fk_model` (`model`),
  CONSTRAINT `tblRobot_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `tblUser` (`id`),
  CONSTRAINT `tblRobot_ibfk_2` FOREIGN KEY (`model`) REFERENCES `tblModel` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblRobot`
--

LOCK TABLES `tblRobot` WRITE;
/*!40000 ALTER TABLE `tblRobot` DISABLE KEYS */;
INSERT INTO `tblRobot` VALUES (1,1000000,1,1,'2014-11-23 06:04:15'),(2,9999999,1,7,'2014-11-23 06:04:15');
/*!40000 ALTER TABLE `tblRobot` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblUser`
--

DROP TABLE IF EXISTS `tblUser`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblUser` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `username` varchar(32) NOT NULL,
  `firstname` varchar(128) NOT NULL,
  `lastname` varchar(128) NOT NULL,
  `email_id` int(12) DEFAULT NULL,
  `ontology_alias` varchar(32) DEFAULT NULL,
  `pwd` char(64) CHARACTER SET ascii COLLATE ascii_bin NOT NULL,
  `usrgroup` int(1) NOT NULL DEFAULT '5',
  `created` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `accessed` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00',
  `enabled` tinyint(1) NOT NULL DEFAULT '0',
  `activation` binary(16) NOT NULL COMMENT 'uuid-v4',
  UNIQUE KEY `id` (`id`),
  UNIQUE KEY `username` (`username`),
  UNIQUE KEY `uid` (`firstname`,`lastname`,`email_id`),
  KEY `tblUser_ibfk_1` (`email_id`),
  CONSTRAINT `tblUser_ibfk_1` FOREIGN KEY (`email_id`) REFERENCES `tblEmail` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=18 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblUser`
--

LOCK TABLES `tblUser` WRITE;
/*!40000 ALTER TABLE `tblUser` DISABLE KEYS */;
INSERT INTO `tblUser` VALUES (1,'admin','Alex','Giokas',0,'NULL','486d18ed96603f0bbae2480e2c98cc80750402b28c1d4069d5df7c570ded0307',0,'2015-06-15 11:28:11','0000-00-00 00:00:00',1,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(2,'merk','Alex','Marko',0,NULL,'486d18ed96603f0bbae4567y2c98cc80750402b28c1d4069d5df7c570ded0307',0,'0000-00-00 00:00:00','0000-00-00 00:00:00',1,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(7,'bsmith','Bob','Smith',0,NULL,'7b4b075939a9c6823f85c35c19d029e2f91e8db8eabdd96a4cc0b4c8328df1c3',2,'2014-11-17 03:18:03','0000-00-00 00:00:00',0,'œK³Ý	¾@4°±'),(10,'etsardou','Manos','Tsardoulias',NULL,NULL,'',5,'2015-05-11 08:34:34','0000-00-00 00:00:00',0,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(11,'klpanagi','Konstantinos','Panagiotou',NULL,NULL,'',5,'2015-05-11 10:47:52','0000-00-00 00:00:00',0,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(12,'zacharias','Zacharias','Retsidis',NULL,NULL,'',5,'2015-05-19 06:58:06','0000-00-00 00:00:00',0,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(13,'gonipanela','Goni','Panela',NULL,NULL,'',5,'2015-05-19 06:58:50','0000-00-00 00:00:00',0,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(14,'dimkalama','Dimitris','Kalamaras',NULL,NULL,'',5,'2015-05-19 06:59:15','0000-00-00 00:00:00',0,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(15,'betatob','Beta','Tompoulidou',NULL,NULL,'',5,'2015-05-19 07:11:59','0000-00-00 00:00:00',0,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(16,'litsakoud','Litsa','Koudroglou',NULL,NULL,'',5,'2015-05-19 07:12:27','0000-00-00 00:00:00',0,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(17,'staumoum','Stauroula','Moumka',NULL,NULL,'',5,'2015-05-19 07:13:00','0000-00-00 00:00:00',0,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0');
/*!40000 ALTER TABLE `tblUser` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblUserRelations`
--

DROP TABLE IF EXISTS `tblUserRelations`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblUserRelations` (
  `user_id1` int(12) NOT NULL,
  `user_id2` int(12) NOT NULL,
  `connection` enum('doctor','caregiver','child of') NOT NULL,
  `description` varchar(128) NOT NULL,
  PRIMARY KEY (`user_id1`,`user_id2`,`connection`),
  KEY `tblUserRelations_ibfk_2` (`user_id2`),
  CONSTRAINT `tblUserRelations_ibfk_1` FOREIGN KEY (`user_id1`) REFERENCES `tblUser` (`id`),
  CONSTRAINT `tblUserRelations_ibfk_2` FOREIGN KEY (`user_id2`) REFERENCES `tblUser` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblUserRelations`
--

LOCK TABLES `tblUserRelations` WRITE;
/*!40000 ALTER TABLE `tblUserRelations` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblUserRelations` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblUsersOntologyInstances`
--

DROP TABLE IF EXISTS `tblUsersOntologyInstances`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblUsersOntologyInstances` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `user_id` int(12) NOT NULL,
  `ontology_class` varchar(50) NOT NULL,
  `ontology_instance` varchar(50) NOT NULL,
  `file_url` varchar(50) NOT NULL,
  `comments` varchar(50) DEFAULT NULL,
  `created_timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `updated_timestamp` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00',
  PRIMARY KEY (`id`),
  KEY `user_id` (`user_id`),
  CONSTRAINT `tblUsersOntologyInstances_ibfk_1` FOREIGN KEY (`user_id`) REFERENCES `tblUser` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=13 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblUsersOntologyInstances`
--

LOCK TABLES `tblUsersOntologyInstances` WRITE;
/*!40000 ALTER TABLE `tblUsersOntologyInstances` DISABLE KEYS */;
INSERT INTO `tblUsersOntologyInstances` VALUES (1,1,'ontology_something','instance_something','/home/smthing','akraio','2014-11-15 16:01:34','2014-11-15 17:01:34'),(3,1,'FoodOrDrink','FoodOrDrink_qdaDeDZn','url_something','comments_something','2015-02-17 22:00:00','2015-02-17 22:00:00');
/*!40000 ALTER TABLE `tblUsersOntologyInstances` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2015-06-30 13:54:38
