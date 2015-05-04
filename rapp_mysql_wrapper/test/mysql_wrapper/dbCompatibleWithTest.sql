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
-- Table structure for table `tblappsrobots`
--

DROP TABLE IF EXISTS `tblappsrobots`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblappsrobots` (
  `app_id` int(12) NOT NULL,
  `robot_id` int(12) NOT NULL,
  PRIMARY KEY (`app_id`,`robot_id`),
  KEY `robot_id` (`robot_id`),
  CONSTRAINT `tblappsrobots_ibfk_1` FOREIGN KEY (`app_id`) REFERENCES `tblrapp` (`id`),
  CONSTRAINT `tblappsrobots_ibfk_2` FOREIGN KEY (`robot_id`) REFERENCES `tblrobot` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblappsrobots`
--

LOCK TABLES `tblappsrobots` WRITE;
/*!40000 ALTER TABLE `tblappsrobots` DISABLE KEYS */;
INSERT INTO `tblappsrobots` VALUES (1,1);
/*!40000 ALTER TABLE `tblappsrobots` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblemail`
--

DROP TABLE IF EXISTS `tblemail`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblemail` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `username` varchar(32) NOT NULL,
  `password` varchar(32) NOT NULL,
  `server` varchar(128) NOT NULL,
  `email` varchar(128) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uid` (`username`,`password`,`server`,`email`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblemail`
--

LOCK TABLES `tblemail` WRITE;
/*!40000 ALTER TABLE `tblemail` DISABLE KEYS */;
INSERT INTO `tblemail` VALUES (1,'some','some','some','ttt');
/*!40000 ALTER TABLE `tblemail` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblmodel`
--

DROP TABLE IF EXISTS `tblmodel`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblmodel` (
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
-- Dumping data for table `tblmodel`
--

LOCK TABLES `tblmodel` WRITE;
/*!40000 ALTER TABLE `tblmodel` DISABLE KEYS */;
INSERT INTO `tblmodel` VALUES (1,'extreme','bioLabs',1.100000000,'1.5','ubuntu 140.0','something');
/*!40000 ALTER TABLE `tblmodel` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblrapp`
--

DROP TABLE IF EXISTS `tblrapp`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblrapp` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `rapp` varchar(64) NOT NULL,
  `version` decimal(18,9) NOT NULL,
  `arch` varchar(5) NOT NULL,
  `owner` int(12) NOT NULL,
  `directory` varchar(128) NOT NULL,
  `enabled` tinyint(1) NOT NULL DEFAULT '0',
  `timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  UNIQUE KEY `id` (`id`),
  UNIQUE KEY `uid` (`rapp`,`version`,`arch`),
  KEY `fk_owner` (`owner`),
  KEY `id_2` (`id`),
  CONSTRAINT `tblrapp_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `tbluser` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblrapp`
--

LOCK TABLES `tblrapp` WRITE;
/*!40000 ALTER TABLE `tblrapp` DISABLE KEYS */;
INSERT INTO `tblrapp` VALUES (1,'uberApp1',1.010000000,'15',1,'homethanos',0,'2014-11-23 06:04:13');
/*!40000 ALTER TABLE `tblrapp` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblrobot`
--

DROP TABLE IF EXISTS `tblrobot`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblrobot` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `macddr` bigint(8) NOT NULL,
  `model` int(12) NOT NULL,
  `owner` int(12) NOT NULL,
  `timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uid` (`macddr`,`model`),
  KEY `fk_owner` (`owner`),
  KEY `fk_model` (`model`),
  CONSTRAINT `tblrobot_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `tbluser` (`id`),
  CONSTRAINT `tblrobot_ibfk_2` FOREIGN KEY (`model`) REFERENCES `tblmodel` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblrobot`
--

LOCK TABLES `tblrobot` WRITE;
/*!40000 ALTER TABLE `tblrobot` DISABLE KEYS */;
INSERT INTO `tblrobot` VALUES (1,1000000,1,1,'2014-11-23 06:04:15'),(2,9999999,1,7,'2014-11-23 06:04:15');
/*!40000 ALTER TABLE `tblrobot` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tbluser`
--

DROP TABLE IF EXISTS `tbluser`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tbluser` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `username` varchar(32) NOT NULL,
  `firstname` varchar(128) NOT NULL,
  `lastname` varchar(128) NOT NULL,
  `email_id` int(12) NOT NULL,
  `pwd` char(64) CHARACTER SET ascii COLLATE ascii_bin NOT NULL,
  `usrgroup` int(1) NOT NULL DEFAULT '5',
  `created` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `accessed` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00',
  `enabled` tinyint(1) NOT NULL DEFAULT '0',
  `activation` binary(16) NOT NULL COMMENT 'uuid-v4',
  UNIQUE KEY `id` (`id`),
  UNIQUE KEY `uid` (`firstname`,`lastname`,`email_id`),
  UNIQUE KEY `username` (`username`),
  KEY `tbluser_ibfk_1` (`email_id`),
  CONSTRAINT `tbluser_ibfk_1` FOREIGN KEY (`email_id`) REFERENCES `tblemail` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=13 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tbluser`
--

LOCK TABLES `tbluser` WRITE;
/*!40000 ALTER TABLE `tbluser` DISABLE KEYS */;
INSERT INTO `tbluser` VALUES (1,'admin','Alex','Giokas',0,'486d18ed96603f0bbae2480e2c98cc80750402b28c1d4069d5df7c570ded0307',0,'2014-11-15 18:01:34','0000-00-00 00:00:00',1,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(2,'merk','Alex','Marko',0,'486d18ed96603f0bbae4567y2c98cc80750402b28c1d4069d5df7c570ded0307',0,'0000-00-00 00:00:00','0000-00-00 00:00:00',1,'\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),(7,'bsmith','Bob','Smith',0,'7b4b075939a9c6823f85c35c19d029e2f91e8db8eabdd96a4cc0b4c8328df1c3',2,'2014-11-17 03:18:03','0000-00-00 00:00:00',0,'œK³Ý	¾@4°±');
/*!40000 ALTER TABLE `tbluser` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tbluserrelations`
--

DROP TABLE IF EXISTS `tbluserrelations`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tbluserrelations` (
  `user_id1` int(12) NOT NULL,
  `user_id2` int(12) NOT NULL,
  `connection` enum('doctor','caregiver','child of') NOT NULL,
  `description` varchar(128) NOT NULL,
  PRIMARY KEY (`user_id1`,`user_id2`,`connection`),
  KEY `tbluserrelations_ibfk_2` (`user_id2`),
  CONSTRAINT `tbluserrelations_ibfk_1` FOREIGN KEY (`user_id1`) REFERENCES `tbluser` (`id`),
  CONSTRAINT `tbluserrelations_ibfk_2` FOREIGN KEY (`user_id2`) REFERENCES `tbluser` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tbluserrelations`
--

LOCK TABLES `tbluserrelations` WRITE;
/*!40000 ALTER TABLE `tbluserrelations` DISABLE KEYS */;
/*!40000 ALTER TABLE `tbluserrelations` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblusersontologyinstances`
--

DROP TABLE IF EXISTS `tblusersontologyinstances`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblusersontologyinstances` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `user_id` varchar(32) NOT NULL,
  `ontology_class` varchar(50) NOT NULL,
  `ontology_instance` varchar(50) NOT NULL,
  `file_url` varchar(50) NOT NULL,
  `comments` varchar(50) DEFAULT NULL,
  `created_timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `updated_timestamp` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00',
  PRIMARY KEY (`id`),
  KEY `user_id` (`user_id`),
  CONSTRAINT `tblusersontologyinstances_ibfk_1` FOREIGN KEY (`user_id`) REFERENCES `tbluser` (`username`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblusersontologyinstances`
--

LOCK TABLES `tblusersontologyinstances` WRITE;
/*!40000 ALTER TABLE `tblusersontologyinstances` DISABLE KEYS */;
INSERT INTO `tblusersontologyinstances` VALUES (1,'admin','ontology_something','instance_something','/home/smthing','akraio','2014-11-15 16:01:34','2014-11-15 17:01:34');
/*!40000 ALTER TABLE `tblusersontologyinstances` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Temporary table structure for view `usersrobotsapps`
--

DROP TABLE IF EXISTS `usersrobotsapps`;
/*!50001 DROP VIEW IF EXISTS `usersrobotsapps`*/;
SET @saved_cs_client     = @@character_set_client;
SET character_set_client = utf8;
/*!50001 CREATE TABLE `usersrobotsapps` (
  `owner` tinyint NOT NULL,
  `id` tinyint NOT NULL,
  `app_id` tinyint NOT NULL
) ENGINE=MyISAM */;
SET character_set_client = @saved_cs_client;

--
-- Final view structure for view `usersrobotsapps`
--

/*!50001 DROP TABLE IF EXISTS `usersrobotsapps`*/;
/*!50001 DROP VIEW IF EXISTS `usersrobotsapps`*/;
/*!50001 SET @saved_cs_client          = @@character_set_client */;
/*!50001 SET @saved_cs_results         = @@character_set_results */;
/*!50001 SET @saved_col_connection     = @@collation_connection */;
/*!50001 SET character_set_client      = utf8 */;
/*!50001 SET character_set_results     = utf8 */;
/*!50001 SET collation_connection      = utf8_general_ci */;
/*!50001 CREATE ALGORITHM=UNDEFINED */
/*!50013 DEFINER=`root`@`localhost` SQL SECURITY DEFINER */
/*!50001 VIEW `usersrobotsapps` AS select `a`.`owner` AS `owner`,`a`.`id` AS `id`,`b`.`app_id` AS `app_id` from (`tblrobot` `a` join `tblappsrobots` `b`) where (`a`.`id` = `b`.`robot_id`) */;
/*!50001 SET character_set_client      = @saved_cs_client */;
/*!50001 SET character_set_results     = @saved_cs_results */;
/*!50001 SET collation_connection      = @saved_col_connection */;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2015-02-08 16:53:16
