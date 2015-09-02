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
  `owner` varchar(32) NOT NULL,
  `directory` varchar(128) NOT NULL,
  `enabled` tinyint(1) NOT NULL DEFAULT '0',
  `timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  UNIQUE KEY `id` (`id`),
  UNIQUE KEY `uid` (`rapp`,`version`,`arch`),
  KEY `fk_owner` (`owner`),
  KEY `id_2` (`id`),
  CONSTRAINT `tblRapp_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `tblUser` (`username`)
) ENGINE=InnoDB AUTO_INCREMENT=25 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Table structure for table `tblRappsModelsVersion`
--

DROP TABLE IF EXISTS `tblRappsModelsVersion`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblRappsModelsVersion` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `rapp_id` int(12) NOT NULL,
  `model_id` int(12) NOT NULL,
  `minimum_coreagent_version` decimal(18,9) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uid` (`model_id`,`rapp_id`,`minimum_coreagent_version`),
  KEY `fk_rapp_id` (`rapp_id`),
  KEY `fk_model_id` (`model_id`),
  CONSTRAINT `tblRappsModelsVersion_ibfk_1` FOREIGN KEY (`rapp_id`) REFERENCES `tblRapp` (`id`),
  CONSTRAINT `tblRappsModelsVersion_ibfk_2` FOREIGN KEY (`model_id`) REFERENCES `tblModel` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

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
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

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
  `email` varchar(254) NOT NULL,
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
) ENGINE=InnoDB AUTO_INCREMENT=25 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

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
) ENGINE=InnoDB AUTO_INCREMENT=79 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

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
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2015-09-01 13:51:42
