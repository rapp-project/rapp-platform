-- MySQL dump 10.13  Distrib 5.5.47, for debian-linux-gnu (x86_64)
--
-- Host: localhost    Database: rapp_platform
-- ------------------------------------------------------
-- Server version	5.5.47-0ubuntu0.14.04.1

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
-- Table structure for table `application_token`
--

DROP TABLE IF EXISTS `application_token`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `application_token` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `token` varchar(256) NOT NULL,
  `platform_user_id` int(10) unsigned NOT NULL,
  `device_token` varchar(256) NOT NULL,
  `status` tinyint(3) unsigned DEFAULT '1',
  `creation_time` bigint(20) unsigned NOT NULL,
  `expiration_time` bigint(20) unsigned NOT NULL,
  `last_update_time` bigint(20) unsigned NOT NULL,
  PRIMARY KEY (`id`),
  KEY `platform_user_id` (`platform_user_id`),
  CONSTRAINT `application_token_ibfk_1` FOREIGN KEY (`platform_user_id`) REFERENCES `platform_user` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `application_token`
--

LOCK TABLES `application_token` WRITE;
/*!40000 ALTER TABLE `application_token` DISABLE KEYS */;
INSERT INTO `application_token` VALUES (1,'rapp_token',1,'rapp_device_token',1,1459412069,1000,1000);
/*!40000 ALTER TABLE `application_token` ENABLE KEYS */;
UNLOCK TABLES;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8 */ ;
/*!50003 SET character_set_results = utf8 */ ;
/*!50003 SET collation_connection  = utf8_general_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = '' */ ;
DELIMITER ;;
/*!50003 CREATE*/ /*!50017 DEFINER=`root`@`localhost`*/ /*!50003 trigger `token_created_BEFORE_INSERT`
  before insert on `application_token`
  for each row
    set NEW.creation_time = UNIX_TIMESTAMP(UTC_TIMESTAMP()) */;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8 */ ;
/*!50003 SET character_set_results = utf8 */ ;
/*!50003 SET collation_connection  = utf8_general_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = '' */ ;
DELIMITER ;;
/*!50003 CREATE*/ /*!50017 DEFINER=`root`@`localhost`*/ /*!50003 trigger `token_updated_BEFORE_UPDATE`
  before update on `application_token`
  for each row
     set NEW.`last_update_time` = UNIX_TIMESTAMP(UTC_TIMESTAMP()) */;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;

--
-- Table structure for table `cloud_agent`
--

DROP TABLE IF EXISTS `cloud_agent`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `cloud_agent` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `platform_user_id` int(10) unsigned NOT NULL,
  `tarball_path` varchar(256) NOT NULL,
  `container_identifier` varchar(256) NOT NULL,
  `image_identifier` varchar(256) NOT NULL,
  `container_type` varchar(256) NOT NULL,
  PRIMARY KEY (`id`),
  KEY `platform_user_id` (`platform_user_id`),
  CONSTRAINT `cloud_agent_ibfk_1` FOREIGN KEY (`platform_user_id`) REFERENCES `platform_user` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cloud_agent`
--

LOCK TABLES `cloud_agent` WRITE;
/*!40000 ALTER TABLE `cloud_agent` DISABLE KEYS */;
/*!40000 ALTER TABLE `cloud_agent` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `cloud_agent_service_arguments`
--

DROP TABLE IF EXISTS `cloud_agent_service_arguments`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `cloud_agent_service_arguments` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `cloud_agent_service_id` int(10) unsigned NOT NULL,
  `argument_name` varchar(64) NOT NULL,
  PRIMARY KEY (`id`),
  KEY `cloud_agent_service_id` (`cloud_agent_service_id`),
  CONSTRAINT `cloud_agent_service_arguments_ibfk_1` FOREIGN KEY (`cloud_agent_service_id`) REFERENCES `cloud_agent_services` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cloud_agent_service_arguments`
--

LOCK TABLES `cloud_agent_service_arguments` WRITE;
/*!40000 ALTER TABLE `cloud_agent_service_arguments` DISABLE KEYS */;
/*!40000 ALTER TABLE `cloud_agent_service_arguments` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `cloud_agent_services`
--

DROP TABLE IF EXISTS `cloud_agent_services`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `cloud_agent_services` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `cloud_agent_id` int(10) unsigned NOT NULL,
  `service_name` varchar(256) NOT NULL,
  `service_type` varchar(32) NOT NULL,
  `container_port` smallint(5) unsigned NOT NULL,
  `host_port` smallint(5) unsigned NOT NULL,
  PRIMARY KEY (`id`),
  KEY `cloud_agent_id` (`cloud_agent_id`),
  CONSTRAINT `cloud_agent_services_ibfk_1` FOREIGN KEY (`cloud_agent_id`) REFERENCES `cloud_agent` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cloud_agent_services`
--

LOCK TABLES `cloud_agent_services` WRITE;
/*!40000 ALTER TABLE `cloud_agent_services` DISABLE KEYS */;
/*!40000 ALTER TABLE `cloud_agent_services` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `platform_user`
--

DROP TABLE IF EXISTS `platform_user`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `platform_user` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `username` varchar(32) NOT NULL,
  `password` varchar(64) NOT NULL,
  `ontology_alias` varchar(64) DEFAULT NULL,
  `language` varchar(64) NOT NULL,
  `device_token` varchar(128) NOT NULL,
  `creation_time` bigint(20) unsigned NOT NULL,
  `status` tinyint(3) unsigned DEFAULT '1',
  PRIMARY KEY (`id`),
  UNIQUE KEY `username` (`username`)
) ENGINE=InnoDB AUTO_INCREMENT=10 DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `platform_user`
--

LOCK TABLES `platform_user` WRITE;
/*!40000 ALTER TABLE `platform_user` DISABLE KEYS */;
INSERT INTO `platform_user` VALUES (1,'rapp','rapp','Person_DpphmPqg','el','rapp',1000,1);
/*!40000 ALTER TABLE `platform_user` ENABLE KEYS */;
UNLOCK TABLES;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8 */ ;
/*!50003 SET character_set_results = utf8 */ ;
/*!50003 SET collation_connection  = utf8_general_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = '' */ ;
DELIMITER ;;
/*!50003 CREATE*/ /*!50017 DEFINER=`root`@`localhost`*/ /*!50003 trigger `user_created_BEFORE_INSERT`
  before insert on `platform_user`
  for each row
    set NEW.creation_time = UNIX_TIMESTAMP(UTC_TIMESTAMP()) */;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2016-03-31 19:03:33
