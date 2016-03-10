-- MySQL dump 10.13  Distrib 5.5.47, for debian-linux-gnu (x86_64)
--
-- Host: localhost    Database: RappStore
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
-- Table structure for table `SSHkey`
--

DROP TABLE IF EXISTS `SSHkey`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `SSHkey` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `user` int(10) unsigned NOT NULL,
  `keyfile` varchar(256) NOT NULL,
  `fingerprint` char(64) DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `user` (`user`),
  CONSTRAINT `fk_user` FOREIGN KEY (`user`) REFERENCES `users` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `SSHkey`
--

LOCK TABLES `SSHkey` WRITE;
/*!40000 ALTER TABLE `SSHkey` DISABLE KEYS */;
/*!40000 ALTER TABLE `SSHkey` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `application_token`
--

DROP TABLE IF EXISTS `application_token`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `application_token` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `token` varchar(256) NOT NULL,
  `robot_id` int(10) unsigned NOT NULL,
  PRIMARY KEY (`id`),
  KEY `application_token_ibfk_1` (`robot_id`),
  CONSTRAINT `application_token_ibfk_1` FOREIGN KEY (`robot_id`) REFERENCES `robots` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `application_token`
--

LOCK TABLES `application_token` WRITE;
/*!40000 ALTER TABLE `application_token` DISABLE KEYS */;
INSERT INTO `application_token` VALUES (1,'rapptesttoken',0);
/*!40000 ALTER TABLE `application_token` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `application_token_services`
--

DROP TABLE IF EXISTS `application_token_services`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `application_token_services` (
  `token_id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `service_name` varchar(128) NOT NULL,
  PRIMARY KEY (`token_id`,`service_name`),
  CONSTRAINT `application_token_services_ibfk_1` FOREIGN KEY (`token_id`) REFERENCES `application_token` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `application_token_services`
--

LOCK TABLES `application_token_services` WRITE;
/*!40000 ALTER TABLE `application_token_services` DISABLE KEYS */;
INSERT INTO `application_token_services` VALUES (1,'rapp_service_name');
/*!40000 ALTER TABLE `application_token_services` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `cloud_agent`
--

DROP TABLE IF EXISTS `cloud_agent`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `cloud_agent` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `user_id` int(10) unsigned NOT NULL,
  `image_identifier` varchar(256) NOT NULL,
  `tarball_path` varchar(256) NOT NULL,
  `container_identifier` varchar(256) NOT NULL,
  `container_type` varchar(256) NOT NULL,
  PRIMARY KEY (`id`),
  KEY `cloud_agent_ibfk_1` (`user_id`),
  CONSTRAINT `cloud_agent_ibfk_1` FOREIGN KEY (`user_id`) REFERENCES `users` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=12 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cloud_agent`
--

LOCK TABLES `cloud_agent` WRITE;
/*!40000 ALTER TABLE `cloud_agent` DISABLE KEYS */;
/*!40000 ALTER TABLE `cloud_agent` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `cloud_agent_service`
--

DROP TABLE IF EXISTS `cloud_agent_service`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `cloud_agent_service` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `cloud_agent_id` int(10) unsigned NOT NULL,
  `service_name` varchar(256) NOT NULL,
  `service_type` varchar(256) NOT NULL,
  `container_port` int(5) NOT NULL,
  `host_port` int(5) NOT NULL,
  PRIMARY KEY (`id`),
  KEY `cloud_agent_service_ibfk_1` (`cloud_agent_id`),
  CONSTRAINT `cloud_agent_service_ibfk_1` FOREIGN KEY (`cloud_agent_id`) REFERENCES `cloud_agent` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=12 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cloud_agent_service`
--

LOCK TABLES `cloud_agent_service` WRITE;
/*!40000 ALTER TABLE `cloud_agent_service` DISABLE KEYS */;
/*!40000 ALTER TABLE `cloud_agent_service` ENABLE KEYS */;
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
  `argument_name` varchar(256) NOT NULL,
  PRIMARY KEY (`id`),
  KEY `cloud_agent_service_arguments_ibfk_1` (`cloud_agent_service_id`),
  CONSTRAINT `cloud_agent_service_arguments_ibfk_1` FOREIGN KEY (`cloud_agent_service_id`) REFERENCES `cloud_agent_service` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=12 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cloud_agent_service_arguments`
--

LOCK TABLES `cloud_agent_service_arguments` WRITE;
/*!40000 ALTER TABLE `cloud_agent_service_arguments` DISABLE KEYS */;
/*!40000 ALTER TABLE `cloud_agent_service_arguments` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `downloads`
--

DROP TABLE IF EXISTS `downloads`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `downloads` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `owner` int(10) unsigned NOT NULL,
  `rapp` int(10) unsigned NOT NULL,
  `created` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  KEY `owner` (`owner`),
  KEY `rapp` (`rapp`),
  CONSTRAINT `downloads_ibfk_2` FOREIGN KEY (`rapp`) REFERENCES `rapps` (`id`),
  CONSTRAINT `downloads_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `users` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `downloads`
--

LOCK TABLES `downloads` WRITE;
/*!40000 ALTER TABLE `downloads` DISABLE KEYS */;
/*!40000 ALTER TABLE `downloads` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `keywords`
--

DROP TABLE IF EXISTS `keywords`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `keywords` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(32) NOT NULL,
  `created` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=26 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `keywords`
--

LOCK TABLES `keywords` WRITE;
/*!40000 ALTER TABLE `keywords` DISABLE KEYS */;
/*!40000 ALTER TABLE `keywords` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `languages`
--

DROP TABLE IF EXISTS `languages`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `languages` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(30) NOT NULL,
  `version` varchar(7) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `name` (`name`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `languages`
--

LOCK TABLES `languages` WRITE;
/*!40000 ALTER TABLE `languages` DISABLE KEYS */;
/*!40000 ALTER TABLE `languages` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `myrobots`
--

DROP TABLE IF EXISTS `myrobots`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `myrobots` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `owner` int(10) unsigned NOT NULL,
  `name` varchar(128) NOT NULL,
  `mac` char(17) NOT NULL,
  `token` char(64) DEFAULT NULL,
  `created` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `mac` (`mac`),
  KEY `owner` (`owner`),
  CONSTRAINT `myrobots_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `users` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `myrobots`
--

LOCK TABLES `myrobots` WRITE;
/*!40000 ALTER TABLE `myrobots` DISABLE KEYS */;
/*!40000 ALTER TABLE `myrobots` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `rapps`
--

DROP TABLE IF EXISTS `rapps`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `rapps` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(64) NOT NULL,
  `owner` int(10) unsigned NOT NULL,
  `robot` int(10) unsigned NOT NULL,
  `language` int(10) unsigned NOT NULL,
  `license` varchar(64) NOT NULL,
  `version` varchar(7) NOT NULL,
  `timestamp` datetime NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uc_key` (`name`,`robot`,`version`),
  KEY `owner` (`owner`),
  KEY `robot` (`robot`),
  KEY `language` (`language`),
  CONSTRAINT `rapps_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `users` (`id`) ON DELETE CASCADE,
  CONSTRAINT `rapps_ibfk_2` FOREIGN KEY (`robot`) REFERENCES `robots` (`id`) ON DELETE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=46 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `rapps`
--

LOCK TABLES `rapps` WRITE;
/*!40000 ALTER TABLE `rapps` DISABLE KEYS */;
/*!40000 ALTER TABLE `rapps` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `repos`
--

DROP TABLE IF EXISTS `repos`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `repos` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `owner` int(10) unsigned NOT NULL,
  `name` varchar(128) NOT NULL,
  `private` char(1) NOT NULL,
  `created` datetime DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uc_name` (`name`),
  KEY `owner` (`owner`),
  CONSTRAINT `repos_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `users` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=21 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `repos`
--

LOCK TABLES `repos` WRITE;
/*!40000 ALTER TABLE `repos` DISABLE KEYS */;
/*!40000 ALTER TABLE `repos` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `robots`
--

DROP TABLE IF EXISTS `robots`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `robots` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(30) NOT NULL,
  `model` varchar(30) NOT NULL,
  `producer` varchar(30) NOT NULL,
  `arch` varchar(6) NOT NULL,
  `platform` varchar(12) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robots`
--

LOCK TABLES `robots` WRITE;
/*!40000 ALTER TABLE `robots` DISABLE KEYS */;
INSERT INTO `robots` VALUES (0,'rapp','rapp','rapp','rapp','rapp');
/*!40000 ALTER TABLE `robots` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tags`
--

DROP TABLE IF EXISTS `tags`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tags` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(30) NOT NULL,
  `rapp` int(10) unsigned NOT NULL,
  PRIMARY KEY (`id`),
  KEY `rapp` (`rapp`),
  CONSTRAINT `tags_ibfk_1` FOREIGN KEY (`rapp`) REFERENCES `rapps` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=11 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tags`
--

LOCK TABLES `tags` WRITE;
/*!40000 ALTER TABLE `tags` DISABLE KEYS */;
/*!40000 ALTER TABLE `tags` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `users`
--

DROP TABLE IF EXISTS `users`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `users` (
  `id` int(10) unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(128) NOT NULL,
  `email` varchar(256) NOT NULL,
  `pwd` char(60) NOT NULL,
  `activated` char(1) NOT NULL,
  `language` varchar(32) NOT NULL,
  `ontology_alias` varchar(32) DEFAULT NULL,
  PRIMARY KEY (`id`,`email`),
  UNIQUE KEY `c_id` (`email`),
  UNIQUE KEY `user_idx` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=14 DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `users`
--

LOCK TABLES `users` WRITE;
/*!40000 ALTER TABLE `users` DISABLE KEYS */;
INSERT INTO `users` VALUES (0,'rapp','rapp@rapp.com','rappPass','Y','el','Person_DpphmPqg');
/*!40000 ALTER TABLE `users` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2016-03-10 18:06:25
