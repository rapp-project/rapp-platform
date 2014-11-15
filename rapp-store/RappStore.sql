-- phpMyAdmin SQL Dump
-- version 3.4.11.1deb2+deb7u1
-- http://www.phpmyadmin.net
--
-- Host: localhost
-- Generation Time: Nov 15, 2014 at 06:35 AM
-- Server version: 5.5.40
-- PHP Version: 5.4.4-14+deb7u14

SET SQL_MODE="NO_AUTO_VALUE_ON_ZERO";
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;

--
-- Database: `RappStore`
--

-- --------------------------------------------------------

--
-- Table structure for table `tblModel`
--
-- Creation: Nov 15, 2014 at 06:32 AM
--

DROP TABLE IF EXISTS `tblModel`;
CREATE TABLE IF NOT EXISTS `tblModel` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `model_str` varchar(128) NOT NULL,
  `manufacturer` varchar(128) NOT NULL,
  `version` decimal(18,9) NOT NULL,
  `arch` varchar(12) NOT NULL,
  `os` varchar(12) NOT NULL,
  `picture` varchar(128) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uid` (`model_str`,`manufacturer`,`version`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8 AUTO_INCREMENT=1 ;

-- --------------------------------------------------------

--
-- Table structure for table `tblRapp`
--
-- Creation: Nov 15, 2014 at 06:27 AM
--

DROP TABLE IF EXISTS `tblRapp`;
CREATE TABLE IF NOT EXISTS `tblRapp` (
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
  KEY `fk_owner` (`owner`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8 AUTO_INCREMENT=1 ;

--
-- RELATIONS FOR TABLE `tblRapp`:
--   `owner`
--       `tblUser` -> `id`
--

-- --------------------------------------------------------

--
-- Table structure for table `tblRobot`
--
-- Creation: Nov 15, 2014 at 06:33 AM
--

DROP TABLE IF EXISTS `tblRobot`;
CREATE TABLE IF NOT EXISTS `tblRobot` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `macddr` bigint(8) NOT NULL,
  `model` int(12) NOT NULL,
  `owner` int(12) NOT NULL,
  `timestamp` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`id`),
  UNIQUE KEY `uid` (`macddr`,`model`),
  KEY `fk_owner` (`owner`),
  KEY `fk_model` (`model`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8 AUTO_INCREMENT=1 ;

--
-- RELATIONS FOR TABLE `tblRobot`:
--   `model`
--       `tblModel` -> `id`
--   `owner`
--       `tblUser` -> `id`
--

-- --------------------------------------------------------

--
-- Table structure for table `tblUser`
--
-- Creation: Nov 14, 2014 at 07:21 PM
--

DROP TABLE IF EXISTS `tblUser`;
CREATE TABLE IF NOT EXISTS `tblUser` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `firstname` varchar(128) NOT NULL,
  `lastname` varchar(128) NOT NULL,
  `email` varchar(254) NOT NULL,
  `pwd` char(64) CHARACTER SET ascii COLLATE ascii_bin NOT NULL,
  `usrgroup` varchar(5) NOT NULL DEFAULT 'guest',
  `created` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `accessed` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00',
  `enabled` tinyint(1) NOT NULL DEFAULT '0',
  UNIQUE KEY `id` (`id`),
  UNIQUE KEY `uid` (`firstname`,`lastname`,`email`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8 AUTO_INCREMENT=1 ;

--
-- Constraints for dumped tables
--

--
-- Constraints for table `tblRapp`
--
ALTER TABLE `tblRapp`
  ADD CONSTRAINT `tblRapp_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `tblUser` (`id`);

--
-- Constraints for table `tblRobot`
--
ALTER TABLE `tblRobot`
  ADD CONSTRAINT `tblRobot_ibfk_2` FOREIGN KEY (`model`) REFERENCES `tblModel` (`id`),
  ADD CONSTRAINT `tblRobot_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `tblUser` (`id`);

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
