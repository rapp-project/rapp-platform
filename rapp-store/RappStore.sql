-- phpMyAdmin SQL Dump
-- version 3.4.11.1deb2+deb7u1
-- http://www.phpmyadmin.net
--
-- Host: localhost
-- Generation Time: Nov 17, 2014 at 04:39 AM
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
--   `owner`
--       `tblUser` -> `id`
--   `model`
--       `tblModel` -> `id`
--

-- --------------------------------------------------------

--
-- Table structure for table `tblUser`
--
-- Creation: Nov 17, 2014 at 03:17 AM
--

DROP TABLE IF EXISTS `tblUser`;
CREATE TABLE IF NOT EXISTS `tblUser` (
  `id` int(12) NOT NULL AUTO_INCREMENT,
  `username` varchar(32) NOT NULL,
  `firstname` varchar(128) NOT NULL,
  `lastname` varchar(128) NOT NULL,
  `email` varchar(254) NOT NULL,
  `pwd` char(64) CHARACTER SET ascii COLLATE ascii_bin NOT NULL,
  `usrgroup` int(1) NOT NULL DEFAULT '5',
  `created` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  `accessed` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00',
  `enabled` tinyint(1) NOT NULL DEFAULT '0',
  `activation` binary(16) NOT NULL COMMENT 'uuid-v4',
  UNIQUE KEY `id` (`id`),
  UNIQUE KEY `uid` (`firstname`,`lastname`,`email`),
  UNIQUE KEY `username` (`username`)
) ENGINE=InnoDB  DEFAULT CHARSET=utf8 AUTO_INCREMENT=8 ;

--
-- Dumping data for table `tblUser`
--

INSERT INTO `tblUser` (`id`, `username`, `firstname`, `lastname`, `email`, `pwd`, `usrgroup`, `created`, `accessed`, `enabled`, `activation`) VALUES
(1, 'admin', 'Alex', 'Giokas', 'a.gkiokas@ortelio.co.uk', '486d18ed96603f0bbae2480e2c98cc80750402b28c1d4069d5df7c570ded0307', 0, '2014-11-15 18:01:34', '0000-00-00 00:00:00', 1, '\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'),
(7, 'bsmith', 'Bob', 'Smith', 'alexge233@hotmail.com', '7b4b075939a9c6823f85c35c19d029e2f91e8db8eabdd96a4cc0b4c8328df1c3', 2, '2014-11-17 03:18:03', '0000-00-00 00:00:00', 0, 'œK³Ý	¾@4°±œK‘{ÛH');

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
  ADD CONSTRAINT `tblRobot_ibfk_1` FOREIGN KEY (`owner`) REFERENCES `tblUser` (`id`),
  ADD CONSTRAINT `tblRobot_ibfk_2` FOREIGN KEY (`model`) REFERENCES `tblModel` (`id`);

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
