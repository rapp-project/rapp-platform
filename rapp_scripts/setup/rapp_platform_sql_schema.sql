CREATE TABLE IF NOT EXISTS `language` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `name` varchar(32) NOT NULL UNIQUE
) ENGINE=InnoDB CHARSET=utf8;

INSERT INTO `language` (`name`) VALUES ('el');

CREATE TABLE IF NOT EXISTS `platform_user` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `username` varchar(32) NOT NULL UNIQUE,
  `password` varchar(64) NOT NULL,
  `status` tinyint unsigned DEFAULT 1,
  `role` tinyint unsigned DEFAULT 20,
  `creator_id` int unsigned NOT NULL,

  `ontology_alias` varchar(64) DEFAULT NULL,
  `language_id` int unsigned,

  `creation_time` bigint unsigned NOT NULL DEFAULT 1,

  FOREIGN KEY (`language_id`) REFERENCES `language` (`id`)
) ENGINE=InnoDB CHARSET=utf8;

INSERT INTO `platform_user`
  (username, password, ontology_alias, language_id, role, creator_id)
  VALUES(
    'rapp',
    '$2b$12$0RzTZr6bjbqRDTzT4SYBV.I44fG6RHUjMtqxeP2c6Qaansh03GhTC',
    'Person_DpphmPqg',
    (SELECT `id` FROM `language` WHERE `name`='el'),
    0,
    1
  );

ALTER TABLE `platform_user` ADD
  FOREIGN KEY (`creator_id`) REFERENCES `platform_user` (`id`);

CREATE TABLE IF NOT EXISTS `device` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `token` varchar(128) NOT NULL UNIQUE,
  `description` text,
  `status` tinyint unsigned DEFAULT 1
) ENGINE=InnoDB CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `application_token` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `token` varchar(256) NOT NULL,
  `platform_user_id` int unsigned NOT NULL,
  `device_id` int unsigned NOT NULL,
  `status` tinyint unsigned DEFAULT 1,

  `creation_time` bigint unsigned NOT NULL DEFAULT 1,
  `expiration_time` bigint unsigned NOT NULL DEFAULT 1,
  `last_update_time` bigint unsigned NOT NULL DEFAULT 1,

  FOREIGN KEY (`platform_user_id`) REFERENCES `platform_user` (`id`),
  FOREIGN KEY (`device_id`) REFERENCES `device` (`id`)
) ENGINE=InnoDB CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `owner_user_device` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `user_id` int unsigned NOT NULL,
  `device_id` int unsigned NOT NULL,

  FOREIGN KEY (`user_id`) REFERENCES `platform_user` (`id`),
  FOREIGN KEY (`device_id`) REFERENCES `device` (`id`)
) ENGINE=InnoDB CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `cloud_agent` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `platform_user_id` int unsigned NOT NULL,
  `tarball_path` varchar(256) NOT NULL,
  `container_identifier` varchar(256) NOT NULL,
  `image_identifier` varchar(256) NOT NULL,
  `container_type` varchar(256) NOT NULL,

  FOREIGN KEY (`platform_user_id`) REFERENCES `platform_user` (`id`)
) ENGINE=InnoDB CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `cloud_agent_services` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `cloud_agent_id` int unsigned NOT NULL,
  `service_name` varchar(256) NOT NULL,
  `service_type` varchar(32) NOT NULL,
  `container_port` smallint unsigned NOT NULL,
  `host_port` smallint unsigned NOT NULL,

  FOREIGN KEY (`cloud_agent_id`) REFERENCES `cloud_agent` (`id`)
) ENGINE=InnoDB CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `cloud_agent_service_arguments` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `cloud_agent_service_id` int unsigned NOT NULL,
  `argument_name` varchar(64) NOT NULL,

  FOREIGN KEY (`cloud_agent_service_id`) REFERENCES `cloud_agent_services` (`id`)
) ENGINE=InnoDB CHARSET=utf8;


-- Add created timestamps on element creation
DROP TRIGGER IF EXISTS `user_created_BEFORE_INSERT`;
CREATE TRIGGER `user_created_BEFORE_INSERT`
BEFORE INSERT ON `platform_user`
FOR EACH ROW
  SET NEW.creation_time = UNIX_TIMESTAMP(UTC_TIMESTAMP());

DROP TRIGGER IF EXISTS `token_created_BEFORE_INSERT`;
CREATE TRIGGER `token_created_BEFORE_INSERT`
BEFORE INSERT ON `application_token`
FOR EACH ROW
    SET NEW.creation_time = UNIX_TIMESTAMP(UTC_TIMESTAMP()),
    NEW.last_update_time = UNIX_TIMESTAMP(UTC_TIMESTAMP());


DROP TRIGGER IF EXISTS `token_updated_BEFORE_UPDATE`;
CREATE TRIGGER `token_updated_BEFORE_UPDATE`
  BEFORE UPDATE ON `application_token`
  FOR EACH ROW
     SET NEW.last_update_time = UNIX_TIMESTAMP(UTC_TIMESTAMP());
