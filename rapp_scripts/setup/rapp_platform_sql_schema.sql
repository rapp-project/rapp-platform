CREATE TABLE IF NOT EXISTS `platform_user` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `username` varchar(32) NOT NULL UNIQUE,
  `password` varchar(64) NOT NULL,
  `device_token` varchar(128) NOT NULL,
  `creation_time` bigint unsigned NOT NULL,
  `status` tinyint unsigned DEFAULT 1
) CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `application_token` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `token` varchar(256) NOT NULL,
  `platform_user_id` int unsigned NOT NULL,
  `device_token` varchar(256) NOT NULL,
  `status` tinyint unsigned DEFAULT 1,
  `creation_time` bigint unsigned NOT NULL,
  `expiration_time` bigint unsigned NOT NULL,
  `last_update_time` bigint unsigned NOT NULL,

  FOREIGN KEY (`platform_user_id`) REFERENCES `platform_user` (`id`)
) CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `cloud_agent` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `platform_user_id` int unsigned NOT NULL,
  `tarball_path` varchar(256) NOT NULL,
  `container_identifier` varchar(256) NOT NULL,
  `image_identifier` varchar(256) NOT NULL,
  `container_type` varchar(256) NOT NULL,

  FOREIGN KEY (`platform_user_id`) REFERENCES `platform_user` (`id`)
) CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `cloud_agent_services` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `cloud_agent_id` int unsigned NOT NULL,
  `service_name` varchar(256) NOT NULL,
  `service_type` varchar(32) NOT NULL,
  `container_port` smallint unsigned NOT NULL,
  `host_port` smallint unsigned NOT NULL,

  FOREIGN KEY (`cloud_agent_id`) REFERENCES `cloud_agent` (`id`)
) CHARSET=utf8;

CREATE TABLE IF NOT EXISTS `cloud_agent_service_arguments` (
  `id` int unsigned AUTO_INCREMENT PRIMARY KEY,
  `cloud_agent_service_id` int unsigned NOT NULL,
  `argument_name` varchar(64) NOT NULL,

  FOREIGN KEY (`cloud_agent_service_id`) REFERENCES `cloud_agent_services` (`id`)
) CHARSET=utf8;


-- Add created timestamps on element creation
drop trigger if exists `user_created_BEFORE_INSERT`;
create trigger `user_created_BEFORE_INSERT`
  before insert on `platform_user`
  for each row
    set NEW.creation_time = UNIX_TIMESTAMP(UTC_TIMESTAMP());

drop trigger if exists `token_created_BEFORE_INSERT`;
create trigger `token_created_BEFORE_INSERT`
  before insert on `application_token`
  for each row
    set NEW.creation_time = UNIX_TIMESTAMP(UTC_TIMESTAMP());


drop trigger if exists `token_updated_BEFORE_UPDATE`;
create trigger `token_updated_BEFORE_UPDATE`
  before update on `application_token`
  for each row
     set NEW.`last_update_time` = UNIX_TIMESTAMP(UTC_TIMESTAMP());
