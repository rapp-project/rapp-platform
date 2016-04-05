INSERT INTO `language` (`name`) VALUES ('el');

--  INSERT INTO `platform_user`
  --  (username, password, ontology_alias, language_id, role, creator_id)
  --  VALUES(
    --  'rapp',
    --  '$2b$12$0RzTZr6bjbqRDTzT4SYBV.I44fG6RHUjMtqxeP2c6Qaansh03GhTC',
    --  'Person_DpphmPqg',
    --  (SELECT `id` FROM `language` WHERE `name`='el'),
    --  0,
    --  1
  --  );
INSERT INTO `platform_user`
  (username, password, role, creator_id)
  VALUES(
    'rapp_store',
    '$2b$12$0RzTZr6bjbqRDTzT4SYBV.I44fG6RHUjMtqxeP2c6Qaansh03GhTC',
    20,
    (SELECT `id` FROM (SELECT * FROM `platform_user`) AS pl
      WHERE pl.`username`='rapp')
  );

INSERT INTO `device` (token, description)
  VALUES ('rapp_device', 'a sample device');

INSERT INTO `owner_user_device` (user_id, device_id)
  VALUES (
  (SELECT `id` FROM `platform_user` WHERE `username`='rapp'),
  (SELECT `id` FROM `device` WHERE `token`='rapp_device')
);


INSERT INTO `application_token` (token, platform_user_id, device_id)
VALUES(
  'rapp_token',
  (SELECT `id` FROM `platform_user` WHERE `username`='rapp'),
  (SELECT `id` FROM `device` WHERE `token`='rapp_device')
);
