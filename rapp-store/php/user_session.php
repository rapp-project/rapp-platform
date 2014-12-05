<?php

class UserSession
{
    var $usr;
    var $pwd;
    var $ip;
    var $group;
    
    function __construct( $username, $passhash )
    {
        if ( empty( $username ) or empty( $passhash ) )
            throw new Exception ( "UserSession::__construct null param(s)" );
        
        $this->ip = $_SERVER['REMOTE_ADDR']?:($_SERVER['HTTP_X_FORWARDED_FOR']?:$_SERVER['HTTP_CLIENT_IP']);
        $this->usr = $username;
        $this->pwd = $passhash;
        
        require_once ( '../db.php' );
        $mysqli = new mysqli( db_host, db_user, db_pwd, db_name );
    
        if ( mysqli_connect_errno() ) throw new Exception ( "UserSession::__construct Connect failed: " . mysqli_connect_error() );
        
        // Query DB - check if user is enabled and check user's group
        if ( $stmt = $mysqli->prepare( "SELECT `usrgroup` FROM `tblUser` WHERE `username`=? AND `pwd`=?") )
        {
            if ( !( $stmt->bind_param( 'ss', $_POST['username'], $_POST['password'] ) ) )
                throw new Exception ('UserSession::__construct bind_param() failed :' . $mysqli->error );
        
            if ( !( $stmt->execute() ) )
                throw new Exception ('UserSession::__construct execute() failed :' . $mysqli->error );
                
            if ( $result = $stmt->get_result() )
            {
                $row = $result->fetch_array( MYSQLI_ASSOC );
                if ( !empty( $row ) )
                {
                    $this->group = $row['usrgroup'];
            
                    if ( !session_start() )
                        throw new Exception( "UserSession::__construct failed to start session" );
                        
                    $mysqli->close();
                    // NOTE: Success !
                }
                else
                {
                    // Cleanup & Throw: We cannot have a username & password without a valid group and enabled - it makes no sense
                    $mysqli->close();
                    throw new Exception ( "UserSession::__construct @params username & passhash do not have a `group` or `enabled` in DB" );
                }
            }
            else
            {
                $mysqli->close();
                throw new Exception ( 'UserSession::__construct get_result() failed: ' . $mysqli->error );
            }
        }
        else
        {
            $mysqli->close();
            throw new Exception ('UserSession::__construct prepare() failed: ' . $mysqli->error );
        }
    }
}

?>