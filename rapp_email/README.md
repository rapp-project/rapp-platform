Documentation about RAPP Email: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Email)

RAPP Email provides an interface to allow users to handle their email accounts.
It provides two services; fetch user's emails and send new email.
The receive email service supports the IMAP protocol and the send mail the
classic SMTP protocol.
RAPP Email *does* **not** *implement an email server* and requires the user to
have his own email provider who allows connections to the IMAP and SMTP servers.

# ROS Services

## Fetch Emails
Service URL: ```/rapp/rapp_email/receive_email```

Service type:

ReceiveEmailSrv.srv
```bash
# The user email username
string email
# The user email password
string password
# The email server's imap address, i.e. 'imap.gmail.com'
string server
# The email server imap port
string port

# Define which mails the users requests.
# Values: ALL, UNSEEN(DEFAULT)
string requestedEmailStatus

# Emails since date (unix timestamp)
uint64 fromDate
# Emails until date (unix timestamp)
uint64 toDate
# Number of requested emails
uint16 numberOfEmails

---
# Response

# 0 success, -1 failure
int8 status

# The requested emails
rapp_platform_ros_communications/MailMsg[] emails
```

MailMsg.msg
```
# Path to the email body file
string bodyPath

string subject
string sender
string[] receivers
string dateTime

# Paths to the email's attachments
string[] attachmentPaths
```


## Send Emails
Service URL: ```/rapp/rapp_email/send_email```

Service type:

SendEmailSrv.srv
```bash
# The user email username
string userEmail
# The user email password
string password
# The email server's smtp address, i.e. 'smtp.gmail.com'
string server
# The email server smtp port
string port

# Email addresses of the recipients
string[] recipients

# The email body
string body
# The email subject
string subject

# File paths of the attachments
string[] files

---
# Response

# 0 success, -1 failure
int8 status
```

# Launchers

## Standard launcher

Launches the **rapp email** node and can be invoked using:
```bash
roslaunch rapp_email email.launch
```

# HOP Services

## Send Email

Service URL: `localhost:9001/hop/email_send`

### Input/Output
```
Input = {
    file_uri: '',
    email: '',
    passwd: '',
    server: '',
    port: '',
    recipients: [],
    body: '',
    subject: ''
  }
```
```
Output = {
    error: ''
  }
```

## Receive Email

Service URL: `localhost:9001/hop/email_fetch`

### Input/Output
```
Input = {
    email: '',
    passwd: '',
    server: '',
    port: '',
    email_status: '',
    from_date: 0,
    to_date: 0,
    num_emails: 0
  }
```
```
Output = {
    emails: [],
    error: ''
  }
```
