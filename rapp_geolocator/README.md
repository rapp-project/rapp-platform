Documentation about RAPP Geolocator: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Geolocator)

RAPP Geolocator allows a user to get information about his location using his
IP address.
It is mainly considered an intermediate node that provides information to other
nodes such as
[RAPP Weather Reporter](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Weather-Reporter).
The availability of geolocator services that rely on third party APIs such as
IP-API is restricted according to the APIs' rules and limitations.
Thus, service call failures may exist.

Currently supported geolocators:
* [IP-API](http://ip-api.com/)

# ROS Service

## Locate

Service URL: `/rapp/rapp_geolocator/locate`

Service Type:

GeolocatorSrv.srv

```
string ip
string geolocator
---
string error

string city
string country
string countryCode
float32 latitude
float32 longtitude
string regionName
string timezone
string zip
```
**Available geolocator values:**
* '' (uses a default geolocator)
* 'ip-api'

# Launchers

## Standard launcher

Launches the rapp geolocator node and can be invoked by executing:

`roslaunch rapp_geolocator geolocator.launch`

# HOP Services

Service URL: `localhost:9001/hop/geolocation`

## Input/Output

```
Input = {
    ipaddr: '',
    engine: ''
  }
```
**Available engine values:**
* '' (uses a default engine)
* 'ip-api'

```
Output = {
    city: '',
    country: '',
    country_code: '',
    latitude: 0.0,
    longtitude: 0.0,
    region: '',
    timezone: '',
    zip: '',
    error: ''
  }
```
