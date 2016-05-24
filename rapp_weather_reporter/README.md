Documentation about RAPP Weather Reporter: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Weather-Reporter)

RAPP Weather Reporter allows users to get details about current or future
weather conditions.
It provides two services; a) current weather conditions and b) weather
forecast.
The user has to provide his current city, which could be evaluated using
[RAPP Geolocator](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Geolocator).
The availability of weather services that rely on third party APIs such
as YWeahter are restricted according to the APIs' rules and limitations.
Thus, service call failures may exist.

Currently supported weather reporters:
* [YWeather](https://pypi.python.org/pypi/yweather/)
* [ForecastIO](https://forecast.io) (default)

In order for the ForecastIO weather reporter to be deployed an API key is
required.
After obtaining the key via registration, the key should be stored in the
following location:
```
${HOME}/.config/rapp_platform/api_keys/forecast.io
```

# ROS Services

## Current Weather
Service URL: `/rapp/rapp_weather_reporter/current_weather`

Service Type:
```
# The desired city
string city
# The weather API
string weather_reporter
# The return value units
int8 metric
---
string error

string date
string temperature
string weather_description
string humidity
string visibility
string pressure
string wind_speed
string wind_temperature
string wind_direction
```
**Available weather_reporter values:**
* '' (uses default weather reporter)
* 'yweather'
* 'forecastio'


## Weather Forecast
Service URL: `/rapp/rapp_weather_reporter/weather_forecast`

Service Type:
```
# The desired city
string city
# The weather API
string weather_reporter
# The return value units
int8 metric
---
string error

rapp_platform_ros_communications/WeatherForecastMsg[] forecast
```

**Available weather_reporter values:**
* '' (uses default weather reporter)
* 'yweather'
* 'forecastio'

WeatherForecastMsg.msg

```
string high_temperature
string low_temperature
string description
string date
```

# Launchers

## Standard Launcher

Launches the rapp weather reporter node and can be invoked by executing:

`roslaunch rapp_weather_reporter weather_reporter.launch`

# HOP Services

## Current weather

Service URL: `localhost:9001/hop/weather_report_current`

### Input/Output
```
Input = {
    city: '',
    weather_reporter: '',
    metric: 0
  }
```

**Available weather_reporter values:**
* '' (uses default weather reporter)
* 'yweather'
* 'forecastio'

```
Output = {
    date: '',
    temperature: '',
    weather_description: '',
    humidity: '',
    visibility: '',
    pressure: '',
    wind_speed: '',
    wind_temperature: '',
    wind_direction: '',
    error = ''
  }
```

## Weather forecast

Service URL: `localhost:9001/hop/weather_report_forecast`

### Input/Output
```
Input = {
    city: '',
    weather_reporter: '',
    metric: 0
  }
```

**Available weather_reporter values:**
* '' (uses default weather reporter)
* 'yweather'
* 'forecastio'

```
Output = {
    forecast: [],
    error: ''
  }
```
