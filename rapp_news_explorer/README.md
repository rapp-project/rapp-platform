Documentation about the RAPP News Explorer: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-News-Explorer)

Rapp News Explorer allows users to search for news articles employing various
news search engine APIs.
The user can specify the desired news search engine, topics (i.e. sports,
economy, headlines, etc.), desired keywords, language/region (can be specified
employing
[RAPP Geolocator](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Geolocator)),
etc. The node returns a list of related articles.

Currently supported News Engines:
* Google News ([API](https://developers.google.com/news-search/v1/devguide))

# ROS Services

## Fetch News

Service URL: `/rapp/rapp_news_explorer/fetch_news`

Service Type:

NewsExplorerSrv.srv
```
# Request
# The news search engine
string newsEngine
# Desired keywords
string[] keywords
# Reject list of previously read articles, in order to avoid duplicates
string[] excludeTitles
# Language/Region
string regionEdition
# Main topics, i.e. sports, politics etc
string topic
# Number of news stories
int8 storyNum

---
# Response
string error           # Error description

# List of news articles
rapp_platform_ros_communications/NewsStoryMsg[] stories
```
**Available newsEngine values:**
* '' (uses default news engine)
* 'google'

NewsStoryMsg.msg
```
# Article title
string title
# Article brief content
string content
# Article publisher
string publisher
# Article publication date
string publishedDate
# Article original url
string url
```

# Launchers

## Standard launcher
Launches the rapp_news_explorer node and can be invoked by executing:

`roslaunch rapp_news_explorer news_explorer.launch`

# HOP Services

Service URL: `localhost:9001/hop/news_explore`

## Input/Output

```
Input = {
    news_engine: '',
    keywords: [],
    exclude_titles: [],
    region: '',
    topic: '',
    num_news: 25
  }
```
**Available news_engine values:**
* '' (uses default news engine)
* 'google'

```
Output = {
    news_stories: [],
    error: ''
  }
```
