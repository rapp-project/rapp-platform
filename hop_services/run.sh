#!/bin/bash

# color list
colors ()
{
  RESET='\e[0m'
  BLACK='\e[1;30m'
  RED='\e[0;31m'
  GREEN='\e[0;32m'
  YELLOW='\e[0;33m'
  BLUE='\e[0;34m'
  PURPLE='\e[0;35m'
  CYAN='\e[0;36m'
  WHITE='\e[0;37m'

  BOIBLACK='\e[1;100m'
  BRED='\e[1;31m'
  BGREEN='\e[1;32m'
  BYELLOW='\e[1;33m'
  BBLUE='\e[1;34m'
  BPURPLE='\e[1;35m'
  BCYAN='\e[1;36m'
  BWHITE='\e[1;37m'
}; colors

## Get this script file directory name ##
CURRENTDIR=$(dirname ${BASH_SOURCE[0]})

## Set executable paths ##
SERVICEDIR=${CURRENTDIR}/services
JSFILE=init.js
JSEXECPATH=${SERVICEDIR}/${JSFILE}
##########################

## Hop Web Services will run on this $PORT ##
PORT=9001

## Define the scheduler type to be used ##
SCHEDULER="accept-many"

## Define the maximum number of simultaneous running threads ##
MAXTHREADS=100

## Logging definitions ##
# Timestamp to be added onto log file name #
TIMESTAMP=`date -d "today" +"%Y%m%d%H%M"`

LOGDIR="/home/${USER}/.hop/log/server"
LOGFILENAME="hop-server-${TIMESTAMP}.log"
LOGFILE="${LOGDIR}/${LOGFILENAME}"
#########################

## File caching definitions ##
CACHEDIR="/home/${USER}/.hop/cache/server"
CLEARCACHE=true
##############################

REPORT_EXECTIME=true

VERB_LEVEL=1
DEBUG_LEVEL=1
WARN_LEVEL=1


########## Assign arguments for Hop execution ##########
FLAGS=" -v${VERB_LEVEL} "
FLAGS+=" -g${DEBUG_LEVEL} "
FLAGS+=" -w${WARN_LEVEL} "

if [ ${CLEARCACHE} = true ]; then
  FLAGS+=" --clear-cache "
else
  FLAGS+=" --no-clear-cache "
fi

FLAGS+=" --cache-dir ${CACHEDIR} "
FLAGS+=" --http-port ${PORT} "
FLAGS+=" --log-file ${LOGFILE} "
FLAGS+=" --scheduler ${SCHEDULER}"
FLAGS+=" --max-threads ${MAXTHREADS}"

if [ ${REPORT_EXECTIME} = true ]; then
  FLAGS+=" --time "
fi
#########################################################

## If log directory does not exist... Maybe create it?!! ##
if [ ! -d ${LOGDIR} ]; then
  echo -e "--- Log Directory [${LOGDIR}] does not exist. Creating now."
  mkdir -p ${LOGDIR}
fi

## If log file does not exists create it. ##
if [ ! -f ${LOGFILE} ]; then
  touch ${LOGFILE}
fi

# If file caching directory does not exist... Maybe create it?!! ##
# Hop server creates it for us!!!!
#if [ ! -d ${CACHEDIR} ]; then
  #echo -e "--- Cache Directory [${CACHEDIR}] does not exist. Creating now."
  #mkdir -p ${CACHEDIR}
#fi

## Inform on HOP configuration ##
echo -e "${BYELLOW}Hop configuration parameters passed:${BLUE}"
echo -e "  * Http-Port: ${PORT}"
echo -e "  * Server Cache directory: ${CACHEDIR}"
echo -e "  * Clear cache directory: ${CLEARCACHE}"
echo -e "  * Server log directory: ${LOGDIR}"
echo -e "  * Scheduler policy: ${SCHEDULER}"
echo -e "  * Max threads: ${MAXTHREADS}"
echo -e "  * Report execution time: ${REPORT_EXECTIME}"
echo -e "  * Verbosity level: ${VERB_LEVEL}"
echo -e "  * Debug level: ${DEBUG_LEVEL}"
echo -e "  * Warning level: ${WARN_LEVEL}"
echo -e "${RESET}"
################################

#export NODE_PATH=$NODE_PATH:`pwd`/modules

echo -e "${BYELLOW}Initialing Hop Web Server and registering Web Services${RESET}"
set -o xtrace
hop ${FLAGS} ${JSEXECPATH}
set +o xtrace
