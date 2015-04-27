#!/usr/bin/python

__author__ = 'klpanagi'

import sys, argparse

parser = argparse.ArgumentParser(description='Hop service creator')

# Add to parser --> the given service name
parser.add_argument('-n','--name', help='Service name', dest='serviceName',
        action='store', nargs=1, type=str)
##############################

# Add to parser --> The given service arguments
parser.add_argument('-a', '--args', dest='serviceArgs', action='store',
        nargs='+', help='Number of service parameters')

parser.add_argument('--ros-srvParam-name', dest='rosSrvParamName', action='store',
        nargs=1, help='Number of service parameters')

args =  parser.parse_args( ) # Parse the given arguments

srvName = args.serviceName[0]
srvArgs = args.serviceArgs   # Service argument names
num_srvArgs = len(srvArgs)   # Number of given service arguments

##### Print information and evaluate before creating the relevant file
print 'ServiceName: ', srvName
print 'Number of service arguments: ', num_srvArgs
print 'Service arguments: ', srvArgs
target_fileName = srvName + ".service.js"
print 'Target File Name for creation:', target_fileName
######################################################################

f = open(target_fileName, "w")

## Create basic structure template
lines = []
lines.append("/*!\n")
lines.append(" * @file " + target_fileName+ "\n")
lines.append(" *\n")
lines.append(" */\n")

lines.append("var ROSbridge = require(/*rapp_hop_path +*/'../utilities/./rosbridge.js');\n")
lines.append("var RandStringGen = require ( /*rapp_hop_path +*/ '../utilities/./randStringGen.js' );\n")
lines.append("/*----------------------------------------------*/\n")
lines.append("/*-----<Defined Name of QR Node ROS service>----*/\n")

rosService = '/ric/knowrob/subclasses_of'
lines.append("var subclassesOf_rosService = '" + rosService + "';\n")

lines.append("/*---Initiatess Communication with RosBridge (Global)---*/\n")
lines.append("var rosbridge = new ROSbridge();\n")
lines.append("/*------------------------------------------------------*/\n")
lines.append("/*----<Random String Generator configurations---->*/\n")
lines.append("var stringLength = 5;\n")
lines.append("var randStrGen = new RandStringGen( stringLength );\n")
lines.append("/*------------------------------------------------*/\n")
lines.append("\n")

serviceDef = "service " + srvName + "( {"
for i in range(len(srvArgs)):
    if i!=0: # If it is NOT the first argument add parameter seperator
        serviceDef = serviceDef + ", "
    serviceDef = serviceDef + srvArgs[i] + ":''" 
serviceDef = serviceDef + "} )\n"
lines.append(serviceDef)

lines.append("{\n")
lines.append("  console.log('[" + srvName + "]: Client Request');\n")
lines.append("  rosbridge.connect();\n")
lines.append("\n")  

lines.append("  var args = createServiceArgs( srv_msgArgs );\n")

lines.append("  /*-----<Call subclassesOf ROS service through rosbridge>-----*/\n")
lines.append("  var returnMessage = rosbridge.callServiceSync( subclassesOf_rosService, args );\n")
lines.append("  rosbridge.close();\n")
lines.append("  /*--<Returned message from qr ROS service>--*/\n")
lines.append("  return  JSON.stringify( returnMessage.values.results )// JSON msg\n")
lines.append("};\n")
lines.append("\n\n")

lines.append("function createServiceArgs( msgArgs_value )\n")
lines.append("{\n")

## TODO -- How to create different service msgs?? Investigate!!!
rosSrv_paramName = args.rosSrvParamName[0]
#rosSrv_paramValue = 'msgArgs'
lines.append("  var " + rosSrv_paramName + " = {\n")
lines.append("    'data': msgArgs_value\n")
lines.append("  };\n")

lines.append("  var args = {};\n")
#lines.append("  for ( var key in _args ){\n")
#lines.append("    args[ key.toString() ] = _args[ key ];\n")
#lines.append("  }\n")

lines.append("  args[ '" + rosSrv_paramName + "' ] = " + rosSrv_paramName + ";\n")
lines.append("\n")
lines.append("  return args;\n")
lines.append("};\n")

for i in range(len(lines)):
    f.write(lines[i])







f.close()
