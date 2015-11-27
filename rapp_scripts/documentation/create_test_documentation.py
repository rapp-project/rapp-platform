#!/usr/bin/env python

#Copyright 2015 RAPP

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

    #http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# Authors: Aris Thallas
# Contact: aris.thallas@{iti.gr, gmail.com}

import os
import sys
import string
import shutil
import subprocess

INFO = '\033[1m\033[103m\033[31m'
DEFAULT = '\033[0m'
PATH = '\033[1m'
ERROR = "\033[1;90m\033[101m"


# Documentation Path
documentPath = os.path.join( os.environ['HOME'], \
  'rapp_platform_files/documentation/platform_tests' )
# Unwanted directories
blacklist = [ '.git', 'rapp_web_services' ]
# Doxygen root folder
platform_root = '../../'


# Create capitalized space seperated package name without the rapp word
def capitalizePackage(packageName):
  packageTransformedName = packageName.split("_")
  packageTransformedName.remove('rapp')
  packageTransformedName = ' '.join(packageTransformedName)
  packageTransformedName = string.capwords(packageTransformedName)
  return packageTransformedName

# Create doxygen documentation for a specific package
def createDoxy( packagePath, packageName, packageTransformedName, testPkg):

  # Create temp configuration
  doxyConf =  './doxy_conf_platform_tests'
  tempConf = doxyConf + packageName
  shutil.copy( doxyConf, tempConf )

  # General params
  projectName = "\"RAPP Platform Tests - " + packageTransformedName + " \""
  if testPkg:
    packageTestPath = packagePath
  else:
    packageTestPath = os.path.join( packagePath, 'tests' )
  outputFile = './platform_test_' + packageName

  # Doxygen Configuration params
  doxInput = 'INPUT = ' + packageTestPath
  doxPatterns = 'FILE_PATTERNS = '
  doxProjectName = 'PROJECT_NAME = ' + projectName
  doxOutput = 'OUTPUT_DIRECTORY = ' + outputFile

  # Temp configuration modification
  doxyFD = open( tempConf, 'a' )
  doxyFD.write( doxInput + '\n' )
  doxyFD.write( doxPatterns + '\n' )
  doxyFD.write( doxProjectName + '\n' )
  doxyFD.write( doxOutput + '\n' )
  doxyFD.close()

  try:
    from subprocess import DEVNULL
  except ImportError:
    DEVNULL = open(os.devnull, 'wb')
  subprocess.call( ['doxygen', tempConf], stdout = DEVNULL, stderr = DEVNULL )
  DEVNULL.close()

  os.remove( tempConf )

  shutil.move( outputFile, documentPath )

def createLatex( packageName ):
  initialPath = os.getcwd()
  packageTestPath = 'platform_test_' + packageName
  latexPath = os.path.join( documentPath, packageTestPath , 'latex' )
  os.chdir( latexPath )
  try:
    from subprocess import DEVNULL
  except ImportError:
    DEVNULL = open(os.devnull, 'wb')
  subprocess.call( ['make'], stdout = DEVNULL, stderr = DEVNULL )
  DEVNULL.close()

  shutil.copy( 'refman.pdf', '../' + packageName + '.pdf')

  os.chdir( initialPath )


def main():

  if os.path.exists(documentPath):
    shutil.rmtree( documentPath )
  os.makedirs(documentPath)

  platform_dirs = os.walk( platform_root ).next()[1]

  for blackDir in blacklist:
    if blackDir in platform_dirs:
      platform_dirs.remove( blackDir )

  for packageName in platform_dirs:
    packagePath = os.path.join( platform_root + packageName )
    pack_dirs = os.walk( packagePath ).next()[1]
    if 'tests' in pack_dirs or 'test' in packageName:

      testPkg = False
      if 'test' in packageName:
        testPkg = True

      packageTransformedName = capitalizePackage( packageName )
      print INFO + '[RAPP] Creating Test Documentation for Package: ' + \
          packageTransformedName + DEFAULT

      print INFO + '[RAPP] Creating Doxygen' + DEFAULT
      createDoxy( packagePath, packageName, packageTransformedName, testPkg)

      print INFO + '[RAPP] Creating LaTeX pdf' + DEFAULT
      createLatex( packageName )

      print INFO + \
        '[RAPP] Finished Cleanly. Documentation can be accessed in: ' + DEFAULT
      print PATH + \
        os.path.join( documentPath, 'platform_test_' + packageName, ) + \
        DEFAULT + '\n'

  print INFO + \
    '[RAPP] Finished Cleanly. Documentation can be accessed in: ' + DEFAULT
  print PATH + documentPath + DEFAULT


if __name__ == '__main__':
  main()
