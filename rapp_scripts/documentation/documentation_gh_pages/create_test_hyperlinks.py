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

testFolder = './platform_tests'

# Create capitalized space seperated package name without the rapp word
def capitalizePackage(packageName):
  packageTransformedName = packageName.split("_")
  packageTransformedName.remove('rapp')
  packageTransformedName.remove('platform')
  packageTransformedName.remove('test')
  packageTransformedName = ' '.join(packageTransformedName)
  packageTransformedName = string.capwords(packageTransformedName)
  return packageTransformedName

def main():

  platform_dirs = os.walk( testFolder ).next()[1]

  for packageName in platform_dirs:

    print packageName
    packageTransformedName = capitalizePackage( packageName )
    print packageTransformedName

    html = open( './documentation2.html', 'a' )
    html.write( '<li> ' + packageTransformedName + ' Tests </li>' + '\n <ul> \n' )

    testPackagePath = os.path.join( testFolder, packageName )

    html.write( '<li><a href="' + testPackagePath + '/html/index.html">HTML from Doxygen</a></li>\n' )

    pdf = packageName.split("_")
    pdf.remove('platform')
    pdf.remove('test')
    pdf = '_'.join(pdf)
    pdf += '.pdf'

    html.write( '<li><a href="' + testPackagePath + pdf + '">PDF from Doxygen</a></li>\n' )
    html.write( '</ul>\n' )
    html.close()


if __name__ == '__main__':
  main()
