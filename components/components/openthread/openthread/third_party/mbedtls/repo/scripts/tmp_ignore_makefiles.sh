#!/bin/bash

# Temporarily (de)ignore Makefiles generated by CMake to allow easier
# git development
#
# Copyright The Mbed TLS Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

IGNORE=""

# Parse arguments
#
until [ -z "$1" ]
do
  case "$1" in
    -u|--undo)
      IGNORE="0"
      ;;
    -v|--verbose)
      # Be verbose
      VERBOSE="1"
      ;;
    -h|--help)
      # print help
      echo "Usage: $0"
      echo -e "  -h|--help\t\tPrint this help."
      echo -e "  -u|--undo\t\tRemove ignores and continue tracking."
      echo -e "  -v|--verbose\t\tVerbose."
      exit 1
      ;;
    *)
      # print error
      echo "Unknown argument: '$1'"
      exit 1
      ;;
  esac
  shift
done

if [ "X" = "X$IGNORE" ];
then
  [ $VERBOSE ] && echo "Ignoring Makefiles"
  git update-index --assume-unchanged Makefile library/Makefile programs/Makefile tests/Makefile
else
  [ $VERBOSE ] && echo "Tracking Makefiles"
  git update-index --no-assume-unchanged Makefile library/Makefile programs/Makefile tests/Makefile
fi
