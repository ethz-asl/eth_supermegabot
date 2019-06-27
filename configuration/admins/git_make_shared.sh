#!/bin/bash
# Author(s): Christian Gehring

SCRIPT_FOLDER=$(dirname $0)

# Function to reset the terminal colors
function color_reset
{
  echo -ne "\033[0m"
}


# Function that displays the branch of a repository
function make_shared
{

  sudo chgrp -R $2 $1                  # set the group
  sudo chmod -R g+rw $1                # allow the group to read/write
  sudo chmod g+s `find $1 -type d`     # new files get group id of directory
  if [ -d $1.git ]; then               # if folder is git repo fix config
        git config -f $1/.git/config core.sharedRepository group
  fi
  NAME=$(basename $1)
  printf "${COLOR_INFO}%-40s\n" ${NAME}
	 
  #echo -e "${COLOR_INFO}  Repo ${COLOR_ITEM}${NAME}${COLOR_INFO}: ${COLOR_RESET}${BRANCH}"

}


# List of usefull colors
COLOR_RESET="\033[0m"
COLOR_INFO="\033[0;32m"
COLOR_ITEM="\033[1;34m"
COLOR_QUES="\033[1;32m"
COLOR_WARN="\033[0;33m"
COLOR_CODE="\033[0m"
COLOR_BOLD="\033[1m"
COLOR_UNDE="\033[4m"
COLOR_TEST="\E[36m"
COLOR_RED="\e[31m"
COLOR_GREEN="\033[0;32m"


# Get target folder
if [ $# -eq 1  -o  $# -eq 2 ]; then
  TARGET=$1
else
  TARGET="."
fi

if [ $# -eq 2 ]; then
  SHAREDGROUP=$2
else
  SHAREDGROUP="developers"
fi

# Read the list of folders
echo ""
echo -e "${COLOR_INFO}List of repositories:${COLOR_RESET}"
echo ""
for FOLDER in ${TARGET}/*/; do
  make_shared "$FOLDER" "$SHAREDGROUP"
done
echo ""
color_reset

