#!/usr/bin/env bash

# Initialize flags and inputs.
didDefineGitFolder=0
git_folder=""

# A method which displays information on how to use this script.
print_help() {
 echo "This script will clone all the necessary repositories to run SMB in a simulated environment."
 echo "By default, repositories will be cloned in the '~/git/' folder. If it doesn't exist, it will be created."
 echo "To clone in a different folder, use the -f option. For example, to clone in '~/my_git' type:"
 echo -e "\t ./clone_repos.sh -f ~/my_git"
}

# Check for options.
while getopts "f:h" opt; do
  case $opt in
	f)
	  didDefineGitFolder=1
	  git_folder=$OPTARG >&2
	echo "User specifiec folder $git_folder"
	  ;;
	h)
	 print_help
	 exit 1
	  ;;
	\?)
	  echo "Invalid option: -$OPTARG" >&2
	 exit 1
	  ;;
	:)
	  echo "Option -$OPTARG requires an argument." >&2
	  exit 1
	  ;;
  esac
done

# If no option was provided for the git folder, use a default one.
if [ $didDefineGitFolder -ne 1 ]; then
  echo "Git folder was not provided. Defaulting to '~/git/'"
 git_folder="$HOME/git/"
fi

# Set the list of repositories to clone.
declare -a REPOSITORIES=(
  "anymal"
  "alma_controller"
  "any_common"
  "any_gazebo"
  "any_joy"
  "any_node"
  "any_utils"
  "any_utils_ros"
  "collisions"
  "cosmo"
  "curves"
  "kindr"
  "kindr_ros"
  "ooqp_eigen_interface"
  "hector_gazebo"
  "kinova_common"
  "kinova_highlevel_controller"
  "loco"
  "locomo"
  "message_logger"
  "numopt"
  "numopt_eth"
  "ooqp_catkin"
  "parameter_handler"
  "quadruped_common"
  "robot_measurements"
  "robot_utils"
  "roco"
  "rocoma"
  "romo"
  "signal_logger"
  "smb_common"
  "user_interface"
  "smb_common"
  "smb_controller"
  "smb_highlevel_controller"
  "whole_body_control"
)

# Create a git folder if necessary and move to it.
mkdir -p $git_folder
cd $git_folder

for REPOSITORY in "${REPOSITORIES[@]}"; do
  echo -e "Checking repository \033[1;34m${REPOSITORY}\033[0m"
  if [ -d "${REPOSITORY}" ]; then
	echo -e "  The folder exists. Skiping repository."
  else
	echo -e "  The folder doesn't exist. Cloning repository."
	git clone "git@bitbucket.org:leggedrobotics/${REPOSITORY}.git"
	git clone "git@github.com:leggedrobotics/${REPOSITORY}.git"
	git clone "git@github.com:ethz-asl/${REPOSITORY}.git"
  fi
done

# Clone rbl with Mercurial
REPOSITORY=rbdl
echo -e "Checking repository \033[1;34m${REPOSITORY}\033[0m"
if [ -d "${REPOSITORY}" ]; then
   echo -e "  The folder exists. Skiping repository."
else
   echo -e "  The folder doesn't exist. Cloning repository."
   hg clone ssh://hg@bitbucket.org/leggedrobotics/${REPOSITORY} -r master
fi
