#!/usr/bin/env bash

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

set -x

if ! $(which catkin_lint 2>&1 &>/dev/null); then
  sudo -H apt-get install python-catkin-lint -y
fi

cd $THIS_DIR/jsk_perception
catkin_lint . \
  --strict -W2 \
  --ignore literal_project_name,description_meaningless,critical_var_append,env_var,target_name_collision,uninstalled_script,link_directory

set +x
