#! /bin/bash

# Copies the install directory from the built workspace (in a bind-mounted directory $BASEDIR/$PREFIX<upstream_ws|target_ws>)
# to a permanent target directory (/opt/$PREFIX<upstream_ws|target_ws>)
function copy_install_dir() {
  local target_dir=/opt/${PREFIX}${1}
  mkdir -p $target_dir
  cp -r ${BASEDIR}/${PREFIX}${1}/install $target_dir
}

copy_install_dir upstream_ws
copy_install_dir target_ws
