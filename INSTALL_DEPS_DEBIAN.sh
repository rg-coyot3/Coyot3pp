#!/bin/bash

v_this_dir=$(cd $(dirname ${BASH_SOURCE[0]}) && pwd)

echo " "
echo " coyot3pp installer"
echo " "




echo " - searching for modules installers and ensuring they are executables."
find ${v_this_dir} -name "install_deps_debian.sh" | xargs chmod +x
v_installers_list=(find ${v_this_dir} -name "install_deps_debian.sh")
echo " "
echo " - now launching them."
for v_installer in ${v_installers_list};do
  echo " - for component at [$(dirname ${v_installer})] : "
  ${v_installer}
done


echo " "
echo " - done"
echo " "