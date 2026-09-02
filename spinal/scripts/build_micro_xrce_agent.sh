#!/usr/bin/env bash

set -euo pipefail

agent_version="2.4.0"
cache_root="${XDG_CACHE_HOME:-${HOME}/.cache}/aerial_robot"
data_root="${XDG_DATA_HOME:-${HOME}/.local/share}/aerial_robot"
source_dir="${cache_root}/Micro-XRCE-DDS-Agent-${agent_version}"
build_dir="${cache_root}/Micro-XRCE-DDS-Agent-${agent_version}-build"
install_dir="${1:-${data_root}/micro_xrce_agent}"

if [[ ! -d "${source_dir}/.git" ]]; then
  mkdir -p "${cache_root}"
  git clone --branch "v${agent_version}" --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${source_dir}"
fi

cmake -S "${source_dir}" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${install_dir}" \
  -DUAGENT_SUPERBUILD=ON
cmake --build "${build_dir}" --parallel
cmake --install "${build_dir}"

printf 'MicroXRCEAgent installed in %s\n' "${install_dir}"
printf 'Before launch, run: export PATH="%s/bin:$PATH"\n' "${install_dir}"
printf 'Before launch, run: export LD_LIBRARY_PATH="%s/lib:${LD_LIBRARY_PATH:-}"\n' "${install_dir}"
