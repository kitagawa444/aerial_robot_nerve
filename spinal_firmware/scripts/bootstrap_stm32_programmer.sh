#!/usr/bin/env bash

set -euo pipefail

readonly PROGRAMMER_VERSION="2.23.0"
readonly BUNDLE_ID="programmer@${PROGRAMMER_VERSION}"

usage() {
  cat <<'EOF'
Usage: bootstrap_stm32_programmer.sh [--check-only]

Install the official STM32CubeProgrammer bundle into the current user's
STM32Cube bundle directory. The ST `cube` bundle CLI is provided by the
official STM32 VS Code extension pack and performs the download, license
display, integrity verification, and installation.

Options:
  --check-only  Verify STM32CubeProgrammer without changing the machine.
  -h, --help    Show this help.
EOF
}

check_only=false
while (($#)); do
  case "$1" in
    --check-only)
      check_only=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

data_root="${XDG_DATA_HOME:-${HOME}/.local/share}"
default_programmer="${data_root}/stm32cube/bundles/programmer/${PROGRAMMER_VERSION}/bin/STM32_Programmer_CLI"
programmer="${STM32_PROGRAMMER:-${default_programmer}}"

check_programmer() {
  local version_output
  [[ -x "${programmer}" ]] || return 1
  version_output=$("${programmer}" --version 2>&1) || return 1
  [[ "${version_output}" == *"STM32CubeProgrammer"* \
    && "${version_output}" == *"${PROGRAMMER_VERSION}"* ]]
}

if check_programmer; then
  echo "STM32CubeProgrammer ${PROGRAMMER_VERSION} is available: ${programmer}"
  exit 0
fi

if [[ "${check_only}" == true ]]; then
  echo "STM32CubeProgrammer ${PROGRAMMER_VERSION} is not installed at ${programmer}" >&2
  exit 1
fi

if [[ -n "${STM32_PROGRAMMER:-}" ]]; then
  echo "Configured STM32_PROGRAMMER is unavailable or is not version ${PROGRAMMER_VERSION}: ${STM32_PROGRAMMER}" >&2
  exit 1
fi

cube_cli="${STM32_CUBE_CLI:-}"
if [[ -z "${cube_cli}" ]] && command -v cube >/dev/null 2>&1; then
  cube_cli=$(command -v cube)
fi

if [[ -z "${cube_cli}" ]]; then
  shopt -s nullglob
  candidates=(
    "${HOME}"/.vscode/extensions/stmicroelectronics.stm32cube-ide-core-*-linux-x64/resources/binaries/linux/x86_64/cube
  )
  shopt -u nullglob
  if ((${#candidates[@]})); then
    cube_cli="${candidates[${#candidates[@]} - 1]}"
  fi
fi

if [[ -z "${cube_cli}" || ! -x "${cube_cli}" ]]; then
  cat >&2 <<'EOF'
The official ST `cube` bundle CLI was not found.
Install the "STM32 VS Code Extension Pack" once, put `cube` on PATH, or set
STM32_CUBE_CLI=/absolute/path/to/cube. In a disposable Docker container, mount
an already installed programmer bundle read-only instead.
EOF
  exit 1
fi

echo "Installing official ST bundle ${BUNDLE_ID} with ${cube_cli}"
"${cube_cli}" bundle install --yes "${BUNDLE_ID}"

if ! check_programmer; then
  echo "Bundle installation completed but ${programmer} is unavailable." >&2
  exit 1
fi

echo "STM32CubeProgrammer ${PROGRAMMER_VERSION} installed: ${programmer}"
