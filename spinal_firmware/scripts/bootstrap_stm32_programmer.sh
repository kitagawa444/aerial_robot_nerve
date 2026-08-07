#!/usr/bin/env bash

set -euo pipefail

readonly PROGRAMMER_VERSION="2.23.0"
readonly BUNDLE_ID="programmer@${PROGRAMMER_VERSION}"
readonly DOWNLOAD_PAGE="https://www.st.com/en/development-tools/stm32cubeprog.html#st-get-software"

usage() {
  cat <<'EOF'
Usage: bootstrap_stm32_programmer.sh [--check-only]

Install the official STM32CubeProgrammer for the current host architecture.
On x86_64, the ST `cube` bundle CLI performs the download, license display,
integrity verification, and installation. On aarch64, the official ARM64
Debian package is validated and installed with apt.

The ARM64 package can be supplied with STM32_PROGRAMMER_DEB. If it is omitted,
the script searches the current directory and the user's Downloads directories.
STM32_PROGRAMMER_DEB_URL can instead contain an ST-issued download URL.

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
programmer="${STM32_PROGRAMMER:-}"

find_programmer() {
  local candidate

  if [[ -n "${programmer}" ]]; then
    return
  fi

  if command -v STM32_Programmer_CLI >/dev/null 2>&1; then
    programmer=$(command -v STM32_Programmer_CLI)
    return
  fi

  if [[ -x "${default_programmer}" ]]; then
    programmer="${default_programmer}"
    return
  fi

  if command -v dpkg-query >/dev/null 2>&1 \
      && dpkg-query -W -f='${Status}' stm32cubeprogrammer 2>/dev/null \
        | grep -q 'install ok installed'; then
    while IFS= read -r candidate; do
      if [[ "${candidate}" == */STM32_Programmer_CLI && -x "${candidate}" ]]; then
        programmer="${candidate}"
        return
      fi
    done < <(dpkg-query -L stm32cubeprogrammer 2>/dev/null)
  fi
}

check_programmer() {
  local version_output
  find_programmer
  [[ -n "${programmer}" ]] || return 1
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
  echo "STM32CubeProgrammer ${PROGRAMMER_VERSION} is not installed." >&2
  exit 1
fi

if [[ -n "${STM32_PROGRAMMER:-}" ]]; then
  echo "Configured STM32_PROGRAMMER is unavailable or is not version ${PROGRAMMER_VERSION}: ${STM32_PROGRAMMER}" >&2
  exit 1
fi

host_arch="${STM32_HOST_ARCH:-$(uname -m)}"
if [[ "${host_arch}" == "aarch64" || "${host_arch}" == "arm64" ]]; then
  package="${STM32_PROGRAMMER_DEB:-}"
  temporary_package=""

  if [[ -z "${package}" && -n "${STM32_PROGRAMMER_DEB_URL:-}" ]]; then
    command -v curl >/dev/null 2>&1 || {
      echo "curl is required to download STM32CubeProgrammer." >&2
      exit 1
    }
    temporary_package=$(mktemp --suffix=_arm64.deb)
    trap 'rm -f "${temporary_package}"' EXIT
    echo "Downloading official STM32CubeProgrammer ARM64 package"
    curl --fail --location --show-error \
      --output "${temporary_package}" "${STM32_PROGRAMMER_DEB_URL}"
    package="${temporary_package}"
  fi

  if [[ -z "${package}" ]]; then
    shopt -s nullglob
    packages=(
      "${PWD}"/stm32cubeprogrammer_*_arm64.deb
      "${HOME}"/Downloads/stm32cubeprogrammer_*_arm64.deb
      "${HOME}"/downloads/stm32cubeprogrammer_*_arm64.deb
      "${HOME}"/ダウンロード/stm32cubeprogrammer_*_arm64.deb
    )
    shopt -u nullglob
    if ((${#packages[@]})); then
      package="${packages[${#packages[@]} - 1]}"
    fi
  fi

  if [[ -z "${package}" || ! -f "${package}" ]]; then
    cat >&2 <<EOF
STM32CubeProgrammer ${PROGRAMMER_VERSION} for Linux ARM64 is distributed by ST
as a Debian package behind its license/export-control download confirmation.
Download stm32cubeprogrammer_*_arm64.deb once from:
  ${DOWNLOAD_PAGE}
Then rerun this target. The package is found automatically in ~/Downloads, or
set STM32_PROGRAMMER_DEB=/absolute/path/to/package.deb.
EOF
    exit 1
  fi

  package=$(realpath "${package}")
  package_arch=$(dpkg-deb --field "${package}" Architecture 2>/dev/null || true)
  package_version=$(dpkg-deb --field "${package}" Version 2>/dev/null || true)
  if [[ "${package_arch}" != "arm64" ]]; then
    echo "Not an ARM64 Debian package: ${package} (Architecture=${package_arch:-unknown})" >&2
    exit 1
  fi
  if [[ "${package_version}" != "${PROGRAMMER_VERSION}"* ]]; then
    echo "Expected STM32CubeProgrammer ${PROGRAMMER_VERSION}, got ${package_version:-unknown}: ${package}" >&2
    exit 1
  fi

  echo "Installing official STM32CubeProgrammer ${package_version} ARM64 package"
  sudo apt-get install -y "${package}"
  programmer=""
  if ! check_programmer; then
    echo "ARM64 package installation completed, but STM32_Programmer_CLI ${PROGRAMMER_VERSION} is unavailable." >&2
    exit 1
  fi
  echo "STM32CubeProgrammer ${PROGRAMMER_VERSION} installed: ${programmer}"
  exit 0
fi

if [[ "${host_arch}" != "x86_64" && "${host_arch}" != "amd64" ]]; then
  echo "Unsupported STM32CubeProgrammer host architecture: ${host_arch}" >&2
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
