#!/bin/bash

set -e

ARG_DEVICE="picopad20"
ARG_OUTDIR="bin"
ARG_GRPDIR=""
ARG_TARGET=""
while getopts d:o:g:t: opt; do
  case ${opt} in
    d)
      ARG_DEVICE="${OPTARG}"
      ;;
    o)
      ARG_OUTDIR="${OPTARG}"
      ;;
    g)
      ARG_GRPDIR="${OPTARG}"
      ;;
    t)
      ARG_TARGET="${OPTARG}"
      ;;
    *)
      echo "Usage: $0 [-d device]"
      exit 1
      ;;
  esac
done

if [ -z "${ARG_GRPDIR}" ]; then
  echo "*Error: -g GRPDIR is required."
  exit 1
fi

if [ -z "${ARG_TARGET}" ]; then
  echo "*Error: -t TARGET is required."
  exit 1
fi

set -u

APP_DIR=$(pwd)
REPO_DIR=$(cd ../.. && pwd)

FONT_DIR="${REPO_DIR}/samples/font/mono8x16"
SRC_TARGET_DIR="${APP_DIR}/${ARG_GRPDIR}/${ARG_TARGET}"
CORE_DIR="${REPO_DIR}/core"

DEST_BIN_DIR="${APP_DIR}/bin/${ARG_DEVICE}"

pushd "${PICOLIBSDK_PATH}/_tools"
  pushd elf2uf2
    g++ -o elf2uf2 *.cpp
  popd
  pushd PicoPadLoaderCrc
    g++ -o LoaderCrc *.cpp
  popd
popd

pushd "${PICOLIBSDK_PATH}/PicoPad"
  mkdir -p "./${ARG_GRPDIR}/${ARG_TARGET}/src"
  pushd "./${ARG_GRPDIR}/${ARG_TARGET}"
    cp -rp ${CORE_DIR}/include/* .
    cp -rp ${CORE_DIR}/src/* src/.
    cp -rp ${FONT_DIR}/* src/.
    cp -rp ${SRC_TARGET_DIR}/* .
    touch src/*.cpp
    ./c.sh "${ARG_DEVICE}"
    mkdir -p "${ARG_OUTDIR}"
    cp *.hex *.bin *.lst *.sym *.siz *.uf2 "${ARG_OUTDIR}/."
  popd
popd
