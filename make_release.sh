#!/bin/bash

set -e

arg_clean=0
while getopts c opt; do
  case ${opt} in
    c)
      arg_clean=1
      ;;
    *)
      echo "Usage: $0 [-c]"
      exit 1
      ;;
  esac
done

TS=$(date +%Y%m%d)
rels_name="shapones_$TS"
rels_dir=`pwd`/releases/$rels_name
asset_dir=`pwd`/assets/release
rm -rf $rels_dir
mkdir -p $rels_dir

set -eu

pushd samples/picocalc > /dev/null
  dest_dir=$rels_dir/picocalc
  mkdir -p $dest_dir
  for board in pico pico2; do
    echo "Building for board: $board"
    build_dir=build/$board
    if [ $arg_clean -eq 1 ]; then
      rm -rf $build_dir
    fi
    mkdir -p $build_dir
    pushd $build_dir > /dev/null
      cmake -G Ninja -DPICO_BOARD=$board ../.. > cmake.log 2>&1
      ninja > ninja.log 2>&1
      cp shapones_picocalc.uf2 $dest_dir/shapones_picocalc_$board.uf2
    popd > /dev/null
  done
popd > /dev/null

pushd samples/picopad > /dev/null
  echo "Building for PicoPad"
  dest_dir=$rels_dir/picopad
  pp_group=EMU
  pp_target=SHAPONES
  pp_board=picopad20
  pp_build_dir=${PICOLIBSDK_PATH}/PicoPad/$pp_group/$pp_target
  mkdir -p $dest_dir
  if [ $arg_clean -eq 1 ]; then
    rm -rf $pp_build_dir
  fi
  ./build.sh -g "$pp_group" -t "$pp_target" -d "$pp_board" -o "$dest_dir" \
    > build.log 2>&1
popd > /dev/null

pushd samples/shapones_for_xiamocon > /dev/null
  dest_dir=$rels_dir/xiamocon
  mkdir -p $dest_dir

  echo "Building for Xiamocon RP2350"
  if [ $arg_clean -eq 1 ]; then
    rm -rf .cmake
  fi
  ./build_rp2350.sh > build_rp2350.log 2>&1
  cp .cmake/shapones_for_xiamocon.uf2 \
    $dest_dir/shapones_for_xiamocon_rp2350.uf2
  
  echo "Building for Xiamocon ESP32S3"
  if [ $arg_clean -eq 1 ]; then
    rm -rf .pio
  fi
  ./build_esp32s3.sh > build_esp32s3.log 2>&1
  cp .pio/build/esp32s3_arduino/firmware.factory.bin \
    $dest_dir/shapones_for_xiamocon_esp32s3.factory.bin
  cp ${asset_dir}/xiamocon/upload_esp32s3.sh $dest_dir/.
popd > /dev/null

echo "Packaging..."

cp README.md $rels_dir/.
cp LICENSE $rels_dir/.

pushd releases > /dev/null
  rm -f $rels_name.zip
  zip -rq $rels_name.zip $rels_name
  ls -l `pwd`/$rels_name.zip
popd > /dev/null

