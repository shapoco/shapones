#!/bin/bash

set -eux

source $XMC_REPO_PATH/setup.shrc
xmc -p rp2350_pico_sdk build
