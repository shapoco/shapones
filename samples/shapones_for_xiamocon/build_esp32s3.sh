#!/bin/bash

set -eux

source $XMC_REPO_PATH/setup.shrc
xmc -p esp32s3_pio_arduino build
