#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
bash "$SCRIPT_DIR/../run_docker.sh" -i erik_elevationmapping_testing -c erik_elevationmapping_testing-container