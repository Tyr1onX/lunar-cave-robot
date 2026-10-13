#!/bin/zsh
set -e
cd -- "${0:A:h}"
export LUNAR_CONSOLE_OPEN_BROWSER=1
npm start
