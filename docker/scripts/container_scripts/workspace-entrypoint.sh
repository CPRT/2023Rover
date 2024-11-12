#!/bin/bash

# -------------------------------------------------
# Persistent bash history
# -------------------------------------------------
if [[ ! -L ~/.bash_history ]]; then
  rm -f ~/.bash_history
  ln -s "$BASH_HISTORY_FILE" ~/.bash_history
fi

# Restart udev daemon
sudo service udev restart

$@
