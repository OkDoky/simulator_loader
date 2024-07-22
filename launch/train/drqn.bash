#!/bin/bash

# init pyenv
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init --path)"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"

cd /root/stable-baselines3-contrib-drqn/sb3_contrib_drqn
pyenv activate train

python nav_train.py