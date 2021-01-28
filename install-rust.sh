#!/bin/bash

curl https://sh.rustup.rs -sSf | sh -s -- --default-toolchain stable -y
export PATH=$PATH:$HOME/.cargo/env
