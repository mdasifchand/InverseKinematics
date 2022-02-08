#!/bin/bash
mkdir -p build
cd build
conan install .. 0.0.1@user/channel
conan source ..
