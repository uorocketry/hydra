#!/bin/bash

cmake -B build -G Ninja
cmake --build build --target "${1}"