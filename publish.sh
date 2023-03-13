#!/usr/bin/env bash

set -e

# Check if jq is installed
if ! [ -x "$(command -v jq)" ]; then
    echo "jq is not installed" >& 2
    exit 1
fi

# Clean previous packages
if [ -d "pkg" ]; then
    rm -rf pkg
fi

if [ -d "pkg-node" ]; then
    rm -rf pkg-node
fi

# Build for both targets
wasm-pack build --scope people_and_robots --target nodejs -d pkg-node -- --features jsbindings
wasm-pack build --scope people_and_robots --target bundler -d pkg -- --features jsbindings

# Get the package name
PKG_NAME=$(jq -r .name pkg/package.json | sed 's/\-/_/g')

# Merge nodejs & browser packages
cp "pkg-node/${PKG_NAME##*/}.js" "pkg/${PKG_NAME##*/}_main.js"
sed "s/require[\(]'\.\/${PKG_NAME##*/}/require\('\.\/${PKG_NAME##*/}_main/" "pkg-node/${PKG_NAME##*/}_bg.js" > "pkg/${PKG_NAME##*/}_bg.js"
jq ".files += [\"${PKG_NAME##*/}_bg.js\"]" pkg/package.json \
    | jq ".main = \"${PKG_NAME##*/}_main.js\"" > pkg/temp.json
mv pkg/temp.json pkg/package.json
rm -rf pkg-node