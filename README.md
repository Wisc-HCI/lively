<h1 align="center">
  <a href="https://wisc-hci.github.io/lively/"><img src="https://user-images.githubusercontent.com/5341396/218564862-aeac4437-873f-48cb-b503-15375250f75b.png" alt="Lively"></a>
</h1>

<p align="center">
   <a href="https://badge.fury.io/py/lively_tk"><img src="https://badge.fury.io/py/lively_tk.svg" alt="PyPI version" height="18"></a>
   <a href="https://badge.fury.io/js/@people_and_robots%2Flively"><img src="https://badge.fury.io/js/@people_and_robots%2Flively.svg" alt="npm version" height="18"></a>
    <br>
    <a href= "https://wisc-hci.github.io/lively/"><img alt="docs: ready" src="https://img.shields.io/badge/docs-ready-success.svg?logoHeight=10"></a>
</p>

# Lively v1.1.0

You can find the documentation for lively [here](https://wisc-hci.github.io/lively/)

# LivelyStudio

For an interface to configure Lively, see [LivelyStudio](https://github.com/Wisc-HCI/LivelyStudio).

## About

Lively Package

The Lively framework provides a highly configurable toolkit for commanding robots in mixed modalities while incorporating liveliness motions. It is adapted from [RelaxedIK](https://github.com/uwgraphics/relaxed_ik_core) framework, and compatible with Python and Javascript/Node.

To configure a robot, the easiest method is to use the LivelyStudio interface in the [LivelyStudio](https://github.com/Wisc-HCI/LivelyStudio) repository, which is a system for configuring and programming the robot visually.

## Documentation

Documentation is provided on our [github pages site](https://wisc-hci.github.io/lively/). It provides an API overview and online tutorials.

## Getting Help

Please feel free to post in our [Github Discussions](https://github.com/Wisc-HCI/lively/discussions), or if you found an issue, report it [here](https://github.com/Wisc-HCI/lively/issues). 


## Contributing


**Python Instructions**

To build, download and `cd` to this directory. Then run:

```bash

# Install Maturin
pip3 install maturin

# If you just want to install locally or develop:
maturin develop
```

## Deploy to GitHub Page

To deploy, run `yarn deploy` from the `main` branch in the `docs` directory. We will need to redeploy every time we make new changes to the documentataion.

**Javascript Instructions**

To build, download and `cd` to this directory. Then run:

```bash
# Build the javascript bundle
wasm-pack build --scope people_and_robots --target bundler -- --features jsbindings

# Pack
wasm-pack pack

# Publish
wasm-pack publish --access=public
```


## References
[^1]:Rakita, Daniel, Bilge Mutlu, and Michael Gleicher. "PROXIMA: An Approach for Time or Accuracy Budgeted Collision Proximity Queries." Proceedings of Robotics: Science and Systems (RSS). 2022. http://www.roboticsproceedings.org/rss18/p043.pdf


