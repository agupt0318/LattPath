# LattPath

Legacy C++ prototype for lattice-based path planning and traffic simulation experiments.

## Overview

This repository contains an early path-planning simulation codebase built around autonomous and manual vehicle agents, graph-based environment utilities, and experiments comparing different planning/control modes.

The code appears to have been developed as a local Visual Studio project and currently includes machine-specific include paths. Treat this as an archived research/prototype snapshot rather than a plug-and-play package.

## Notable pieces

- C++ simulation/control-flow scaffolding for autonomous and manual cars
- Graph/environment utilities for path-planning experiments
- Test-group logic for varying autonomous/manual vehicle ratios
- Visual Studio project files from the original development setup

## Status

Archived prototype. To make this production-ready, the next steps would be:

- Replace absolute local include paths with relative includes or a build system such as CMake
- Remove generated IDE/build artifacts from version control
- Add a reproducible build command
- Document input data format and expected outputs
- Add a small runnable example
