# Experimental Reverb (and Delay) Building Blocks for the Daisy Seed

This repository contains a collection of **modular, experimental reverb and spatial audio building blocks**, intended for use on the [**Electrosmith Daisy Seed**](https://www.electro-smith.com/daisy/daisy). The focus is on **flexibility**, **exploration**, and **rapid iteration**, rather than polished production-ready code.

## ⚠️ Work in Progress Disclaimer

This is an experimental project and a work in progress.

It was developed primarily for research and learning on the Daisy Seed, and while it has been tested on real hardware, no guarantees are made regarding the accuracy, stability, or sonic quality of its results.

This code should be viewed as a foundation for exploration — not a finished product. You are encouraged to modify, measure, and validate it for your own purposes, and to share improvements or findings with the community.

---

## Overview

The modules here are designed to be composable. Typical topologies include:

- **Early Reflection Generators**
- **Diffusion Networks (Schroeder-style, nested, modulated)**
- **FDN (Feedback Delay Network) Tanks**  
  - Householder mixing
  - Dynamic damping and modulation per line
  - Variable line counts
- **Spectral Shaping Filters**
- **Freeze / Infinite Sustain Behaviors**
- **Cloud / Tail Enhancement Layers**

The goal is to make it easy to try:

- Different diffuser patterns
- Tank structures (4×4, 8×8, etc.)
- Hybrid reverbs (plate/room/hall abstractions)
- Nonlinear or spectral reverb enhancements

---

## Design Philosophy

- **Everything is a Building Block:**  
  Components are intentionally not tightly coupled, so they may be rearranged, replaced, or bypassed.

- **Simple First, Fancy Later:**  
  Many modules start out minimal in parameterization until stabilized.

- **Real-Time Safe Where Possible:**  
  Allocations are typically expected to be done at initialization, not during audio processing.

---

## Attribution & Credits

None of this code is uniquely mine — it draws heavily on the shared knowledge of the **Daisy**, **Teensy**, **ARM**, and other DSP communities. Most components in this repository build upon or were inspired by prior open-source work:

- **External Buffer Delay Line Concept**
  
  - Based on design approaches from **bkshepherd**  
    GitHub: https://github.com/bkshepherd  
    This repository includes modified forms of the external-buffer pattern for SDRAM-based delay lines.  
    All credit to the original author for the underlying approach; any errors introduced here are my own.

- General conceptual inspiration from:
  
  - **Zita-Rev** (Fons Adriaensen)
  - **FDN literature** (Jot, Stautner & Puckette, etc.)
  - **Large-hall plate aesthetics** of classic and modern reverb processors

Special thanks to the countless developers, forum members, and DSP enthusiasts whose posts, code snippets, and shared experiments continue to inspire.

---

## License

Unless otherwise noted in specific source files, this repository is provided under the **MIT License**.

See `LICENSE` file for full text.
