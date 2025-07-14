# 🚗 Motion Control of Wheeled Mobile Robot

This project presents a complete pipeline for motion control of a car-like wheeled mobile robot using the Lyapunov Direct Method. It includes **controller design**, **path planning**, and **optimization**, all implemented and tested in MATLAB.

## 📋 Overview

The goal is to design a globally asymptotically stable path-following controller based on Lyapunov theory. The robot follows a polynomial-based obstacle-free path planned via a hybrid of A* and polynomial techniques.

## ✨ Features

- ✅ Lyapunov-based controller with formal stability proof
- 🎯 Path planning using A* and polynomial trajectory smoothing
- 🧠 Optimization of controller gains using RMSE as performance criterion
- 📊 Simulation outputs including error plots, velocity profiles, and steering behavior
- 📁 Well-structured MATLAB implementation with modular components

## 🧠 Control Design

The controller uses:
- Error state formulation based on reference trajectory
- Lyapunov stability analysis using La Salle's Principle
- Gain tuning to minimize trajectory tracking error

Stability is ensured through rigorous proof and simulation validation.

## 🗺️ Path Planning

- **A\*** algorithm for initial obstacle-avoiding path
- **Polynomial fitting** for steering-conforming, smooth trajectory
- Comparison with Dijkstra algorithm for performance benchmarking

## 🛠️ Optimization

The control gains \(K_x, K_y, K_\theta\) are tuned via minimization of RMSE between the robot's actual trajectory and the desired path.

## 📈 Simulation Results

Key outcomes include:
- Near-zero error convergence
- Smooth trajectory tracking despite nonholonomic constraints
- Limitations in lateral (y-axis) tracking noted for further improvement

