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

<img width="572" height="595" alt="image" src="https://github.com/user-attachments/assets/ebe2a947-dc8c-45a3-a9d7-7294da859b66" />

<img width="755" height="607" alt="image" src="https://github.com/user-attachments/assets/d781214f-a854-45d3-a68f-8650a9dfc465" />

<img width="760" height="607" alt="image" src="https://github.com/user-attachments/assets/ff992fd0-544d-4134-ac0c-25a65bf62646" />

<img width="767" height="607" alt="image" src="https://github.com/user-attachments/assets/e6555178-d10a-4ca5-a5d6-38aa0e8b4081" />

