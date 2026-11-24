# Pendulum-Cart Control: from Lagrangian modeling to sliding mode control

![MATLAB](https://img.shields.io/badge/MATLAB-R2026-orange) ![License: MIT](https://img.shields.io/badge/License-MIT-green)

Modeling, analysis and control of a single-wheel inverted pendulum in MATLAB/Simulink,
validated on the full nonlinear model.

![demo](media/demo.gif)

## The problem
<img src="docs/images/system.png" width="460" align="right">
One or two sentences: what moves, what is measured, what must be stabilized. Parameter table.

## What's inside
Table: step | method | result (modeling, linearization, analysis, PID, pole placement, LQR, observer, integral action, SMC).

## Key results
4 to 6 bullets with numbers: unstable pole +3.60, zero at s = 0 and its consequences,
observability from q1 only, model validation (linear vs nonlinear error), controller performance.

## How to run
openProject("pendulum-cart-control.prj"); main; runtests("tests")

## Full report
[docs/report.md](docs/report.md)

## What I learned
3 or 4 bullets.

## Context
Project carried out in the <course> course at <school>. License: MIT.