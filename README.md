# SofaDiff

Plugin for differentiable physics in SOFA.

## Installation

Cf installation of SOFA plugins.

Note: the examples may rely on a change in `SpringForceField` that has not been pushed on the master branch of SOFA yet (TODO).

## Features

Supports:
* Differentiation of static and quasi-static simulations
* Propagation of the gradient back to parameters in the force fields
    * Requires "augmenting" the force field, cf the `ParameterizedSpringForceField` example
    * Or implementation from scratch in Python (cf below)
* Gradient-based optimization of said parameters
* Implementation of components in Python:
    * Optimization algorithm (e.g. gradient descent based on JAX)
    * Parameterized force fields (e.g. force field leveraging JAX autodiff)

Will support:
* Differentiation of dynamic simulations
* Propagation of the gradient back to parameters in projective constraints
* Differentiation through Lagrange constraints & contacts
