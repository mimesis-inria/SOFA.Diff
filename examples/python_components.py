"""
SOFA.Diff components in Python.

Shows how to implement an OptimizationLoop, a GradientBasedOptimizationLoop and a ParameterizedForceField in Python.
The application consists of two independent particles (red) connected to the origin with zero-length springs, whose
stiffness is optimized to reach the two target points (blue). Both optimizers can be used in combination after opening
the "Loops Controls" panel: the RandomOptimizationLoop can be used to find a good initial guess for the
PythonGradientDescent, that fine-tunes the result.
"""
# =====================================================================================================================
# Preamble
# =====================================================================================================================
import Sofa
import SofaRuntime
import SofaImGui
import Sofa.Gui
import sofadiff

import numpy as np


# =====================================================================================================================
# Custom optimizers in Python
# =====================================================================================================================
class RandomOptimizer(sofadiff.OptimizationLoop):
    """
    A Python optimizer that picks parameters randomly between their lower and upper bounds.

    sofadiff.OptimizationLoop has 3 methods that can be overridden:
        * allocate():          optional, perform operations that must happen at initialization only
        * initialize():        optional, perform operations that must happen at initialization and at reset
        * update_parameters(): required, compute the next value to be used for each optimized parameter

    sofadiff.OptimizationLoop provides the attribute `self.parameters`, a list of parameters to be optimized.
    Each parameter in this list comes with the following interface:
        * parameter.value:      read-only, the current value of the parameter, as a flat array
        * parameter.size:       read-only, the size of parameter.value
        * parameter.next_value: read-write, the next value to be used for the parameter, as a flat array
        * parameter["key"]:     read-only, the "key" hyperparameter for this parameter, e.g. parameter["lowerBound"]
                                any "key" can be used, provided the "key" hyperparameter is defined for this parameter
        * "key" in parameter:   whether the "key" hyperparameter was defined for this parameter
    Note that the parameters are not yet initialized in the __init__() method.

    sofadiff.OptimizationLoop also provides the attribute `self.start_with_update`, a boolean indicating whether the
    `update_parameters()` method should be called at the start of the optimization. If not, then the current value of
    the parameters will be used for the first iteration. This attribute is `False` by default, but can be changed at
    any point.

    sofadiff.OptimizationLoop comes with a built-in mechanism that stores the best set of parameters found so far, so
    there is no need to implement that here.
    """
    def __init__(self, *args, **kwargs):
        sofadiff.OptimizationLoop.__init__(self, *args, **kwargs)  # Do not use super()
        self.start_with_update = True  # Pick random values from the very beginning
        # Do not use `self.parameters` here, the parameters are not initialized yet.

    def allocate(self):
        """Perform operations that must happen at initialization only."""
        print("The following parameters are going to be optimized:")
        for parameter in self.parameters:  # Now the parameters are initialized
            print(parameter)  # Parameters can also be printed

    def initialize(self):
        """Perform operations that must happen at initialization and at reset."""
        pass

    def update_parameters(self):
        """Compute the next value to be used for each optimized parameter."""
        for parameter in self.parameters:
            random_value = np.random.uniform(parameter["lowerBound"], parameter["upperBound"], size=parameter.size)
            parameter.next_value = random_value


class PythonGradientDescent(sofadiff.GradientBasedOptimizationLoop):
    """
    A simple gradient descent implemented in Python.

    sofadiff.GradientBasedOptimizationLoop inherits sofadiff.OptimizationLoop, and shares the same interface.
    In addition to what sofadiff.OptimizationLoop does, it makes sure that the simulation is differentiable, and
    triggers the computation of the gradients once the loss is computed.

    The derivative of the loss with respect to each parameter can be accessed with `parameter.gradient` for each
    parameter in `self.parameters`.
    """
    def update_parameters(self):
        # The list of parameters to be optimized can be accessed in self.parameters
        for parameter in self.parameters:
            # The parameter's value and gradient can be accessed like attributes
            # Any hyperparameter can be added to a Parameter and accessed with parameter["key"]
            next_value = parameter.value - parameter["learningRate"] * parameter.gradient
            # For optional hyperparameters one can check if they were provided or not
            if "lowerBound" in parameter:
                next_value = np.maximum(next_value, parameter["lowerBound"])
            if "upperBound" in parameter:
                next_value = np.minimum(next_value, parameter["upperBound"])
            # The task of this method is to set the next value of each parameter
            parameter.next_value = next_value
            # This assignment involves a copy, so it's preferable to do it only once


# =====================================================================================================================
# Custom parameterized force field in Python
# =====================================================================================================================
class PythonForceField(sofadiff.ParameterizedForceFieldVec3d):
    """
    A Python force field that adds zero-length springs between the vertices and the origin.

    sofadiff.ParameterizedForceFieldXXX inherits Sofa.Core.ForceFieldXXX and the usual interface: init(), addForce(),
    addDForce(), addKToMatrix(), ...

    On top of that, sofadiff.ParameterizedForceFieldXXX adds the possibility for the force field to be differentiated
    with respect to its parameters. Doing so involves two steps:
        1. Retrieving the Parameters passed by the user to the force field, if any.
           This is done in the `init()` method with `self.get_parameter(data_name, default=None)`.
        2. Implementing the differentiation of the force with respect to any potential parameter.
           This is done in the `propagate_gradient_to_parameters(self, force_gradient)` method.

    When a parameter is retrieved with `self.get_parameter()`, its gradient can be accessed via `parameter.gradient`.
    This gradient should be *incremented* with the force field's contribution, since the same parameter may be used by
    different components.

    Note that each potential parameter must first be added as a Data to the force field, and only then can the user
    decide to provide its value through a Parameter. The user can still provide the Data in other ways, in which case
    `self.get_parameter()` returns None.
    """
    def __init__(self, stiffness, *args, **kwargs):
        sofadiff.ParameterizedForceFieldVec3d.__init__(self, *args, **kwargs)  # Do not use super()
        # addData() must be called in init() (not __init__()) to support initialization with data="@/path/to.value",
        # therefore we need to store the argument here to access it later in `init()`.
        self.stiffness_arg = stiffness
        # Declare attribute, also defined in init()
        self.stiffness_parameter = None

    def init(self):
        # Add a stiffness Data – this creates a `self.stiffness` attribute
        self.addData("stiffness", self.stiffness_arg)
        # Get the Parameter used to supply the stiffness, or None if the stiffness was initialized differently
        self.stiffness_parameter = self.get_parameter("stiffness", None)

    def addForce(self, mechanical_parameters, force, position, velocity):
        """force[i, j] = - stiffness[i] * position[i, j]"""
        with force.writeableArray() as wa:
            wa[:] -= self.stiffness.value[:, None] * position.value

    def addDForce(self, mechanical_parameters, df, dx):
        """df[i, j] = - stiffness[i] * dx[i, j]"""
        with df.writeableArray() as wa:
            wa[:] -= self.stiffness.value[:, None] * dx * mechanical_parameters['kFactor']

    def addKToMatrix(self, mechanical_parameters, n_particles, n_dimensions):
        """K = -diag(k0, k0, k0, k1, k1, k1, ...) where ki means stiffness[i]"""
        n_dofs = n_particles * n_dimensions
        return np.array([[i, i, -self.stiffness.value[i//3]] for i in range(n_dofs)]) * mechanical_parameters["kFactor"]

    def propagate_gradient_to_parameters(self, force_gradient):
        """
        Add the contribution of this component to the derivative of the loss with respect to each parameter.

        This contribution consists in d(loss)/d(param) += d(loss)/d(force) @ d(force)/d(param), where
            * d(loss)/d(param) is accessed via `param.gradient`
            * d(loss)/d(force) is computed by the adjoint solver and passed as `force_gradient`
            * the product with d(force)/d(param) must be implemented here

        Here we have
            force[i, j] = -stiffness[i] * position[i, j],
        therefore
            d(force[i, j])/d(stiffness[i]) = -position[i, j], and
            stiffness_gradient[i] = sum_j (-position[i, j] * force_gradient[i, j])
        """
        if self.stiffness_parameter is not None:
            self.stiffness_parameter.gradient += np.sum(-self.mstate.position.value * force_gradient.value, axis=1)


# =====================================================================================================================
# Tools to make writing the scene more convenient
# =====================================================================================================================
_current_node: Sofa.Core.Node

def get_current_node():
    return _current_node

def set_current_node(node):
    global _current_node
    _current_node = node

def add_object(*args, **kwargs):
    return get_current_node().addObject(*args, **kwargs)

def add_child(*args, **kwargs):
    return get_current_node().addChild(*args, **kwargs)

class Node:
    def __init__(self, name, **kwargs):
        self.name = name
        self.kwargs = kwargs
        self.parent = None
        self.child = None

    def __enter__(self):
        self.parent = get_current_node()
        self.child = add_child(self.name, **self.kwargs)
        set_current_node(self.child)
        return self.child

    def __exit__(self, exc_type, exc_value, traceback):
        set_current_node(self.parent)

def set_data(**kwargs):
    node = get_current_node()
    for data, value in kwargs.items():
        setattr(node, data, value)

# =====================================================================================================================
# The scene
# =====================================================================================================================
def createScene(root):
    set_current_node(root)
    set_data(dt=0.05, gravity=(0, -10, 0))

    with Node("Plugins"):
        add_object("RequiredPlugin", pluginName=[
            "SOFA.Diff",
            "Sofa.Component.Visual",  # Needed to use components [VisualStyle]
            "Sofa.Component.LinearSolver.Iterative",  # Needed to use components [CGLinearSolver]
            "Sofa.Component.LinearSolver.Direct",  # Needed to use components [SparseLDLSolver]
            "Sofa.Component.ODESolver.Backward",  # Needed to use components [NewtonRaphsonSolver, StaticSolver]
            "Sofa.Component.StateContainer",  # Needed to use components [MechanicalObject]
            "Sofa.Component.Mass",  # Needed to use components [UniformMass]
            "Sofa.Component.Mapping.NonLinear",  # Needed to use components [DistanceFromTargetMapping]
        ])

    add_object("VisualStyle", displayFlags="showBehavior showBehaviorModels showForceFields showMappings")

    # ======================================== The two Python optimizers ==============================================
    # The RandomOptimizer can be used to find a better initial guess for the PythonGradientDescent optimizer.
    add_object(RandomOptimizer(name="RandomOptimizer", parameters="@/Parameters/stiffness"))
    add_object(PythonGradientDescent(name="GradientDescent", parameters="@/Parameters/stiffness"))
    # =================================================================================================================

    add_object("DifferentiableAnimationLoop", name="differentiable-simulator", computeBoundingBox=False)

    add_object("MechanicalObject", template="Vec3d", name="origin", position="0 0 0", showObject="true", drawMode="1", showObjectScale="0.05")

    with Node("Parameters"):
        add_object("TrainableParameterVector", name="stiffness", value="6 6", learningRate=0.5, lowerBound=1, upperBound=10)

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixd", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton="20", maxNbIterationsLineSearch="1", warnWhenLineSearchFails="false")
        add_object("StaticSolver", name="static", newtonSolver="@newton")
        add_object("StaticAdjointSolver", name="adjoint")

        add_object("MechanicalObject", template="Vec3d", name="state", position="0 -7 0  0 -7 0", showObject="true", drawMode="1", showObjectScale="0.1", showColor=(1, 0, 0, 1))
        add_object("UniformMass", template="Vec3d", name="mass", totalMass=10)

        # ================================ Zero-length springs with PythonForceField ==================================
        add_object(PythonForceField(name="Springs", stiffness="@/Parameters/stiffness.value"))
        # =============================================================================================================

    with Node("Loss"):
        add_object("MechanicalObject", template="Vec3d", name="state", position="0 -6 0  0 -8 0", showObject="true", drawMode="1", showObjectScale="0.08", showColor=(0, 0, 1, 1))
        with Node("Distance"):
            add_object("MechanicalObject", template="Vec1d", name="state", position="0")
            add_object("DistanceFromTargetMapping", input="@Physics/state", targetPositions="@/Loss/state.position")
            with Node("MSE"):
                add_object("LossState", name="state")
                add_object("MeanSquaredErrorMapping")
