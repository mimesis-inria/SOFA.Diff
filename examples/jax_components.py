"""
SOFA.Diff components in JAX.

Shows how to implement a ParameterizedForceField and a GradientDescentOptimizationLoop in JAX, leveraging
JIT-compilation and automatic differentiation. The application consists of two independent particles (red) connected to
the origin with springs, whose stiffness is equal to their length. These parameters are optimized to reach the two
target points (blue). The optimizer can be used with the usual interface or after opening the "Loops Controls" panel.
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
import jax
import jax.numpy as jnp
import optax

jax.config.update("jax_default_device", jax.devices("gpu")[0])  # default "gpu"
jax.config.update("jax_enable_x64", True)  # default False (ie use float32)


# =====================================================================================================================
# Custom parameterized force field with JAX (adaptation of the SofaPython3 example)
# =====================================================================================================================
@jax.jit
def get_force(position, length, stiffness):
    """
    Spring between the origin and the given position.

    position: array of shape (n_particles, n_dimensions)
    length: array of shape (n_particles,)
    stiffness: array of shape (n_particles,)
    """
    distance = jnp.sqrt(jnp.sum(position**2, axis=1, keepdims=True))
    direction = position / distance
    return - stiffness[:, None] * (distance - length[:, None]) * direction

@jax.jit
def get_dforce(position, length, stiffness, vector):
    def get_force_from_position(x):
        return get_force(x, length, stiffness)
    return jax.jvp(get_force_from_position, (position,), (vector,))[1]

@jax.jit
def get_kmatrix(position, length, stiffness):
    def get_force_from_position(x):
        return get_force(x, length, stiffness)
    return jax.jacrev(get_force_from_position)(position)

@jax.jit
def get_length_gradient(position, length, stiffness, vector):
    def get_force_from_length(l):
        return get_force(position, l, stiffness)
    _, vjp = jax.vjp(get_force_from_length, length)
    return vjp(vector)[0]  # note: vjp(vector) is the tuple (length_gradient,)

@jax.jit
def get_stiffness_gradient(position, length, stiffness, vector):
    def get_force_from_stiffness(s):
        return get_force(position, length, s)
    _, vjp = jax.vjp(get_force_from_stiffness, stiffness)
    return vjp(vector)[0]  # note: vjp(vector) is the tuple (stiffness_gradient,)

@jax.jit
def get_length_and_stiffness_gradient(position, length, stiffness, vector):
    def get_force_form_parameters(l, s):
        return get_force(position, l, s)
    _, vjp = jax.vjp(get_force_form_parameters, length, stiffness)
    return vjp(vector)  # note: vjp(vector) is the tuple (length_gradient, stiffness_gradient)


class JaxForceField(sofadiff.ParameterizedForceFieldVec3d):
    def __init__(self, length, stiffness, *args, **kwargs):
        sofadiff.ParameterizedForceFieldVec3d.__init__(self, *args, **kwargs)
        self.length_arg = length
        self.stiffness_arg = stiffness
        self.length_parameter = None
        self.stiffness_parameter = None

    def init(self):
        self.addData("length", self.length_arg, type="vector<double>")
        self.addData("stiffness", self.stiffness_arg, type="vector<double>")
        self.length_parameter = self.get_parameter("length", None)
        self.stiffness_parameter = self.get_parameter("stiffness", None)

    def addForce(self, mechanical_parameters, out_force, position, velocity):
        with out_force.writeableArray() as wa:
            wa[:] += get_force(position.value, self.length.value, self.stiffness.value)

    def addDForce(self, mechanical_parameters, df, dx):
        with df.writeableArray() as wa:
            wa[:] += get_dforce(self.mstate.position.value, self.length.value, self.stiffness.value, dx.value) * mechanical_parameters['kFactor']

    def addKToMatrix(self, mechanical_parameters, n_particles, n_dimensions):
        jacobian = get_kmatrix(self.mstate.position.value, self.length.value, self.stiffness.value)
        jacobian = jacobian.reshape((n_particles*n_dimensions, n_particles*n_dimensions))
        i, j = jacobian.nonzero()
        sparse_jacobian = jnp.stack([i, j, jacobian[i, j]], axis=1)
        return np.array(sparse_jacobian)

    def propagate_gradient_to_parameters(self, gradient):
        if self.length_parameter is None and self.stiffness_parameter is None:
            return
        position, length, stiffness = self.mstate.position.value, self.length.value, self.stiffness.value
        if self.length_parameter is not None and self.stiffness_parameter is not None:
            length_gradient, stiffness_gradient = get_length_and_stiffness_gradient(position, length, stiffness, gradient.value)
            self.length_parameter.gradient += np.array(length_gradient)
            self.stiffness_parameter.gradient += np.array(stiffness_gradient)
        elif self.length_parameter is not None:
            length_gradient = get_length_gradient(position, length, stiffness, gradient.value)
            self.length_parameter.gradient += np.array(length_gradient)
        elif self.stiffness_parameter is not None:
            stiffness_gradient = get_stiffness_gradient(position, length, stiffness, gradient.value)
            self.stiffness_parameter.gradient += np.array(stiffness_gradient)


# =====================================================================================================================
# Custom gradient descent with optax (JAX)
# =====================================================================================================================
class OptaxGradientDescent(sofadiff.GradientBasedOptimizationLoop):
    def __init__(self, optax_optimizer, **kwargs):
        sofadiff.GradientBasedOptimizationLoop.__init__(self, **kwargs)
        self.optax_optimizer = optax_optimizer
        self.optimizers = []
        self.opt_states = []

    def initialize(self):
        self.optimizers = [self.optax_optimizer(parameter["learningRate"]) for parameter in self.parameters]
        self.opt_states = [optimizer.init(parameter.value) for optimizer, parameter in zip(self.optimizers, self.parameters)]

    def update_parameters(self):
        for optimizer, opt_state, parameter in zip(self.optimizers, self.opt_states, self.parameters):
            updates, opt_state = optimizer.update(parameter.gradient, opt_state, parameter.value)
            next_value = optax.apply_updates(parameter.value, updates)
            if "lowerBound" in parameter:
                next_value = np.maximum(next_value, parameter["lowerBound"])
            if "upperBound" in parameter:
                next_value = np.minimum(next_value, parameter["upperBound"])
            parameter.next_value = next_value


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

    # ================================== Gradient descent base on optax.sgd algorithm =================================
    add_object(OptaxGradientDescent(optax.sgd, name="optax-gradient-descent", parameters="@/Parameters/parameter"))
    # =================================================================================================================

    add_object("DifferentiableAnimationLoop", name="differentiable-simulator", computeBoundingBox=False)

    add_object("MechanicalObject", template="Vec3d", name="origin", position="0 0 0", showObject="true", drawMode="1", showObjectScale="0.05")

    with Node("Parameters"):
        add_object("TrainableParameterVector", name="parameter", value="6 6", learningRate=0.5, lowerBound=1)

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixd", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton="20", maxNbIterationsLineSearch="1", warnWhenLineSearchFails="false")
        add_object("StaticSolver", name="static", newtonSolver="@newton")
        add_object("StaticAdjointSolver", name="adjoint")

        add_object("MechanicalObject", template="Vec3d", name="state", position="0 -7 0  0 -7 0", showObject="true", drawMode="1", showObjectScale="0.1", showColor=(1, 0, 0, 1))
        add_object("UniformMass", template="Vec3d", name="mass", totalMass=1)

        # ============================ Springs with stiffness equal to length, because why not ========================
        add_object(JaxForceField(name="Springs", stiffness="@/Parameters/parameter.value", length="@/Parameters/parameter.value"))
        # =============================================================================================================

    with Node("Loss"):
        add_object("MechanicalObject", template="Vec3d", name="state", position="0 -6 0  0 -8 0", showObject="true", drawMode="1", showObjectScale="0.08", showColor=(0, 0, 1, 1))
        with Node("Distance"):
            add_object("MechanicalObject", template="Vec1d", name="state", position="0")
            add_object("DistanceFromTargetMapping", input="@Physics/state", targetPositions="@/Loss/state.position")
            with Node("MSE"):
                add_object("LossState", name="state")
                add_object("MeanSquaredErrorMapping")
