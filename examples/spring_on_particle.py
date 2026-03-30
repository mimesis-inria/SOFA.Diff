# =====================================================================================================================
# Some utils for easier scene description
# =====================================================================================================================
import Sofa

_current_node: Sofa.Core.Node


def get_current_node():
    try:
        return _current_node
    except NameError as error:
        raise RuntimeError(
            "You must call 'set_current_node(root)' at the beginning of createScene()"
        ) from error


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
# A controller to extract data
# =====================================================================================================================
import numpy as np


class WriteDataController(Sofa.Core.Controller):
    """
    Store the value from the given data in a file
    """
    def __init__(self, path, data_fields, animation_loop, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.path = path
        self.data_fields = data_fields
        self.animation_loop = animation_loop  # For a dirty hack

        self._trajectories = [[] for _ in self.data_fields]

    def onAnimateEndEvent(self, _):
        if not self.animation_loop.differentiableMode.getValue():
            for trajectory, data_field in zip(self._trajectories, self.data_fields):
                value = float(data_field.getValue())  # Conversion to float to get an actual value and not a reference
                trajectory.append(value)
            np.savetxt(self.path, np.stack(self._trajectories))


# =====================================================================================================================
# Custom gradient descent
# =====================================================================================================================
import SofaDiff  # Don't forget that

class RandomOptimizer(SofaDiff.OptimizationLoop):
    def initialize(self):
        self.start_with_update = True

    def update_parameters(self):
        for parameter in self.parameters:
            parameter.next_value = np.random.uniform(parameter["lowerBound"], parameter["upperBound"], size=parameter.size)

class MyGradientDescent(SofaDiff.GradientBasedOptimizationLoop):
    def __init__(self, *args, **kwargs):
        SofaDiff.GradientBasedOptimizationLoop.__init__(self, *args, **kwargs)  # Do not use super()
        # Here the parameters are not initialized yet...

    def initialize(self):
        # ... Therefore, any initialization with regard to the parameters should be done here instead
        for parameter in self.parameters:
            print(parameter)

    def update_parameters(self):
        # The list of parameters to be optimized can be accessed in self.parameters
        for parameter in self.parameters:
            # The parameter's value and gradient can be accessed like attributes
            # Any "hyperparameter" can be added to a Parameter and accessed with parameter["key"]
            next_value = parameter.value - parameter["learningRate"] * parameter.gradient
            # For optional hyperparameters you can check if they were provided or not
            if "lowerBound" in parameter:
                next_value = np.maximum(next_value, parameter["lowerBound"])
            if "upperBound" in parameter:
                next_value = np.minimum(next_value, parameter["upperBound"])
            # The task of this method is to set the next value of each parameter:
            parameter.next_value = next_value
            # This assignment involves a copy, so it's preferable to do it only once


import optax

class OptaxGradientDescent(SofaDiff.GradientBasedOptimizationLoop):
    def __init__(self, optax_optimizer, **kwargs):
        SofaDiff.GradientBasedOptimizationLoop.__init__(self, **kwargs)
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
# The scene
# =====================================================================================================================
def createScene(root):
    set_current_node(root)
    set_data(dt=0.05, gravity=(0, -10, 0))

    with Node("Plugins"):
        add_object("RequiredPlugin", pluginName=[
            "SofaDiff",
            "Sofa.Component.Visual",  # Needed to use components [VisualStyle]
            "Sofa.Component.LinearSolver.Iterative",  # Needed to use components [CGLinearSolver]
            "Sofa.Component.LinearSolver.Direct",  # Needed to use components [SparseLDLSolver]
            "Sofa.Component.ODESolver.Backward",  # Needed to use components [NewtonRaphsonSolver, StaticSolver]
            "Sofa.Component.StateContainer",  # Needed to use components [MechanicalObject]
            "Sofa.Component.Mass",  # Needed to use components [UniformMass]
            "Sofa.Component.MechanicalLoad",  # Needed to use components [ConstantForceField]
            "Sofa.Component.Constraint.Projective",  # Needed to use components [FixedProjectiveConstraint]
            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components [SpringForceField]
            "Sofa.Component.Topology.Container.Grid",  # Needed to use components [RegularGridTopology]
            "Sofa.Component.Engine.Select",  # Needed to use components [BoxROI]
            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components [HexahedronFEMForceField]
            "Sofa.Component.Mapping.Linear",  # Needed to use components [BarycentricMapping]
            "Sofa.Component.Mapping.NonLinear",  # Needed to use components [DistanceFromTargetMapping]
            # "SofaMatrix",  # Needed to  use components [GlobalSystemMatrixExporter]
        ])

    add_object("VisualStyle", displayFlags="showBehavior showBehaviorModels showForceFields showMappings")

    add_object("GridSearchOptimizationLoop", name="cpp-grid-search", parameters="@/Parameters/stiffness")
    # add_object("GradientDescentOptimizationLoop", name="cpp-gradient-descent", parameters="@/Parameters/stiffness")
    # add_object(RandomOptimizer(name="numpy-random-search", parameters="@/Parameters/stiffness"))
    add_object(MyGradientDescent(name="numpy-gradient-descent", parameters="@/Parameters/stiffness"))
    # add_object(OptaxGradientDescent(optax.sgd, name="optax-gradient-descent", parameters="@/Parameters/stiffness"))

    add_object("DifferentiableAnimationLoop", name="differentiable-simulator", computeBoundingBox=False)
    # add_object("DefaultAnimationLoop", name="default-simulator", computeBoundingBox=False)

    add_object("MechanicalObject", template="Vec3d", name="state", position="0 10 0")

    with Node("Parameters"):
        add_object("TrainableParameterVector", name="stiffness", value="10", learningRate=1.0, lowerBound=1, upperBound=50, resolution=50)
        add_object("TrainableParameterVector", name="mock", value="10 20 30", learningRate=1.0)

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixd", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton="100", maxNbIterationsLineSearch="1", warnWhenLineSearchFails="false")
        add_object("StaticSolver", name="static", newtonSolver="@newton")
        add_object("StaticAdjointSolver", name="adjoint")

        add_object("MechanicalObject", template="Vec3d", name="state", position="0 0 0", showObject="true", drawMode="1", showObjectScale="0.1")
        add_object("UniformMass", template="Vec3d", name="mass", totalMass="10")

        add_object("ParameterizedSpringForceField", name="spring", object1="@/Physics/state", indices1="0", object2="@/state", indices2="0", length="5", stiffness="@/Parameters/stiffness.value", damping=0)

    with Node("Loss", tags="NoBBox"):
        add_object("MechanicalObject", template="Vec3d", name="state", position="0 -2 0", showObject="true", drawMode="1", showObjectScale="0.08")
        with Node("Distance"):
            add_object("MechanicalObject", template="Vec1d", name="state", position="0")
            add_object("DistanceFromTargetMapping", input="@Physics/state", targetPositions="@/Loss/state.position")
            with Node("MSE"):
                add_object("LossState", name="state")
                add_object("MeanSquaredErrorMapping")

    # add_object(WriteDataController("data_control.txt", [root.Parameters.stiffness.value, root.Loss.Distance.MSE.state.value], root.simulator))

if __name__ == "__main__":
    help(SofaDiff)
