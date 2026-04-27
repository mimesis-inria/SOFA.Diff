"""
Introduction to optimization in SofaDiff with a simple grid search.

This example shows how to use an optimizer along with some parameters and a loss to set up a simple optimization.
In this example, a particle is attached with springs to two fixed points, and the task is to find the stiffness of both
springs so that the particle gets as close as possible to the given target.
"""

# =====================================================================================================================
# Tools to make writing the scene more convenient
# =====================================================================================================================
import Sofa

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
            "SofaDiff",
            "Sofa.Component.Visual",  # Needed to use components [VisualStyle]
            "Sofa.Component.LinearSolver.Direct",  # Needed to use components [SparseLDLSolver]
            "Sofa.Component.ODESolver.Backward",  # Needed to use components [NewtonRaphsonSolver, StaticSolver]
            "Sofa.Component.StateContainer",  # Needed to use components [MechanicalObject]
            "Sofa.Component.Mass",  # Needed to use components [UniformMass]
            "Sofa.Component.Mapping.NonLinear",  # Needed to use components [DistanceFromTargetMapping]
            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components [SpringForceField]
        ])

    add_object("VisualStyle", displayFlags="showBehavior showBehaviorModels showForceFields showMappings")

    # ============================================= 1. Add optimizer(s) ===============================================
    add_object("GridSearchOptimizationLoop", name="GridSearch", parameters="@/Parameters/stiffness")
    # We use "GridSearchOptimizationLoop", a simple optimizer that explores the parameter space through a grid pattern
    # We provide the parameter(s) to be optimized, here the stiffness parameter defined below
    # In the ImGUI interface, we can display the "Loops Controls" window to interact with the optimizer
    # =================================================================================================================

    add_object("DefaultAnimationLoop", computeBoundingBox=False)

    add_object("MechanicalObject", template="Vec3d", name="fixed", position="-2 5 0  2 5 0")

    # ============================================= 2. Add parameter(s) ===============================================
    with Node("Parameters"):
        add_object("TrainableParameterVector", name="stiffness", value="5 5", lowerBound=1, upperBound=10, resolution=10)
        # We use "TrainableParameterVector" because the spring force field below expects one stiffness per spring
        # We provide the hyperparameters "lowerBound", "upperBound" and "resolution" for the grid search optimizer
    # =================================================================================================================

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixd", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton="20", maxNbIterationsLineSearch="1", warnWhenLineSearchFails="false")
        add_object("StaticSolver", name="static", newtonSolver="@newton")

        add_object("MechanicalObject", template="Vec3d", name="state", position="0 0 0", showObject="true", drawMode="1", showObjectScale="0.1", showColor=(255, 0, 0, 255))
        add_object("UniformMass", template="Vec3d", name="mass", totalMass=1)

        # =================================== 3. Use parameter(s) in force field(s) ===================================
        add_object("SpringForceField", name="Springs", stiffness="@/Parameters/stiffness.value", object1="@/Physics/state", indices1="0 0", object2="@/fixed", indices2="0 1", length="5 5", damping="0 0", elongationOnly="0 0", enabled="1 1")
        # We pass *the value of* the stiffness parameter as data for the stiffness of the springs
        # =============================================================================================================


    # ============================================== 4. Add loss function =============================================
    with Node("Loss"):
        add_object("MechanicalObject", template="Vec3d", name="state", position="0.7 -1.0 0", showObject="true", drawMode="1", showObjectScale="0.08", showColor=(0, 0, 255, 255))
        with Node("Distance"):
            add_object("MechanicalObject", template="Vec1d", name="state", position="0")
            add_object("DistanceFromTargetMapping", input="@Physics/state", targetPositions="@/Loss/state.position")
            with Node("MSE"):
                add_object("LossState", name="state")
                add_object("MeanSquaredErrorMapping")
    # Above is a somewhat convoluted way of computing the mean squared error between the particle and the target
    # What is important here is that the final loss to be minimized is in a "LossState", not a "MechanicalObject"
    # =================================================================================================================