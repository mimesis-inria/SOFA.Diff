"""
Introduction to gradient based optimization in SofaDiff.

This example introduces DifferentiableAnimationLoop and StaticAdjointSolver to set up a simple gradient descent.
The optimization task is the same as in the example "1_grid_search.py".
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
            "SOFA.Diff",
            "Sofa.Component.Visual",  # Needed to use components [VisualStyle]
            "Sofa.Component.LinearSolver.Direct",  # Needed to use components [SparseLDLSolver]
            "Sofa.Component.ODESolver.Backward",  # Needed to use components [NewtonRaphsonSolver, StaticSolver]
            "Sofa.Component.StateContainer",  # Needed to use components [MechanicalObject]
            "Sofa.Component.Mass",  # Needed to use components [UniformMass]
            "Sofa.Component.Mapping.NonLinear",  # Needed to use components [DistanceFromTargetMapping]
        ])

    add_object("VisualStyle", displayFlags="showBehavior showBehaviorModels showForceFields showMappings")

    # ============================================== 1. Add optimizer(s) ==============================================
    add_object("GradientDescentOptimizationLoop", name="GradientDescent", parameters="@/Parameters/stiffness")
    # We use "GradientDescentOptimizationLoop", a simple gradient descent algorithm
    # We provide the parameter(s) to be optimized, here the stiffness parameter defined below
    # In the ImGUI interface, we can display the "Loops Controls" window to interact with the optimizer
    # =================================================================================================================

    # ======================================= 2. Use differentiable simulation ========================================
    add_object("DifferentiableAnimationLoop", computeBoundingBox=False)
    # Any gradient based optimization requires a differentiable animation loop to work
    # The differentiable animation loop comes with a "step_adjoint()" method that triggers the computation of gradients
    # The actual computing is done by an AdjointSolver that must be added next to the standard solver, see below
    # =================================================================================================================

    add_object("MechanicalObject", template="Vec3d", name="fixed", position="-2 5 0  2 5 0")

    # ============================================== 3. Add parameter(s) ==============================================
    with Node("Parameters"):
        add_object("TrainableParameterVector", name="stiffness", value="5 5", learningRate=(1.0, 2.0))
        # We use "TrainableParameterVector" because the spring force field below expects one stiffness per spring
        # We provide the "learningRate" hyperparameter for the gradient descent optimizer
        # We provide two different learning rates, one for each spring, for illustration purposes
    # =================================================================================================================

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixd", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton="100", maxNbIterationsLineSearch="1", warnWhenLineSearchFails="false", relativeSuccessiveStoppingThreshold=0, relativeInitialStoppingThreshold=0, absoluteResidualStoppingThreshold=0, relativeEstimateDifferenceThreshold=0, absoluteEstimateDifferenceThreshold=0)
        add_object("StaticSolver", name="static", newtonSolver="@newton")

        # =========================================== 4. Add adjoint solver ===========================================
        add_object("StaticAdjointSolver", name="adjoint")
        # The static adjoint solver can differentiate the static solver above, even if used for quasi-static simulation
        # This syntax will allow for more flexibility when the support for dynamic simulations is added
        # =============================================================================================================

        add_object("MechanicalObject", template="Vec3d", name="state", position="0 0 0", showObject="true", drawMode="1", showObjectScale="0.1", showColor="255 0 0 255")
        add_object("UniformMass", template="Vec3d", name="mass", totalMass=1)

        # ==================================== 5. Use parameterized force field(s) ====================================
        add_object("ParameterizedSpringForceField", name="Springs", stiffness="@/Parameters/stiffness.value", object1="@/Physics/state", indices1="0 0", object2="@/fixed", indices2="0 1", length="5 5", damping="0 0", elongationOnly="0 0", enabled="1 1")
        # The parameterized version comes from SofaDiff and includes a method to differentiate wrt length and stiffness
        # Otherwise it is simply an extension of "SpringForceField", with the exact same interface
        # =============================================================================================================


    # ============================================== 6. Add loss function =============================================
    with Node("Loss"):
        add_object("MechanicalObject", template="Vec3d", name="state", position="0.7 -1.0 0", showObject="true", drawMode="1", showObjectScale="0.08", showColor="0 0 255 255")
        with Node("Distance"):
            add_object("MechanicalObject", template="Vec1d", name="state", position="0")
            add_object("DistanceFromTargetMapping", input="@Physics/state", targetPositions="@/Loss/state.position")
            with Node("MSE"):
                add_object("LossState", name="state")
                add_object("MeanSquaredErrorMapping")
    # Above is a somewhat convoluted way of computing the mean squared error between the particle and the target
    # What is important here is that the final loss to be minimized is in a "LossState", not a "MechanicalObject"
    # =================================================================================================================


def main():
    from validation import validate_parameter_gradient, validate_force_gradient
    validate_parameter_gradient(createScene, "Parameters.stiffness", "Loss.Distance.MSE.state", verbose=True)
    # validate_force_gradient(createScene, "Physics", "Loss.Distance.MSE.state")

if __name__ == "__main__":
    main()