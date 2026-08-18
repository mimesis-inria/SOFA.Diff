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
def random_unit_ball_point():
    while True:
        point = np.random.uniform(-1, 1, size=3)
        if np.sum(point**2) <= 1:
            return point

class TargetController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

    def onAnimateEndEvent(self, *args, **kwargs):
        root = self.getContext().getRoot()
        target = root["Loss.state"]
        loss = root["Loss.Distance.MSE.state"].value.getValue()[0, 0]
        if loss < 0.01:
            new_target = np.array([0, 0, 10]) + 4 * random_unit_ball_point()
            target.startingPosition.setValue(new_target[None, :].tolist())


def createScene(root):
    set_current_node(root)
    set_data(dt=0.1, gravity=(0, 0, 0))

    with Node("Plugins"):
        add_object("RequiredPlugin", pluginName=[
            "SOFA.Diff",
            "Elasticity",  # Needed to use components [HexahedronLinearSmallStrainFEMForceField]
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
            "Sofa.Component.Topology.Container.Dynamic",  # Needed to use components [TetrahedronSetGeometryAlgorithms,TetrahedronSetTopologyContainer,TetrahedronSetTopologyModifier]
            "Sofa.Component.Topology.Mapping",  # Needed to use components [Hexa2TetraTopologicalMapping]
            "SofaMatrix",  # Needed to  use components [GlobalSystemMatrixExporter]
        ])

    add_object("VisualStyle", displayFlags="showBehavior showBehaviorModels showForceFields showMappings")

    add_object("GradientDescentOptimizationLoop", name="GradientDescent", parameters="@/Parameters/stiffness")
    add_object("DifferentiableAnimationLoop", computeBoundingBox=False)
    add_object(TargetController(root))

    n = 10  # Number of cubes in the beam
    r = 3  # Subdivision of cubes
    m = 5  # Number of springs

    with Node("Parameters"):
        add_object("TrainableParameterVector", name="stiffness", value=(1,)*m, learningRate=1.0, lowerBound=0.1)

    with Node("Fixed"):
        add_object("MechanicalObject", template="Vec3d", name="state", position=(0, 0, 20, -10, 0, 5, 10, 0, 5, 0, -10, 5, 0, 10, 5), showObject="true", drawMode="1", showObjectScale="0.08")

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton=20, maxNbIterationsLineSearch=10, warnWhenLineSearchFails="false")#, relativeSuccessiveStoppingThreshold=0, relativeInitialStoppingThreshold=0, absoluteResidualStoppingThreshold=0, relativeEstimateDifferenceThreshold=0, absoluteEstimateDifferenceThreshold=0)
        add_object("StaticSolver", name="static", newtonSolver="@newton")
        add_object("StaticAdjointSolver", name="adjoint")

        add_object("RegularGridTopology", name="grid", min=(-0.5, -0.5, -0.5), max=(0.5, 0.5, n-0.5), n=(r+1, r+1, n*r+1))
        add_object("MechanicalObject", template="Vec3d", name="state", position="@grid.position")
        add_object(
            "HexahedronFEMForceField",
            name="elasticity",
            youngModulus="100",
            poissonRatio=0.45,
        )

        with Node("Constraint"):
            add_object("BoxROI", name="box", box=((-2, -2, -0.6), (2, 2, -0.4)))
            add_object("FixedProjectiveConstraint", indices="@box.indices")

        with Node("Springs"):
            add_object("MechanicalObject", template="Vec3d", name="state", position=(0, 0, n-0.5))
            add_object("BarycentricMapping")
            add_object(
                "ParameterizedSpringForceField", name="spring",
                stiffness="@/Parameters/stiffness.value",
                length=(0.0,)*m,  damping=(0,)*m, elongationOnly=(0,)*m, enabled=(1,)*m,
                object1="@/Physics/Springs/state", indices1=(0, 0, 0, 0, 0),
                object2="@/Fixed/state", indices2=(0, 1, 2, 3, 4),
            )

        with Node("Marker"):
            add_object("MechanicalObject", template="Vec3d", name="state", position=(0, 0, n-0.5))
            add_object("BarycentricMapping")

        add_object("VisualMesh", position="@state.position", topology="@grid")

    with Node("Loss"):
        add_object("MechanicalObject", template="Vec3d", name="state", position=(0, 0, 10), showObject="true", drawMode="1", showObjectScale="0.1", showColor="1 0 0 1")
        with Node("Distance"):
            add_object("MechanicalObject", template="Vec1d", name="state", position="0")
            add_object("DistanceFromTargetMapping", name="mapping", input="@Physics/Marker/state", targetPositions="@/Loss/state.position")
            with Node("MSE"):
                add_object("LossState", name="state")
                add_object("MeanSquaredErrorMapping")

def main():
    from validation import validate_parameter_gradient, validate_force_gradient
    validate_parameter_gradient(createScene, "Parameters.stiffness", "Loss.Distance.MSE.state", parameter_value=[20]*5, verbose=True)
    # validate_force_gradient(createScene, "Physics", "Loss.Distance.MSE.state")

if __name__ == "__main__":
    main()