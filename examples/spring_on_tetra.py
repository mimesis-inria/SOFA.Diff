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
    def __init__(self, name):
        self.name = name
        self.parent = None
        self.child = None

    def __enter__(self):
        self.parent = get_current_node()
        self.child = add_child(self.name)
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
    set_data(dt=0.05, gravity=(0, -9.8, 0))

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
    add_object("GradientDescentOptimizationLoop", name="optimizer")
    add_object("DifferentiableAnimationLoop", computeBoundingBox=False)

    add_object("MechanicalObject", template="Vec3d", name="state", position="0 10 0")

    with Node("Parameters"):
        add_object("TrainableParameterVector", name="stiffness", value="30", learningRate="0.005")

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixd", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton="100", maxNbIterationsLineSearch="1", warnWhenLineSearchFails="false")
        # add_object("StaticSolver", newtonSolver="@newton", name="static")
        # add_object("StaticAdjointSolver", name="adjoint")
        add_object("EulerImplicitSolver", name="euler")
        add_object("ImplicitAdjointSolver", name="adjoint")

        add_object("MechanicalObject", template="Vec3d", name="state", position="0 0 0", showObject="true", drawMode="1", showObjectScale="0.1")
        add_object("UniformMass", template="Vec3d", name="mass", totalMass="10")
        # add_object("MechanicalObject", template="Vec3d", name="state", position="0 0 0", showObject="true", drawMode="1", showObjectScale="0.1")
        # add_object("UniformMass", template="Vec3d", name="mass", totalMass="1")
        # add_object("ConstantForceField", forces=(0, -9.8, 0, 0, 0, 0, 0))
        # with Node("Point"):
        #     add_object("MechanicalObject", template="Vec3d", name="state", position="0 1 0")
        #     add_object("RigidMapping")
        # add_object("RestShapeSpringsForceField", angularStiffness=1000000)

        add_object("ParameterizedSpringForceField", name="spring", object1="@/Physics/state", indices1="0", object2="@/state", indices2="0", length="3", stiffness="@/Parameters/stiffness.value")

    with Node("Loss"):
        add_object("MechanicalObject", template="Vec3d", name="state", position="0 2.67 0", showObject="true", drawMode="1", showObjectScale="0.08")
        with Node("Distance"):
            add_object("MechanicalObject", template="Vec1d", name="state", position="0")
            add_object("DistanceFromTargetMapping", input="@Physics/state", targetPositions="@/Loss/state.position")
            with Node("MSE"):
                add_object("LossState", name="state")
                add_object("MeanSquaredErrorMapping")
