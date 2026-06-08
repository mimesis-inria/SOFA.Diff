# =====================================================================================================================
# Preamble
# =====================================================================================================================
import Sofa
import numpy as np


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
def createScene(root, initial_displacement=0, newton_iterations=1, jac_file=""):
    set_current_node(root)
    set_data(dt=1.0, gravity=(0, 0, 0))

    with Node("Plugins"):
        add_object("RequiredPlugin", pluginName=[
            "Elasticity",  # Needed to use components [HexahedronHyperelasticityFEMForceField]
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
    add_object("DefaultAnimationLoop", computeBoundingBox=False)

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton=newton_iterations, maxNbIterationsLineSearch=1, warnWhenLineSearchFails="false", relativeSuccessiveStoppingThreshold=0, relativeInitialStoppingThreshold=0, absoluteResidualStoppingThreshold=0, relativeEstimateDifferenceThreshold=0, absoluteEstimateDifferenceThreshold=0)
        add_object("StaticSolver", name="static", newtonSolver="@newton")

        if jac_file != "":
            add_object("GlobalSystemMatrixExporter", exportEveryNumberOfSteps="1", filename=jac_file, printLog="true", format="txt", precision="12")

        add_object("RegularGridTopology", name="grid", min=(-0.5, -0.5, -0.5), max=(0.5, 0.5, 0.5), n=(2, 2, 2))
        deformed_position = np.array([
            [-0.5, -0.5, -0.5],
            [+0.5, -0.5, -0.5],
            [-0.5, +0.5, -0.5],
            [+0.5, +0.5, -0.5],
            [-0.5, -0.5, +0.5],
            [+0.5, -0.5, +0.5],
            [-0.5, +0.5, +0.5],
            [+0.5, +0.5, +0.5],
        ]) + initial_displacement
        add_object("MechanicalObject", template="Vec3d", name="state", rest_position="@grid.position", position=deformed_position)
        # add_object("HexahedronFEMForceField",
        #            name="elasticity",
        #            youngModulus=200,
        #            poissonRatio=0.45,
        #            # updateStiffness=True,
        #            )
        add_object("StVenantKirchhoffMaterial", youngModulus="200", poissonRatio="0.45")
        add_object("HyperelasticityFEMForceField", name="elasticity")

        add_object("VisualMesh", position="@state.position", topology="@grid")


def load_matrix(filename):
    A = []
    with open(filename, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            A.append([float(x) for x in line.split() if x not in ["[", "]"]])
    return np.array(A)

def print_matrix(mat, header=None):
    rows, cols = mat.shape
    if header is not None:
        print(f"{' '+header+' ':=^{9*cols-1}}")
    for row in mat:
        print(" ".join(f"{coeff:8.3f}" for coeff in row))


def get_force(u):
    root = Sofa.Core.Node("root")
    createScene(root, initial_displacement=u, newton_iterations=0)
    Sofa.Simulation.initRoot(root)
    Sofa.Simulation.animate(root, root.dt.value)
    return root.Physics.state.force.getValue()

def get_jacobian_fd(u, h, centered=True):
    n = len(u)
    f = get_force(u)
    jac = np.zeros((n*3, n*3))
    for i in range(n):
        for j in range(3):
            uh = u.copy()
            uh[i, j] += h
            f_plus = get_force(uh)
            uh = u.copy()
            uh[i, j] -= h
            f_minus = get_force(uh)
            if centered:
                col = ((f_plus - f_minus) / (2*h)).reshape(-1)
            else:
                col = ((f_plus - f) / h).reshape(-1)
            jac[:, i*3 + j] = col
    return jac


def main():
    # Deformation at which the jacobians are evaluated
    u = np.array([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [ 0.03546312, -0.08985449, -0.00126088],
        [ 0.03438233,  0.03236226, -0.08005212],
        [ 0.01568617,  0.09636057, -0.05149985],
        [-0.07768177,  0.00867797,  0.09879704]
    ])
    # SOFA jacobian
    root = Sofa.Core.Node("root")
    createScene(root, initial_displacement=u, jac_file="tmp_")
    Sofa.Simulation.initRoot(root)
    Sofa.Simulation.animate(root, root.dt.value)
    jac_sofa = -load_matrix("tmp_00001.txt")
    # Finite differences jacobian
    h = 0.001
    jac_fd = get_jacobian_fd(u, h, centered=True)
    # Print results
    print_matrix(jac_sofa, "BUILD STIFFNESS MATRIX")
    print_matrix(jac_fd, "FINITE DIFFERENCES")

if __name__ == "__main__":
    main()
