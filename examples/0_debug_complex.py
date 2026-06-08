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
from dataclasses import dataclass, field

tmp = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0.2],
    [0, 0, 0.2],
    [0, 0, 0],
    [0, 0, 0],
])

theta = 0#np.pi/4
rota = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta), np.cos(theta), 0],
    [0, 0, 1]
])
theta = 0
rota2 = np.array([
    [np.cos(theta), 0, -np.sin(theta)],
    [0, 1, 0],
    [np.sin(theta), 0, np.cos(theta)],
])

u_test = np.zeros((8, 3))
u_test[4:] += 0.1 * np.array([
    [-0.5, -0.5, +0.9],
    [+0.5, -0.5, +0.9],
    [-0.5, +0.5, +0.5],
    [+0.5, +0.5, +0.5],
]) @ rota @ rota2

@dataclass
class Inputs:
    stiffness: float = 40
    df: tuple = (0, 0, 0)
    target: tuple = (0, 0.760+0.1, 0.635-0.1)
    newton_iterations: int = 100
    u: np.ndarray = field(default_factory=lambda: np.zeros((8, 3)))
    jac_file: str = ""

def createScene(root, inputs=Inputs()):
    set_current_node(root)
    set_data(dt=0.1, gravity=(0, 0, 0))

    with Node("Plugins"):
        add_object("RequiredPlugin", pluginName=[
            "SOFA.Diff",
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

    add_object("GradientDescentOptimizationLoop", name="GradientDescent", parameters="@/Parameters/stiffness")
    add_object("DifferentiableAnimationLoop", computeBoundingBox=False)
    # add_object("DefaultAnimationLoop", computeBoundingBox=False)

    add_object("StVenantKirchhoffMaterial", youngModulus="200", poissonRatio="0.45")
    # add_object("NeoHookeanMaterial", youngModulus="400", poissonRatio="0.45")

    n = 1  # Length of the beam

    # with Node("Topology"):
    #     add_object("RegularGridTopology", name="grid", min=(0, -1.5, -1.5), max=(60, 1.5, 1.5), n=(61, 4, 4))
    #     add_object("RegularGridTopology", name="grid", min=(0, -0.5, -0.5), max=(n, 0.5, 0.5), n=(n+1, 2, 2))

    m = 1  # Number of springs
    with Node("Parameters"):
        add_object("TrainableParameterVector", name="youngModulus", value=1e2, learningRate=1e10)
        add_object("TrainableParameterVector", name="stiffness", value=(inputs.stiffness,)*m, learningRate=100, lowerBound=1)

    with Node("Fixed"):
        add_object("MechanicalObject", template="Vec3d", name="state", position=(0, 3, 2.5), showObject="true", drawMode="1", showObjectScale="0.08")

    with Node("Physics"):
        add_object("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d", name="solver", printLog="false")
        add_object("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton=inputs.newton_iterations, maxNbIterationsLineSearch=10, warnWhenLineSearchFails="false", relativeSuccessiveStoppingThreshold=0, relativeInitialStoppingThreshold=0, absoluteResidualStoppingThreshold=0, relativeEstimateDifferenceThreshold=0, absoluteEstimateDifferenceThreshold=0)
        add_object("StaticSolver", name="static", newtonSolver="@newton")
        # add_object("EulerImplicitSolver", name="euler")
        add_object("StaticAdjointSolver", name="adjoint")
        if inputs.jac_file != "":
            add_object("GlobalSystemMatrixExporter", exportEveryNumberOfSteps="1", filename=inputs.jac_file, printLog="true", format="txt", precision="12")

        r = 1  # Subdivision of unit squares
        # add_object("TetrahedronSetGeometryAlgorithms")
        # add_object("TetrahedronSetTopologyModifier")
        # add_object("TetrahedronSetTopologyContainer", name="topology", position="@/Topology/grid.position")
        # add_object("Hexa2TetraTopologicalMapping", input="@/Topology/grid", output="@topology", swapping=True)
        add_object("RegularGridTopology", name="grid", min=(-0.5, -0.5, -0.5), max=(0.5, 0.5, 0.5), n=(r+1, r+1, r+1))
        # position = [
        #     [-0.5, -0.5, -0.5],
        #     [-0.5, -0.5, 0.5],
        #     [-0.5, 0.5, 0.5],
        #     [-0.5, 0.5, -0.5],
        #     [0.5, -0.5, -0.5],
        #     [0.5, -0.5, 0.5],
        #     [0.5, 0.5, 0.5],
        #     [0.5, 0.5, -0.5],
        # ]  # ordered as in HexahedronFEMForceField
        position = np.array([
            [-0.5, -0.5, -0.5],
            [+0.5, -0.5, -0.5],
            [-0.5, +0.5, -0.5],
            [+0.5, +0.5, -0.5],
            [-0.5, -0.5, +0.5],
            [+0.5, -0.5, +0.5],
            [-0.5, +0.5, +0.5],
            [+0.5, +0.5, +0.5],
        ])  # ordered as in RegularGridTopology
        add_object("MechanicalObject", template="Vec3d", name="state", rest_position=position@rota@rota2, position=position@rota@rota2+inputs.u)
        # add_object("UniformMass", template="Vec3d", name="mass", totalMass="1")
        # add_object("HexahedronFEMForceField",
        #            name="elasticity",
        #            youngModulus="@/Parameters/youngModulus.value",
        #            poissonRatio=0.45,
        #            # updateStiffness=True,
        #            )
        add_object("HyperelasticityFEMForceField", name="elasticity")

        with Node("Constraint"):
            # add_object("BoxROI", name="box", box=((-0.6, -2, -2), (-0.4, 2, 2)))
            # add_object("BoxROI", name="box", box=((-2, -2, -0.6), (2, 2, -0.4)))
            add_object("FixedProjectiveConstraint", indices=[0, 1, 2, 3])

        # with Node("Springs"):
        #     add_object("MechanicalObject", template="Vec3d", name="state", position=(0, 0, 0.5))
        #     # add_object("ConstantForceField", mstate="@state", forces=[0, 0, -10], indices=[0])
        #     add_object("BarycentricMapping")
        #     # add_object(
        #     #     "ParameterizedSpringForceField", name="spring",
        #     #     stiffness="@/Parameters/stiffness.value",
        #     #     length=(0.0,)*m,  damping=(0,)*m, elongationOnly=(0,)*m, enabled=(1,)*m,
        #     #     object1="@/Physics/Springs/state", indices1=(0,),
        #     #     object2="@/Fixed/state", indices2=(0,),
        #     # )
        #     add_object("ParameterizedRestShapeSpringsForceField", points=[0], external_points=[0], external_rest_shape="@/Fixed/state", stiffness="@/Parameters/stiffness.value")
        # force = np.array(inputs.df)
        # add_object("ConstantForceField", name="lilPush", forces=[0.25 * force]*8)

        with Node("Marker"):
            add_object("MechanicalObject", template="Vec3d", name="state", position=(0, 0, 0.5))
            add_object("BarycentricMapping")

        add_object("VisualMesh", position="@state.position", topology="@grid")

    with Node("Loss"):
        add_object("MechanicalObject", template="Vec3d", name="state", position=inputs.target, showObject="true", drawMode="1", showObjectScale="0.08")
        with Node("Distance"):
            add_object("MechanicalObject", template="Vec1d", name="state", position="0")
            add_object("DistanceFromTargetMapping", input="@Physics/Marker/state", targetPositions="@/Loss/state.position")
            with Node("MSE"):
                add_object("LossState", name="state")
                add_object("MeanSquaredErrorMapping")


@dataclass
class Outputs:
    loss: float
    gradient: np.ndarray
    residual: float
    force_gradient: np.ndarray
    force: np.ndarray
    jacobian: np.ndarray

def load_matrix(filename):
    A = []
    with open(filename, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            A.append([float(x) for x in line.split() if x not in ["[", "]"]])
    return np.array(A)

def run_simulation(inputs=Inputs()) -> Outputs:
    root = Sofa.Core.Node("root")
    createScene(root, inputs)
    Sofa.Simulation.initRoot(root)
    Sofa.Simulation.animate(root, root.dt.value)

    loss = root.Loss.Distance.MSE.state.value.getValue()[0, 0]
    gradient = sum(root.Parameters.stiffness.gradient)
    residual = float(root.Physics.newton.residualGraph.getValue().split()[-1])
    force_gradient = None#root.Physics.Marker.state["gradient of the global loss wrt f"].getValue()[0]
    force = root.Physics.state.force.getValue()
    if inputs.jac_file != "" and inputs.newton_iterations > 0:
        jacobian = -load_matrix(inputs.jac_file + "00001.txt")
    else:
        jacobian = np.zeros((24, 24))
    return Outputs(loss=loss, gradient=gradient, residual=residual, force_gradient=force_gradient, force=force, jacobian=jacobian)

def write_loss_and_gradient(filename, stiffness_min, stiffness_max, n_points):
    points = []
    delta_stiffness = (stiffness_max - stiffness_min) / (n_points - 1)
    for i in range(n_points):
        stiffness = stiffness_min + i * delta_stiffness
        out = run_simulation(Inputs(stiffness=stiffness))
        points.append((stiffness, out.loss, out.gradient, out.residual))
    with open(f"outputs/{filename}.txt", "w") as file:
        file.write("\n".join([f"{s} {l} {g} {r}" for s, l, g, r in points]))

def get_force_gradients(h, stiffness=40, target=Inputs().target, newton_iterations=100):
    out = run_simulation(Inputs(stiffness=stiffness, target=target, newton_iterations=newton_iterations))
    loss_mid = out.loss
    grad_sofadiff = out.force_gradient

    grad_finite_difference = []
    for df in [(h, 0.0, 0.0), (0.0, h, 0.0), (0.0, 0.0, h)]:
        out = run_simulation(Inputs(stiffness=stiffness, target=target, newton_iterations=newton_iterations, df=df))
        grad_finite_difference.append((out.loss - loss_mid) / h)

    return grad_sofadiff, np.array(grad_finite_difference)

def print_dydf_fd(hs):
    dydf = {}
    for h in hs:
        dydf_sofadiff, dydf_fd = get_force_gradients(h)
        dydf[h] = dydf_fd

    print("============================== dy/df ================================")
    print(f"{'SofaDiff':30}", ", ".join(f"{x:9.6f}" for x in np.array(dydf_sofadiff)))
    for h in hs:
        print(f"{f'Finite Difference h={h:8}':30}", ", ".join(f"{x:9.6f}" for x in np.array(dydf[h])))
    print("=====================================================================")

def map_error_vs_target(x, y, h=0.01, newton_iterations=100):
    error = np.zeros((len(x), len(y)))
    for i, xi in enumerate(x):
        for j, yj in enumerate(y):
            target = (0, yj, xi)
            grad_sofadiff, grad_fd = get_force_gradients(h, target=target, newton_iterations=newton_iterations)
            diff = grad_sofadiff - grad_fd
            error[i, j] = np.sqrt(np.sum(diff**2))
    return error

def write_error_vs_target(filename):
    x_mid, y_mid = (0.626002, 1.413649)  # /!\ y is y, but x is z now (cf target above)
    dx = dy = np.linspace(-3, 3, 50)
    x = x_mid + dx
    y = y_mid + dy
    z = map_error_vs_target(x, y, newton_iterations=20)
    xx, yy = np.meshgrid(dx, dy, indexing="ij")
    np.save(filename, np.stack([xx, yy, z]))

def get_force(u):
    outputs = run_simulation(Inputs(u=u, newton_iterations=0))
    return outputs.force

def get_jacobian_fd(u, h, centered=True):
    f = get_force(u)
    jac = np.zeros((4*3, 4*3))
    for i in range(4):
        for j in range(3):
            uh = u.copy()
            uh[4+i, j] += h
            f_plus = get_force(uh)
            uh = u.copy()
            uh[4+i, j] -= h
            f_minus = get_force(uh)
            if centered:
                col = ((f_plus - f_minus) / (2*h))[4:, :].reshape(-1)
            else:
                col = ((f_plus - f) / h)[4:, :].reshape(-1)
            jac[:, i*3 + j] = col
    return jac

def test_jacobian_convergence(u, d=None):
    """
    Print table with convergence rate for the quantity
        f(x + h*d) - f(x) - h * Jac(f)(x) @ d
    with x = x0 + u, and h getting smaller and smaller.
    The rate of convergence should be 2 when h tends to 0.
    A rate of 1 means that the jacobian is incorrect.
    This test does not work for linear functions, for which the residual is exactly 0.
    """
    if d is None:
        d = np.concat([np.zeros((4, 3)), np.random.uniform(-1, 1, size=(4, 3))], axis=0)
    f_x = get_force(u).reshape(-1)
    jac_f_x = run_simulation(Inputs(u=u, jac_file="tmp_", newton_iterations=1)).jacobian
    # jac_f_x = -np.eye(len(f_x))
    # jac_f_x[12:, 12:] = get_jacobian_fd(u, 0.0001, centered=True)
    # jac_f_x = 0.5 * (jac_f_x + jac_f_x.T)
    hs = 10**np.linspace(0, -8, 9)
    residuals = []
    for h in hs:
        f_x_hd = get_force(u + h*d).reshape(-1)
        result = f_x_hd - f_x - h * jac_f_x @ d.reshape(-1)
        residuals.append(np.sqrt(np.sum(result**2)))
    hs = np.array(hs)
    residuals = np.array(residuals)
    rates = np.log10(residuals[1:]/residuals[:-1]) / np.log10(hs[1:]/hs[:-1])
    rates = np.concat([[0], rates])
    print("   h    | residual | rate |")
    print("———————————————————————————")
    for h, residual, rate in zip(hs, residuals, rates):
        print(f"{h:.1e} | {residual:8.1e} | {rate:.2f} |")

def print_matrix(mat, header=None):
    if header is not None:
        print(f"{' '+header+' ':=^108}")
    for row in mat:
        print(" ".join(f"{coeff:8.3f}" for coeff in row))


def main():
    # filename = "test_test"
    # write_loss_and_gradient(filename, stiffness_min=1.0, stiffness_max=20.0, n_points=91)
    # print_dydf_fd(hs=[0.1, 0.01, 0.001, 0.0001])
    # write_error_vs_target("outputs/error_vs_target.npy")

    u = np.zeros((8, 3))
    # u[4:, 2] = 0.2
    # u[4:] = 0.2 * np.array([
    #     [-0.5, -0.5, +0.5],
    #     [+0.5, -0.5, +0.5],
    #     [-0.5, +0.5, +0.5],
    #     [+0.5, +0.5, +0.5],
    # ])
    u[4:] += np.array([
        [ 0.03546312, -0.08985449, -0.00126088],
        [ 0.03438233,  0.03236226, -0.08005212],
        [ 0.01568617,  0.09636057, -0.05149985],
        [-0.07768177,  0.00867797,  0.09879704]
    ])
    jac_fd = get_jacobian_fd(u, 0.0001)
    jac_fd = 0.5*(jac_fd + jac_fd.T)
    jac_sd = run_simulation(Inputs(u=u, jac_file="tmp_", newton_iterations=1)).jacobian[12:, 12:]
    print_matrix(jac_sd, "SOFA")
    print_matrix(jac_fd, "SOFA FD")
    # print_matrix(jac_fd - jac_sd, "Difference")
    # print_matrix((jac_fd / jac_sd - 1) * 100, "Ratio")
    # print_matrix(np.isclose(jac_fd, jac_fd.T), "Symetry of FD")
    # test_jacobian_convergence(u)

if __name__ == "__main__":
    main()