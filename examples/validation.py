"""
Tools to validate the gradients computed by SOFA.Diff.

Relies on finite differences estimations of the gradients to check the
convergence towards the gradients computed by SOFA.Diff.
"""
import Sofa
import numpy as np


def validate_parameter_gradient(createScene, parameter_path, loss_path, parameter_value=None, verbose=True):
    """
    Validate the gradient of the loss with respect to the parameters.

    Requires the createScene function, as well as the paths to the parameter and to the loss.
    """
    def run(parameter=None):
        # Init scene
        root = Sofa.Core.Node("root")
        createScene(root)
        Sofa.Simulation.initRoot(root)
        # Set parameter value
        if parameter is None:
            parameter = root[parameter_path].value.getValue()
        root[parameter_path].value.setValue(np.array(parameter).tolist())
        parameter = root[parameter_path].value.getValue()
        # Run scene and return stuff
        Sofa.Simulation.animate(root, root.dt.value)
        loss = root[loss_path].value.getValue()
        gradient = root[parameter_path].gradient.getValue()
        return parameter, loss[0][0], gradient

    # SOFA.Diff
    parameter_mid, loss_mid, gradient_mid = run(parameter_value)
    n_params = len(parameter_mid)

    # Finite differences
    n = 8
    epsilon = 1 / 10**np.arange(n)
    parameter = [None] * n
    loss = [None] * n
    gradient = [None] * n
    residual = [None] * n

    for i, eps in enumerate(epsilon):
        gradient[i] = np.zeros(n_params)
        for j in range(n_params):
            p, loss_j, _ = run(parameter_mid * (1 + eps * (np.arange(n_params) == j)))
            gradient[i][j] = (loss_j - loss_mid) / (p[j] - parameter_mid[j])
        parameter[i], loss[i], _ = run(parameter_mid * (1 + eps))
        residual[i] = loss[i] - loss_mid - np.dot(gradient_mid, (parameter[i] - parameter_mid))

    # Convergence analysis
    hs = np.array(epsilon)
    residuals = np.array(residual)
    rates = np.log10(residuals[1:]/residuals[:-1]) / np.log10(hs[1:]/hs[:-1])
    rates = np.concat([[0], rates])

    # Display results
    if verbose:
        for i in range(n):
            print(f"{f'RUN {i} ':=^40}")
            print("    epsilon       ", epsilon[i])
            print("    parameter     ", parameter[i])
            print("    parameter ref ", parameter_mid)
            print("    loss          ", loss[i])
            print("    loss ref      ", loss_mid)
            print("    gradient      ", gradient[i])
            print("    gradient ref  ", gradient_mid)
            print("    residual      ", residual[i])
            print()

        print(f"{f'CONVERGENCE ANALYSIS ':=^40}")
        print("    ———————————————————————————————")
        print("    |   eps   | residual |  rate  |")
        print("    ———————————————————————————————")
        for h, residual, rate in zip(hs, residuals, rates):
            print(f"    | {h:.1e} | {residual:8.1e} | {rate:6.2f} |")


def validate_force_gradient(createScene, solver_node_path, loss_path, verbose=True):
    def get_force_gradient():
        # Run scene
        root = Sofa.Core.Node("root")
        createScene(root)
        Sofa.Simulation.initRoot(root)
        Sofa.Simulation.animate(root, root.dt.value)
        # Return force gradient
        mo = root[solver_node_path].getMechanicalState()
        return mo.forceGradient.getValue()

    def get_loss(forces=None):
        # Init scene
        root = Sofa.Core.Node("root")
        createScene(root)
        solver_node = root[solver_node_path]
        if forces is not None:
            solver_node.addObject("ConstantForceField", name="lilPush", forces=forces)
        Sofa.Simulation.initRoot(root)
        # Run scene and return stuff
        Sofa.Simulation.animate(root, root.dt.value)
        loss = root[loss_path].value.getValue()
        return loss[0][0]

    force_gradient = get_force_gradient()
    force_shape = force_gradient.shape

    loss_mid = get_loss()

    n = 10
    epsilon = 1 / 2**np.arange(n)
    residual = [None] * n
    for i_eps, eps in enumerate(epsilon):
        rows, cols = force_shape
        gradient_fd = np.zeros(force_shape)
        for i in range(rows):
            for j in range(cols):
                forces = np.zeros((rows, cols))
                forces[i, j] = eps
                loss_eps = get_loss(forces)
                gradient_fd[i, j] = (loss_eps - loss_mid) / eps
        residual[i_eps] = np.linalg.norm(force_gradient - gradient_fd)

    # Convergence analysis
    hs = np.array(epsilon)
    residuals = np.array(residual)
    rates = np.log10(residuals[1:]/residuals[:-1]) / np.log10(hs[1:]/hs[:-1])
    rates = np.concat([[0], rates])

    if verbose:
        print(f"{f'CONVERGENCE ANALYSIS ':=^40}")
        print("    ———————————————————————————————")
        print("    |   eps   | residual |  rate  |")
        print("    ———————————————————————————————")
        for h, residual, rate in zip(hs, residuals, rates):
            print(f"    | {h:.1e} | {residual:8.1e} | {rate:6.2f} |")
