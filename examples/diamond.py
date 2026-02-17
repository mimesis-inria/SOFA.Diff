def createScene(root):

    with root.addChild("Plugins") as plugins:
        plugins.addObject("RequiredPlugin", pluginName=[
            "SofaDiff",  # Needed to use components [TrainableParameterVector, ParameterizedSpringForceField, LossState]
            "Sofa.Component.IO.Mesh",  # Needed to use components [MeshVTKLoader]
            "Sofa.Component.LinearSolver.Direct",  # Needed to use components [SparseLDLSolver]
            "Sofa.Component.LinearSolver.Iterative",  # Needed to use components [CGLinearSolver,ShewchukPCGLinearSolver]
            "Sofa.Component.ODESolver.Backward",  # Needed to use components [EulerImplicitSolver, NewtonRaphsonSolver, StaticSolver]
            "Sofa.Component.StateContainer",  # Needed to use components [MechanicalObject]
            "Sofa.Component.Topology.Container.Constant",  # Needed to use components [MeshTopology]
            "Sofa.Component.Visual",  # Needed to use components [VisualStyle]
            "Sofa.Component.Engine.Select",  # Needed to use components [BoxROI]
            "Sofa.Component.Mapping.Linear",  # Needed to use components [BarycentricMapping]
            "Sofa.Component.Mapping.NonLinear",  # Needed to use components [DistanceFromTargeMapping, SquareMapping]
            "Sofa.Component.Mass",  # Needed to use components [UniformMass]
            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components [TetrahedronFEMForceField]
            "Sofa.Component.Constraint.Projective",  # Needed to use components [FixedProjectiveConstraint]
            "Sofa.Component.MechanicalLoad"  # Needed to use components [ConstantForceField]
        ])

    root.findData("dt").value=1
    root.findData("gravity").value=[0, 0, -9810]

    root.addObject("VisualStyle", displayFlags="showCollision showVisualModels showForceFields showInteractionForceFields")
    root.addObject("DefaultVisualManagerLoop")
    root.addObject("DifferentiableAnimationLoop", computeBoundingBox=False)

    with root.addChild("Parameters") as parameters:
        # length = 88.28363382  # length at rest configuration
        length2 = 97.52237356  # length of the solution with no springs
        parameters.addObject("TrainableParameterVector", name="springLengths", value=[length2/2]*4, learningRate=0.5-0.5)
        parameters.addObject("TrainableParameterVector", name="springStiffness", value=[1, 1, 1, 1], learningRate=0.1)

    with root.addChild("PullPoints") as pull_points:
        pull_points.addObject("MechanicalObject", position=[[0, 10, 30], [-10, 0, 30], [0, -10, 30], [10, 0, 30]])

    with root.addChild("Robot") as robot:
        robot.addObject("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton="100", maxNbIterationsLineSearch="1", warnWhenLineSearchFails="false")
        robot.addObject("StaticSolver", newtonSolver="@newton", name="static")
        robot.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixd", name="solver")

        robot.addObject("MeshVTKLoader", name="loader", filename="meshes/diamond.vtk")
        robot.addObject("MeshTopology", src="@loader")
        robot.addObject("MechanicalObject", showIndicesScale=4e-5, rx=90, dz=35)
        robot.addObject("UniformMass", totalMass=0.5)
        robot.addObject("TetrahedronFEMForceField", youngModulus=180, poissonRatio=0.45)
        robot.addObject("BoxROI", box=[-15, -15, -40, 15, 15, 10], drawBoxes=True)
        robot.addObject("FixedProjectiveConstraint", indices="@BoxROI.indices")

        with robot.addChild("controlledPoints") as controlled_points:
            visual_kwargs = dict()  # dict(showObject="true", drawMode="1", showObjectScale="5", showColor="1 0 0 1")
            controlled_points.addObject(
                "MechanicalObject", name="state",
                position=[[0, 97, 45], [-97, 0, 45], [0, -97, 45], [97, 0, 45], [0, 0, 115]],
                **visual_kwargs
            )
            controlled_points.addObject("BarycentricMapping")

        robot.addObject("ConstantForceField", template="Vec3d", indices=456, forces="0 0 0")

        with robot.addChild("Marker") as marker:
            visual_kwargs = dict(showObject="true", drawMode="1", showObjectScale="1", showColor="1 0 1 1")
            marker.addObject("MechanicalObject", name="state", position=[[0, 0, 125]], **visual_kwargs)
            marker.addObject("BarycentricMapping", mapForces=False, mapMasses=False)

        robot.addObject(
            "ParameterizedSpringForceField",
            object1="@/PullPoints", object2="@controlledPoints",
            indices1=[0, 1, 2, 3], indices2=[0, 1, 2, 3],
            stiffness="@/Parameters/springStiffness.value",
            length="@/Parameters/springLengths.value",
            damping=[0]*4, elongationOnly=[0]*4, enabled=[1]*4,
            drawMode=1, showArrowSize=1,
        )

    with root.addChild("Goal") as goal:
        visual_kwargs = dict(showObject="true", drawMode="1", showObjectScale="1", showColor="1 0.2 0 1")
        goal.addObject("MechanicalObject", name="state", position=[[15, 10, 95]], **visual_kwargs)
        # Before: 0, 0, 125

    with root.addChild("Loss") as loss:
        with loss.addChild("Distance") as distance:
            distance.addObject("MechanicalObject", template="Vec1d", name="state", position=0)
            distance.addObject("DistanceFromTargetMapping", input="@/Robot/Marker", targetPositions="@/Goal/state.position")
            with distance.addChild("MSE") as mse:
                mse.addObject("LossState", name="state")
                mse.addObject("MeanSquaredErrorMapping")

    root.addObject("GradientDescentController", name="optimizer")

    return root

