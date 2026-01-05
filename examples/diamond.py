def createScene(rootNode):

    settings = rootNode.addChild("Settings")
    settings.addObject('RequiredPlugin', pluginName=[
        'SofaDiff',  # Needed to use components [TrainableParameterVector, ParameterizedSpringForceField, LossState]
        'Sofa.Component.IO.Mesh',  # Needed to use components [MeshVTKLoader]
        'Sofa.Component.LinearSolver.Direct',  # Needed to use components [SparseLDLSolver]
        'Sofa.Component.LinearSolver.Iterative',  # Needed to use components [CGLinearSolver,ShewchukPCGLinearSolver]
        'Sofa.Component.ODESolver.Backward',  # Needed to use components [EulerImplicitSolver, NewtonRaphsonSolver, StaticSolver]
        'Sofa.Component.StateContainer',  # Needed to use components [MechanicalObject]
        'Sofa.Component.Topology.Container.Constant',  # Needed to use components [MeshTopology]
        'Sofa.Component.Visual',  # Needed to use components [VisualStyle]
        'Sofa.Component.Engine.Select',  # Needed to use components [BoxROI]
        'Sofa.Component.Mapping.Linear',  # Needed to use components [BarycentricMapping]
        'Sofa.Component.Mapping.NonLinear',  # Needed to use components [DistanceFromTargeMapping, SquareMapping]
        'Sofa.Component.Mass',  # Needed to use components [UniformMass]
        'Sofa.Component.SolidMechanics.FEM.Elastic',  # Needed to use components [TetrahedronFEMForceField]
        'Sofa.Component.Constraint.Projective',  # Needed to use components [FixedProjectiveConstraint]
    ])

    rootNode.findData('dt').value=1
    rootNode.findData('gravity').value=[0, 0, -9810]
    rootNode.addObject('VisualStyle', displayFlags='showCollision showVisualModels showForceFields showInteractionForceFields')
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('DefaultAnimationLoop', computeBoundingBox=False)

    #parameters
    parametersNode = rootNode.addChild("Parameters")
    length = 88.28363382  # length at rest configuration
    length2 = 97.52237356  # length of the solution with no springs
    parametersNode.addObject("TrainableParameterVector", name="springLengths", value=[length2/2]*4, learningRate=0.5-0.5)
    parametersNode.addObject("TrainableParameterVector", name="springStiffness", value=[1]*4, learningRate=0.1)

    #pull points
    pullPointsNode = rootNode.addChild("PullPoints")
    pullPointsNode.addObject("MechanicalObject", position=[[0, 10, 30], [-10, 0, 30], [0, -10, 30], [10, 0, 30]])

    #goal
    goal = rootNode.addChild('goal')
    # Before: 0, 0, 125
    goal.addObject('MechanicalObject', name='goalMO', position=[[15, 10, 95]])#, showObject="true", drawMode="1", showObjectScale="1", showColor="1 0.2 0 1")

    #robot
    robot = rootNode.addChild('robot')
    # robot.addObject('EulerImplicitSolver')
    robot.addObject("NewtonRaphsonSolver", name="newton", maxNbIterationsNewton="100", maxNbIterationsLineSearch="1", warnWhenLineSearchFails="false")
    robot.addObject("StaticSolver", newtonSolver="@newton", name="static")
    robot.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixd", name="solver")

    robot.addObject('MeshVTKLoader', name="loader", filename="meshes/diamond.vtk")
    robot.addObject('MeshTopology', src="@loader")
    robot.addObject('MechanicalObject', showIndicesScale=4e-5, rx=90, dz=35)
    robot.addObject('UniformMass', totalMass=0.5)
    robot.addObject('TetrahedronFEMForceField', youngModulus=180, poissonRatio=0.45)
    robot.addObject('BoxROI', box=[-15, -15, -40, 15, 15, 10], drawBoxes=True)
    robot.addObject('FixedProjectiveConstraint', indices="@BoxROI.indices")

    #robot/controlledPoints
    controlledPoints = robot.addChild('controlledPoints')
    controlledPoints.addObject('MechanicalObject', name="actuatedPoints",
                               position=[[0, 97, 45],
                                         [-97, 0, 45],
                                         [0, -97, 45],
                                         [97, 0, 45],
                                         [0, 0, 115]],
                               # showObject="true", drawMode="1", showObjectScale="5", showColor="1 0 0 1"
                               )
    controlledPoints.addObject('BarycentricMapping')

    targetNode = robot.addChild("Target")
    targetNode.addObject("MechanicalObject", name="Target", position=[[0, 0, 125]], showObject="true", drawMode="1", showObjectScale="10", showColor="1 0 0 1")
    targetNode.addObject("BarycentricMapping", mapForces=False, mapMasses=False)

    robot.addObject(
        "ParameterizedSpringForceField",
        object1="@/PullPoints", object2="@controlledPoints",
        indices1=[0, 1, 2, 3], indices2=[0, 1, 2, 3],
        stiffness="@/Parameters/springStiffness.value",
        length="@/Parameters/springLengths.value",
        damping=[0, 0, 0, 0],
        drawMode=1, showArrowSize=1,
    )

    lossNode = rootNode.addChild("Loss")
    lossNode.addObject("LossState", name="MSE")
    distanceNode = lossNode.addChild("Distance")
    distanceNode.addObject("MechanicalObject", template="Vec1d", name="output", position=0)
    distanceNode.addObject("DistanceFromTargetMapping", input="@/robot/Target", targetPositions="@/goal/goalMO.position", output="@/Loss/Distance/output")
    squaredNode = distanceNode.addChild("Squared")
    squaredNode.addObject("SquareMapping", input="@..", output="@/Loss/MSE")

    rootNode.addObject("GradientDescentController", name="optimizer")

    return rootNode

