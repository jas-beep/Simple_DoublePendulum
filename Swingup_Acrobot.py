import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    LeafSystem,
    Linearize,
    Saturation,
    LinearQuadraticRegulator,
    plot_system_graphviz,
    LogVectorOutput,
    PidController,
    MeshcatVisualizer,
    ModelVisualizer,
    Parser,
    Simulator,
    StartMeshcat,
    RegionOfAttraction
)
from pydrake.examples import AcrobotGeometry, AcrobotPlant, AcrobotParameters, AcrobotState, AcrobotInput, AcrobotSpongController

meshcat=StartMeshcat() #visualizing outside of notebook
print(f"Open in interactive window: {meshcat.web_url()}")

xequilibrium = np.array([np.pi, 0, 0.0, 0.0])  # upright Equilibrium state: [theta1, theta2, theta1_dot, theta2_dot]


class EnergyShapingController(LeafSystem):
    def __init__(self, acrobot):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("state", 4)
        self.DeclareVectorOutputPort("control", 1, self.CalcOutput)
        self.acrobot = acrobot
        self.acrobot_context = acrobot.CreateDefaultContext()
        self.SetAcrobotParameters(AcrobotParameters())

    def SetAcrobotParameters(self, params):
        self.acrobot_context.get_mutable_numeric_parameter(0).SetFromVector(params.CopyToVector())
        self.acrobot_context.set_continuous_state([np.pi, 0, 0, 0])
        self.desired_energy = self.acrobot.get

    def CalcOutput(self, context, output):
        acrobot_state =  self.get_input_port(0).Eval(context)
        self.acrobot_context.SetContinuousState(acrobot_state)
        params = self.acrobot_context.get_numeric_parameter(0)
        theta2dot = acrobot_state[3]
        total_energy = (self.acrobot.EvaluateKineticEnergy(self.acrobot_context) +
                        self.acrobot.EvaluatePotentialEnergy(self.acrobot_context))
        output.SetAtIndex(()) # add swing up controller

class SwingUpAndBalanceController(LeafSystem):
    def __init__(self, acrobot):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("state", 4)
        self.DeclareVectorOutputPort("control", 1, self.CalcOutput)
        self.K, self.S = BalancingLQR(acrobot)
        self.energy_shaping = EnergyShapingController(acrobot)
        self.saturation = Saturation(-10, 10)

def BalancingLQR(acrobot):
    acrobot_context = acrobot.CreateDefaultContext()
    acrobot.get_actuation_input_port().FixValue(acrobot_context, np.zeros(1))
    acrobot_context.set_continuous_state(xequilibrium)

    Q = np.diag([10, 10, 1, 1])
    R = np.eye([[1]])

    Linearsystem = Linearize(acrobot, acrobot_context)
    (K, S) = LinearQuadraticRegulator(Linearsystem.A(), Linearsystem.B(), Q, R)
    return K, S
