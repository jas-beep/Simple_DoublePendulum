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
    RegionOfAttraction,
    SceneGraph
)
from pydrake.examples import AcrobotGeometry, AcrobotPlant, AcrobotParams, AcrobotState, AcrobotInput, AcrobotSpongController

meshcat=StartMeshcat() #visualizing outside of notebook
print(f"Open in interactive window: {meshcat.web_url()}")

xequilibrium = np.array([np.pi, 0, 0.0, 0.0])  # upright Equilibrium state: [theta1, theta2, theta1_dot, theta2_dot]


def Acrobot_SwingUp_Balance():
    builder = DiagramBuilder()
    acrobot = builder.AddSystem(AcrobotPlant())
    acrobot_context = acrobot.CreateDefaultContext()
    scene_graph = builder.AddSystem(SceneGraph())
    acrobot.set_name("Acrobot")

    AcrobotGeometry.AddToBuilder(builder, acrobot.get_output_port(0), scene_graph)

    SpongSwingUp = builder.AddSystem(AcrobotSpongController())

    # can adjust spong paramters with 
    # spong_context = SpongSwingUp.CreateDefaultContext()
    # spong_params = SpongSwingUp.get_mutable_parameters(spong_context)
    # params.set_k_e....etc 
    # SpongSwingUp.set_default_parameters(params)

    #now just connect everything
    builder.Connect(acrobot.get_output_port(0), SpongSwingUp.get_input_port(0))
    builder.Connect(SpongSwingUp.get_output_port(0), acrobot.get_input_port(0))

    actuation_logger = LogVectorOutput(SpongSwingUp.get_output_port(), builder)
    state_logger = LogVectorOutput(acrobot.get_output_port(), builder)
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    visualizer.set_name("visualizer")

    diagram = builder.Build()
    return diagram, actuation_logger, state_logger

def simulate_and_animate(diagram, xstart, alogger, slogger, sim_time=5.0, visualize=True):
    simulator = Simulator(diagram)
    simulator.set_publish_every_time_step(False)
    simulator.get_mutable_integrator().set_fixed_step_mode(True)

    visualizer=diagram.GetSubsystemByName("visualizer")
    visualizer.StartRecording(False)

    if len(xstart) != diagram.GetSubsystemByName("Acrobot").num_continuous_states():
        print (f"Your plant doesn't have {len(xstart)} state variables.")
        return  
    
    # reset initial conditions
    context = simulator.get_mutable_context()
    context.SetTime(0.0)
    context.SetContinuousState(xstart)

    # run sim
    simulator.Initialize()
    simulator.AdvanceTo(sim_time)
    
    # stop video
    visualizer.PublishRecording()
    visualizer.DeleteRecording()

    # access logged data
    alog = alogger.FindLog(context)
    slog = slogger.FindLog(context)
    return alog, slog


xstart = np.array([np.pi, -np.pi/2, 0.0, 0.0])  # Initial state: [theta1, theta2, theta1_dot, theta2_dot]
diagram, actuation_logger, state_logger = Acrobot_SwingUp_Balance()
alog, slog = simulate_and_animate(diagram, xstart, actuation_logger, state_logger, sim_time=10.0, visualize=True)