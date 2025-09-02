import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
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


meshcat=StartMeshcat() #visualizing outside of notebook
print(f"Open the following URL in your browser to see the visualizer: {meshcat.web_url()}")

builder = DiagramBuilder()
acrobot, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
Parser(acrobot).AddModels("Acrobot.urdf")
acrobot.Finalize()
acrobot.set_name("Acrobot")


xequilibrium = np.array([np.pi, 0, 0.0, 0.0])  # upright Equilibrium state: [theta1, theta2, theta1_dot, theta2_dot]

def acrobot_balancing(xequilibrium, Q, R):
    context = acrobot.CreateDefaultContext()
    context.get_mutable_continuous_state_vector().SetFromVector(xequilibrium)
    acrobot.get_actuation_input_port().FixValue(context, np.zeros(1))  # Fix the input to zero for now, based on # of joints?

    lqr = LinearQuadraticRegulator(
        acrobot,
        context,
        Q,
        R,
        input_port_index=acrobot.get_actuation_input_port().get_index(), 
    )

    lqr = builder.AddSystem(lqr)
    lqr.set_name("LQR")

    builder.Connect(acrobot.get_state_output_port(), lqr.get_input_port(0)) 
    builder.Connect(lqr.get_output_port(0), acrobot.get_actuation_input_port()) # nan atr actuation input port?

    actuation_logger = LogVectorOutput(lqr.get_output_port(), builder)
    state_logger = LogVectorOutput(acrobot.get_state_output_port(), builder)
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    visualizer.set_name("visualizer2")

    diagram = builder.Build()
    return diagram, context, actuation_logger, state_logger 


def simulate_and_animate(diagram, xstart, alogger, slogger, sim_time=5.0, visualize=True):
    simulator = Simulator(diagram)
    simulator.set_publish_every_time_step(False)
    simulator.get_mutable_integrator().set_fixed_step_mode(True) #RK3 by default

    visualizer=diagram.GetSubsystemByName("visualizer2")
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

# LQR for double pendulum balancing
Q = np.diag([10.0, 10.0, 1, 1])  # State cost matrix
R = np.eye(1)
xstart = np.array([0.99*np.pi, 0.0, 0.0, 0.0])  # Initial state: [theta1, theta2, theta1_dot, theta2_dot], LQR can only handle small deviations from equilibrium
diagram, context, actuation_logger, state_logger = acrobot_balancing(xequilibrium, Q, R)


a_log, s_log = simulate_and_animate(diagram, xstart, actuation_logger, state_logger, sim_time=6.0, visualize=True)
#log.sample_times()
u=a_log.data()
x=s_log.data()
print(u.shape)
print(x.shape)
for i in range(10):
    print(u[:,i])
    #print(x[:,-i])
print(x[:,-1]) #initial state

# roa = RegionOfAttraction(acrobot, context)
# print(f"Estimated ROA volume: {roa.volume():.4f}") 
# non-unique solution gives error, study Lyapunov functions and ROA before attempting this again

# For Visualization you need to run in interatcive window...

