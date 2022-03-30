import numpy as np


class Dynamics:

    """This is the Dynamics class
       It contains various solvers
       It is also the base class for any physical object """

    step_size = 0.01      # The size of simultaion step
    initial_time = 0      # Initial time
    final_time = 20       # Final time

    global_time = 0


    def __init__(self, init_state, extra_data = None):
        """Initializes all the states"""
        self.init_state = init_state   # Initial state of the system
        self.time = self.global_time   # Variable storing the current time
        self.current_state = np.array(self.init_state)  # Current state of
                                                        # the system
        self.sys_dim = np.size(self.init_state)  # No. of states
        self.no_iter = int((self.final_time-self.initial_time)  # Calculate th
                           /self.step_size)                     # itereations
        self.states = np.zeros([self.sys_dim, self.no_iter]) # For storing
                                                             # Value of state
        self.extra_data = extra_data

    def solver(self, dyn, leaders=None, followers=None):
        """
            Defines the algorithm which predicts the next 
            state of the system based on current state
            and input calculated by the function input_sys. Note that the 
            function input_sys will be defined in the child class. In this
            solver interface we have used a Runge-Kutta Solver.
        """
        x = self.current_state   # Current state 
        u = self.input_sys(leaders, followers) # Input based on the curr. state
        t = self.time  # Current time

        # Updating the state
        k1 = self.step_size * dyn(x, t, u)
        k2 = self.step_size * dyn(x+k1/2, t+self.step_size/2, u)
        k3 = self.step_size * dyn(x+k2/2, t+self.step_size/2, u)
        k4 = self.step_size * dyn(x+k3, t+self.step_size, u)
        self.current_state += 1 / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        self.time += self.step_size

    def update_state(self, observed_state=None, leaders=None, followers=None):
        self.observed_state = np.array(observed_state)
        self.solver(self.dynamics, leaders, followers)
        # self.dynamics is a virtual function defined in the derived class
        return self.current_state
