import numpy as np
import matplotlib.pyplot as plt
from simulink import Dynamics


r = 1.5  # Communication Radius


class SimpleFollower(Dynamics):

    """This is an agent whose dynamics represent  that of simple agent and 
    is a follower"""

    followers_count = 0  # Global varibale: Total number of followers till now

    def __init__(self, init_state):
        """TODO: Define vehicle id, depth flag, leader flag, converge flag 
                 and increase the followers count         
        """
        Dynamics.__init__(self, init_state)

        self.veh_id = -1           # The vehicle which the agent has decided to follow
        self.depth_flag = np.nan   # Depth of the agent from  
                                   # a particular leader
        self.leader_flag = -1      # The leader which is being followed            
        self.converge_flag = False
        self.identity = self.followers_count   # The identity of the given follower 
        SimpleFollower.followers_count += 1

        self.leader_check = 0   # To make sure that the leaders are checked only once
        self.followers_check = 0  # Also followers are checked only once
        self.leader_found = 0   # Set to one if any agent can directly see a
                                # leader
        self.neighbors_leaders = []   # Adds the neighbors in the one abve level
        self.neighbors_followers = []   # Adds the neighbors in the one abve level

    def leader_selection(self, leaders, followers):
        # Select the leaders which are in communication range
        if self.leader_check == 0:
            dist_lead = [np.linalg.norm(l.current_state -
                                   self.current_state) for l in leaders]

            for i in range(0, len(leaders)):
                if dist_lead[i] <= r:
                    self.leader_flag = leaders[i].identity 
                    self.depth_flag = 1
                    self.neighbors_leaders.append(leaders[i])
                    self.leader_found = 1

            self.leader_check = 1 # Ensures that leader checking is done only once
        

        # If no leaders are in the vicinity then select from the followers which are a level above
        dist_followers = [np.linalg.norm(followers[i].current_state -
                                   self.current_state) for i in range(0, len(followers))]

        for i  in range(0, len(followers)):
            if dist_followers[i] <= r and followers[i].leader_flag != -1 and \
                    self.identity != i and len(self.neighbors_leaders) == 0:
                self.neighbors_followers.append(followers[i])

        mini = np.inf
        for i in range(0, len(self.neighbors_followers)):
            if self.neighbors_followers[i].depth_flag < mini:
                mini = self.neighbors_followers[i].depth_flag 
        self.neighbors_followers = [l for l in self.neighbors_followers if l.depth_flag == mini]   
        if len(self.neighbors_followers) > 0:
            if self.leader_found == 0:
                self.depth_flag = mini + 1

            self.leader_flag = self.neighbors_followers[0].leader_flag
            for l in self.neighbors_followers:
                if l.leader_flag != self.leader_flag:
                    self.leader_flag = 0
                    break

            if self.depth_flag != np.nan and self.followers_check == 0:
                self.followers_check = 1



        # Resolve the conflicts
        if self.depth_flag == 1 and self.leader_flag == 0 and \
                len(self.neighbors_leaders) > 0 :
            bl = dict([(l.identity,dist_lead[l.identity-1]) for l in self.neighbors_leaders])
            leader_id = min(bl, key=bl.get)
            self.leader_flag = leader_id
            
        if self.depth_flag != np.nan and self.depth_flag != 1 and \
                self.leader_flag == 0 and len(self.neighbors_followers) > 0:
            bf = {f.identity:f.leader_flag
                    for f in self.neighbors_followers}

            distinct_values = set(values for values in bf.values())
            # print(distinct_values)

            count_dict = {len([k for k,v in bf.items() if v == values]):\
                          values for values in distinct_values}
                
            self.leader_flag = count_dict.get(min(count_dict))


    def observed_identifier(self, leaders, followers):
        """This method sets all the flags"""
        # Calculate the distance of the follower from its choosen leader
        dist_lead = np.linalg.norm(leaders[self.leader_flag-1].current_state -
                                   self.current_state) 

        if dist_lead < r:
            self.depth_flag = 1
            self.veh_id = self.leader_flag
        else:
            dist_prev = np.infty
            prev_flag = np.infty
            for i in range(0, len(self.neighbors_followers)):
                dist = np.linalg.norm(self.current_state -
                                      self.neighbors_followers[i].current_state)
                if dist < r and dist < dist_prev and \
                        self.neighbors_followers[i].depth_flag < self.depth_flag \
                        and self.identity != self.neighbors_followers[i].identity and \
                        self.neighbors_followers[i].depth_flag <= prev_flag:
                    self.veh_id = self.neighbors_followers[i].identity
                    self.depth_flag = self.neighbors_followers[i].depth_flag + 1
                    dist_prev = dist
                    prev_flag = self.neighbors_followers[i].depth_flag

    def input_sys(self, leaders, followers):
        xn = self.current_state

        if self.depth_flag == 1:
            # print(self.veh_id)
            xo = leaders[self.veh_id-1].current_state
        else:
            xo = followers[self.veh_id].current_state
            # print(xo)

        lead_dist = np.linalg.norm(leaders[self.leader_flag-1].current_state
                                   - self.current_state)
        if lead_dist <= .2:
            self.converge_flag = True

        u = np.arctan2(xo[1]-xn[1], xo[0]-xn[0])
        return u

    def dynamics(self, x, t, u):
        x1 = 0.6*np.cos(u)
        y1 = 0.6*np.sin(u)
        f_x = np.array([x1, y1])
        return f_x

    def select_leader(self):
        pass


class SimpleLeader(Dynamics):

    """
    This describes dynamics and input function for leader agent.
    The dynamics are that of simple motion.
    """
    leader_count = 0

    def __init__(self, init_state, extra_data):
        """TODO: to be defined1. """
        Dynamics.__init__(self, init_state, extra_data)
        self.depth_flag = 0
        self.identity = SimpleLeader.leader_count + 1
        SimpleLeader.leader_count += 1

    def input_sys(self, leaders, followers):
        xn = self.current_state
        # xo = np.array(followers[3].current_state)
        # xo = np.array([0, 100])
        xo = np.array(self.extra_data)
        u = np.arctan2(xo[1]-xn[1], xo[0]-xn[0])
        return u

    def dynamics(self, x, t, u):
        x1 = 0.2*np.cos(u)
        y1 = 0.2*np.sin(u)
        f_x = np.array([x1, y1])
        return f_x


Dynamics.initial_time = 0
Dynamics.final_time = 30
Dynamics.step_size = 0.01


# Create agents with initial Conditions
# followers = []
# followers.append(SimpleFollower([0., 0]))
# followers.append(SimpleFollower([0.66, 0.75]))
# followers.append(SimpleFollower([0.66, -0.75]))
# followers.append(SimpleFollower([-1., 0]))

# leaders = []
# leaders.append(SimpleLeader([1.32, 0], [100., 0]))
# leaders.append(SimpleLeader([-1.9, 0], [-100., 0]))

# # Create agents with initial Conditions
followers = []
followers.append(SimpleFollower([1., 0]))
followers.append(SimpleFollower([1., 1]))
followers.append(SimpleFollower([0., 1]))
followers.append(SimpleFollower([2., 0.3]))
followers.append(SimpleFollower([2, 1.4]))
followers.append(SimpleFollower([0.3, 1.8]))
followers.append(SimpleFollower([1.6, 2.6]))
followers.append(SimpleFollower([3., 1.]))
# followers.append(SimpleFollower([3., 2.5]))

leaders = []
leaders.append(SimpleLeader([0., 0], [-40., -40]))
leaders.append(SimpleLeader([3., 2.5], [100.,100]))
# leaders.append(SimpleLeader([-2., 0], [-100., 0]))
# leaders.append(SimpleLeader([4., 1.], [0., 100]))

# Some initial Settings
no_iter = leaders[0].no_iter
t = np.zeros(no_iter)
t0 = 0

for k in range(0, SimpleFollower.followers_count):
    for l in range(0, SimpleFollower.followers_count):
        for m in range(0, SimpleFollower.followers_count):
            followers[m].leader_selection(leaders, followers)

for l in range(0, SimpleFollower.followers_count):
    for m in range(0, SimpleFollower.followers_count):
        followers[m].observed_identifier(leaders, followers)

for l in range(0, SimpleFollower.followers_count):
    print(followers[l].leader_flag, followers[l].depth_flag,
          followers[l].veh_id)


# Run the Simulation
for i in range(0, no_iter):
    # for l in range(0, SimpleFollower.followers_count):
    #     for m in range(0, SimpleFollower.followers_count):
    #         followers[m].observed_identifier(leaders, followers)
    # Updating the states being observed
    for l in range(0, len(leaders)):
        leaders[l].states[:, i] = leaders[l].update_state(leaders=leaders,
                                                      followers=followers)
    for j in range(0, SimpleFollower.followers_count):
        followers[j].states[:, i] = followers[j].update_state(leaders=leaders,
                                                              followers=
                                                              followers)
    converge = True
    for j in range(0, SimpleFollower.followers_count):
        converge = converge and followers[j].converge_flag

    if converge:
        break

    t[i] = t0
    # Connecting the edges
    t0 = t0 + Dynamics.step_size
    new_iter = i

# ipdb.set_trace()
# Plotting the results
# Plot of x-y
plt.figure(1)
for j in range(0, SimpleFollower.followers_count):
    plt.plot(followers[j].init_state[0], followers[j].init_state[1], marker = 'o')
    plt.plot(followers[j].states[0, 0:new_iter],
            followers[j].states[1, 0:new_iter], linestyle=':', linewidth=2.0,
             label='a'+str(j))
for j in range(0, SimpleLeader.leader_count):
    plt.plot(leaders[j].init_state[0], leaders[j].init_state[1], marker = 'v')
    plt.plot(leaders[j].states[0, 0:new_iter], leaders[j].states[1, 0:new_iter],\
         linestyle='-', linewidth=2.0, label='leader'+str(j))
plt.grid()
plt.xlim((-5, 5))
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('x-y graph for all agents')
plt.legend(loc='upper left', frameon=False)
plt.axis('equal')
# plt.savefig('/home/aditya/Dropbox/Multiple_Leaders_2018/Multiple_Lyx/xy_plot_multiple_leaders.png')
plt.show()
print(t0)
