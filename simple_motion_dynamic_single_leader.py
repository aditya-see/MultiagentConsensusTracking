from simulink import Dynamics
import numpy as np
import matplotlib.pyplot as plt


r = 6 # Communication Radius

class SimpleFollower(Dynamics):

    """Docstring for Simple"""
    followers_count = 0

    def __init__(self, init_state):
        """TODO: to be defined1. """
        Dynamics.__init__(self, init_state)
        self.depth_flag = np.nan
        self.identity = SimpleFollower.followers_count
        self.converge_flag = False
        SimpleFollower.followers_count += 1
        self.veh_id = 0

    def observed_identifier(self, leaders, followers):
#        if self.depth_flag == np.nan:
        dist_lead = np.linalg.norm(leaders[0].current_state -
                                   self.current_state)
#        print(dist_lead,"Hello")
        if dist_lead < r:
            self.depth_flag = 1
            self.veh_id = -1
        else:
#            print("Its false")
            dist_prev = np.infty
            prev_flag = np.nan
            for i in range(0, len(followers)):
                dist = np.linalg.norm(self.current_state -
                                      followers[i].current_state)
                if dist < r and dist < dist_prev and \
                   followers[i].depth_flag != np.nan and self.depth_flag != 1 \
                   and self.identity != i and followers[i].depth_flag <= \
                   prev_flag:
#                    print("Hi I am here")
                    self.veh_id = i
                    self.depth_flag = followers[i].depth_flag + 1
                    dist_prev = dist
                    prev_flag = followers[i].depth_flag

    def input_sys(self, leaders, followers):
        xn = self.current_state

        if self.veh_id == -1:
            xo = leaders[0].current_state
        else:
            xo = followers[self.veh_id].current_state

        lead_dist = np.linalg.norm(leaders[0].current_state
                                   - self.current_state)
        if lead_dist <= .2:
            self.converge_flag = True

        u = np.arctan2(xo[1]-xn[1], xo[0]-xn[0])
        return u

    def dynamics(self, x, t, u):
        x1 = np.cos(u)
        y1 = np.sin(u)
        f_x = np.array([x1, y1])
        return f_x

    def select_leader(self):
        pass


class SimpleLeader(Dynamics):

    """Docstring for Simple. """

    def __init__(self, init_state):
        """TODO: to be defined1. """
        Dynamics.__init__(self, init_state)
        self.depth_flag = 0
        self.flag = 1

    def input_sys(self, leaders, followers):
        xn = self.current_state
        xo = np.array(followers[3].current_state)
        u = np.arctan2(xn[1]-xo[1], xn[0]-xo[0])
        return u

    def dynamics(self, x, t, u):
        x1 = 0.2*np.cos(u)
        y1 = 0.2*np.sin(u)
        f_x = np.array([x1, y1])
        return f_x


Dynamics.initial_time = 0
Dynamics.final_time = 20
Dynamics.step_size = 0.01


# Create agents with initial Conditions
followers = []
followers.append(SimpleFollower([0., 0]))
followers.append(SimpleFollower([4., 0]))
followers.append(SimpleFollower([-1., -2]))
followers.append(SimpleFollower([1., -5]))

leaders = []
a0 = SimpleLeader([2., 2])
leaders.append(a0)

# Some initial Settings
no_iter = a0.no_iter
t = np.zeros(no_iter)
t0 = 0

for l in range(0, SimpleFollower.followers_count):
    for m in range(0, SimpleFollower.followers_count):
        followers[m].observed_identifier(leaders, followers)

# Run the Simulation
for i in range(0, no_iter):
    for l in range(0, SimpleFollower.followers_count):
        for m in range(0, SimpleFollower.followers_count):
            followers[m].observed_identifier(leaders, followers)
    # Updating the states being observed
    leaders[0].states[:, i] = leaders[0].update_state(leaders=leaders,
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
    t0 = t0 + Dynamics.step_size
    new_iter = i



plt.figure(1)
for j in range(0, SimpleFollower.followers_count):
    plt.plot(followers[j].states[0, 0:new_iter],
             followers[j].states[1, 0:new_iter], linestyle='--', linewidth=2.0,
             label='a'+str(j))
plt.plot(leaders[0].states[0, 0:new_iter], leaders[0].states[1, 0:new_iter],\
         linestyle='--', linewidth=2.0, label='leader')
plt.grid()
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('x-y graph for all agents')
plt.legend(loc='upper left', frameon=False)
# plt.savefig('/home/aditya/Dropbox/Multiple_Leaders_2018/Multiple_Lyx/xy_plot_dynamic.png')
plt.show()
print(t0)
