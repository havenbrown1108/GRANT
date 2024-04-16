import numpy as np
from sympy import divisors

class Task:
    period = 0
    execution_time = 0
    
    def __init__(self, period, execution_time):
        self.period = period
        self.execution_time = execution_time

class CyclicExecutive:
    H = float('-inf')
    f = float('-inf')
    periods = []
    execution_times = []
    
    def Calculate_Hyperframe(self, tasks):
        for task in tasks:
            self.periods.append(task.period)
        print(self.periods)
        self.H = np.lcm.reduce(self.periods)
        print("H: " + str(self.H))

    def Calculate_Frame(self, tasks):
        for task in tasks:
            self.execution_times.append(task.execution_time)
        
        # Constraint 1
        f = np.max(self.execution_times)
        print("f after constraint 1: " + str(f))

        # Constraint 2
        F = []
        for task in tasks:
            frown = divisors(task.period)
            for divisor in frown:
                if(divisor not in F):
                    F.append(divisor)
        F.sort()
        print(F)

        F = [x for x in F if x >= f]
        print("F: " + str(F))

        # Constraint 3
        # this isn't done :( im gonna punt this :)
        possible_frames = []
        smallest_possible_frames = float('inf')
        for task in tasks:
            t_f = [f for f in F if 2*f - max(divisors(task.period, f)) <= task.period]
            print("t_f: " + str(t_f))
            possible_frames.append(t_f)
            if len(possible_frames) < smallest_possible_frames:
                smallest_possible_frames = len(possible_frames)
        
         



# t1 = Task(50000, 1320)
# t2 = Task(150000, 12400)
# t3 = Task(300000, 576)

# tasks = [t1, t2, t3]

# ce = CyclicExecutive()

# ce.Calculate_Hyperframe(tasks)
# ce.Calculate_Frame(tasks)

t_1a = Task(15,1)
t_2a = Task(20,2)
t_3a = Task(22,3)

tasks_a = [t_1a, t_2a, t_3a]
ce2 = CyclicExecutive()
ce2.Calculate_Hyperframe(tasks_a)
ce2.Calculate_Frame(tasks_a)




