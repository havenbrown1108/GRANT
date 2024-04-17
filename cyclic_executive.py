import numpy as np
from sympy import divisors
import math

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
        if(self.periods.count == 0):
            for task in tasks:
                self.periods.append(task.period)
        for task in tasks:
            self.execution_times.append(task.execution_time)
        
        # Constraint 1
        f = np.max(self.execution_times)
        print("Constraint 1 => f must be greater than " + str(f))

        # Constraint 2
        F = []
        for task in tasks:
            frown = divisors(task.period)
            for divisor in frown:
                if(divisor not in F):
                    F.append(divisor)
        F.sort()

        F = [x for x in F if x >= f]
        print("Constraint 2 => F: " + str(F))

        # Constraint 3
        possible_frames = []
        print("Constraint 3 => ")

        count = 0
        for task in tasks:
            t_f = []
            for frame in F:
                if (2*frame - math.gcd(task.period, frame)) <= task.period:
                    t_f.append(frame)
                else:
                    break
            count = count + 1
            print("possible frames for task " + str(count) + ": " + str(t_f))
            possible_frames.append(t_f)
        
        max_shared_f = 0
        for frame in possible_frames[0]:
            isInAllSets = True
            for frame_set in possible_frames:
                if(not np.isin(frame, frame_set)):
                    isInAllSets = False
                    break
            if(isInAllSets and (frame > max_shared_f)):
                max_shared_f = frame
        
        print("Best frame (in microseconds): " + str(max_shared_f))

    @staticmethod
    def gcd(num1, num2):
        all_divs = divisors(num1, num2)
        all_divs = list(all_divs)
        return np.max(all_divs)
    
    def gcd_list(num1, list_of_nums):
        max = 0
        for num in list_of_nums:
            val = gcd(num1, num)
            if val > max:
                max = val
        
        return max

        
         



t1 = Task(50000, 1320)
t2 = Task(150000, 12400)
t3 = Task(300000, 576)

tasks = [t1, t2, t3]

ce = CyclicExecutive()

ce.Calculate_Hyperframe(tasks)
ce.Calculate_Frame(tasks)

# Example 1
# t_1a = Task(15,1)
# t_2a = Task(20,2)
# t_3a = Task(22,3)

# tasks_a = [t_1a, t_2a, t_3a]
# ce2 = CyclicExecutive()
# ce2.Calculate_Hyperframe(tasks_a) # should be 660
# ce2.Calculate_Frame(tasks_a) # should be 

# Example 2
# t1 = Task(4, 1)
# t2 = Task(5, 2)
# t3 = Task(20, 5)

# tasks = [t1, t2, t3]

# ce = CyclicExecutive()

# ce.Calculate_Hyperframe(tasks)
# ce.Calculate_Frame(tasks)



