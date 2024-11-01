from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    # print(quat)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    # just unpack yaw
    return yaw


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    print(goal_pose)
    currx, curry, theta, timestamp = current_pose
    goalx, goaly = goal_pose
    # print("stats:", currx, curry, goal_pose)
    error_linear= sqrt((currx-goalx)**2 + (curry-goaly)**2)
    print("lin e:", error_linear)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal
    
    currx, curry, theta, timestamp = current_pose
    goalx, goaly = goal_pose
    print(goal_pose)

    desiredTheta = atan2(goaly-curry, goalx-currx)
    # print("desired theta", desiredTheta)
    # print("current Theta", theta)
    error_angular = desiredTheta - theta

    # Remember to handle the cases where the angular error might exceed the range [-π, π]

    if error_angular >= M_PI:
        error_angular -= 2*M_PI
    elif error_angular <= -M_PI:
        # error_angular = error_angular % M_PI - M_PI
        error_angular += 2*M_PI
    # if error_angular > M_PI:
    #     error_angular %= 2*M_PI - M_PI
    # elif error_angular < -M_PI:
    #     error_angular %= 2*M_PI + M_PI
    # print("error ang", error_angular)
    return error_angular
