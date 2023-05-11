# importing libraries
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt



#----------PARAMETERS-----------------#
#--EgoVehicle--#
m1 = 1000   # weight
v1 = np.linspace(1, 100, 20)   # velocity in m/s
belted = True   # Passengers belted?


#--Object Found--#
m2 = 1000   # weight
v2 = 0   # velocity in m/s
vulnerability_weight = 1    # Vulnerability of object
distance = np.linspace(1, 500, 20)   # Distance to object in m


#--Other Parameters--#
# Normalization coefficients
n1 = 1   #Severity
n2 = 2   #Collision Propensity

#Relative weights
beta1 = 0.5 #Severity
beta2 = 1 - beta1 #Collision Propensity





#--------SEVERETY SCORE CALCULATION--------------#
V1, D = np.meshgrid(v1, distance)

if belted:
    alfa = 67.4
    k = 2.62

else:
    alfa = 66.1
    k=2.22


delta_v = (m2/(m1+m2))*abs(v2-V1)*2.2369

prob_delta_v = (delta_v/alfa)**k

severity = (prob_delta_v*vulnerability_weight)/n1

ttc = D/abs(v2-V1)

collision_prob = ttc/n2

severity_score = (beta1*severity)/(beta2*collision_prob)

#--------PLOTTING---------#
fig = plt.figure()
 
# syntax for 3-D plotting
ax = plt.axes(projection ='3d')
 
# syntax for plotting
ax.plot_surface(V1, D, severity_score, cmap ='viridis', edgecolor ='green')
ax.set_title('Severity Score Calculation')
ax.set_xlabel('Ego-vehicle velocity [m/s]')
ax.set_ylabel('Distance between ego-vehicle and object detected [m]')
ax.set_zlabel('Severity Score [-]')
plt.show()







# #------------PRINTING RESULT-----------------#
# print('----------PARAMETERS-------------')
# print('Ego Vehicle')
# print('Weight: ' + str(m1) + ' kg')
# print('Velocity' + str(v1) + ' m/s')
# print('Passengers belted: ' + str(belted))


# print('Object Found')
# print('Weight: ' + str(m2) + ' kg')
# print('Velocity: ' + str(v2) + ' m/s')
# print('Vulnerability weight: ' + str(vulnerability_weight))
# print('Distance between ego vehicle and object: ' + str(distance) + ' m')


# print('Normalization coefficients')
# print('Severity - ' + str(n1))
# print('Collision propensity - ' + str(n2))

# print('Relative weights')
# print('Severity - ' + str(beta1))
# print('Collision propensity - ' + str(beta2))

# print('------------RESULTS-----------')
# print('SEVERITY SCORE: ' + str(severity_score))
# print(severity_score)
