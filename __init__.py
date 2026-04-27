#--- Forward Kinematics:
#  3 revolute joints, link lengths: l1, l2, and l3
#  Joint angles: theta1, theta2, theta3

#  F.K. : 
#     
#        x = l1 * cos(theta1) + l2*cos(theta 1 + theta2) + l3*cos(theta1 + theta2+ theta3)
#        y = l1 * sin(theta1) + l2* sin(theta1 + theta2) + l3 * sin(theta1 + theta2 + theta3)