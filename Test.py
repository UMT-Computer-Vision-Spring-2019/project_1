import numpy as np
import scipy.optimize as so
import matplotlib.pyplot as plt

# Use Levenberg-Marquardt algorithm to solve linear least squares problem
# for some data y, and the assumption of a line as a model

# True intercept and slope
# How does this change for the pose estimation problem?
p_true = np.array([0.5,1.25])

# Define our model
# What should this be for the pose estimation problem?
def f(x,p):
    y_predicted = p[0] + p[1]*x
    return y_predicted

# Create synthetic data
n = 11
x = np.random.randn(n)
sigma_obs = 0.1  # Measurement error
y_obs = f(x,p_true) + np.random.randn(n)*sigma_obs

# Define the residual as f(intercept,slope,x) - y
def residual(p,x,y):
    return f(x,p) - y

# Make an initial guess about the slope and intercept
p_0 = np.array([0,0])

# Use scipy implementation of Levenburg-Marquardt to find the optimal
# slope and intercept values.
p_opt = so.least_squares(residual, p_0, method='lm',args=(x,y_obs))['x']
print(p_opt)
#plt.plot(x,y_obs,'k.')
#plt.plot(x,f(x,p_true),'r-')
#plt.plot(x,f(x,p_opt),'b-')
#plt.show()
