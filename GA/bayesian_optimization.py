
from re import X
import numpy as np
np.random.seed(237)
import matplotlib.pyplot as plt
from skopt.plots import plot_convergence, plot_gaussian_process
import json
import os
import time


noise_level = 0.0

def f(x, noise_level=noise_level):
    with open('PnH_aba/PnH/SimCond.json', 'r') as f:
        SimSet = json.load(f)
    data = {'iteration': SimSet['iteration']+1, \
            'OverRot': x[0], \
            'RotBac': x[1], \
            'iter_num': SimSet['iter_num'], \
            'iter_to_Insert': SimSet['iter_to_Insert']}
    with open('PnH_aba/PnH_OP/SimCondOP.json', 'w', encoding='utf-8') as f:
        json.dump(data, f)

    os.system("abaqus cae noGUI=PnH_aba/PnH_OP/TiePH_4steps_op.py")
    # os.system("abaqus cae script=PnH_aba/PnH_OP/TiePH_4steps_op.py")

    with open('PnH_aba/PnH_OP/SimRes.json', 'r') as f:
        GAObj = json.load(f)
    
    FuncVal = np.square(GAObj['MisesStress']) +  (np.square(GAObj['Xcoord'])+np.square(GAObj['Zcoord']))*12500000
    return FuncVal

from skopt import gp_minimize

def RunGO():
    startOP = time.time()
    res = gp_minimize(f,                  # the function to minimize
                    [(0.01, 12),(0.01, 12)],      # the bounds on each dimension of x
                    acq_func="EI",      # the acquisition function
                    n_calls=50,         # the number of evaluations of f
                    n_random_starts=30,  # the number of random initialization points
                    noise=0.0,  )     # the noise level (optional)
                      # the random seed

  #############################################################################
  # Accordingly, the approximated minimum is found to be:

    "x^*=%.4f, f(x^*)=%.4f" % (res.x[0], res.fun)

  #############################################################################
  # For further inspection of the results, attributes of the `res` named tuple
  # provide the following information:
  #
  # - `x` [float]: location of the minimum.
  # - `fun` [float]: function value at the minimum.
  # - `models`: surrogate models used for each iteration.
  # - `x_iters` [array]:
  #   location of function evaluation for each iteration.
  # - `func_vals` [array]: function value for each iteration.
  # - `space` [Space]: the optimization space.
  # - `specs` [dict]: parameters passed to the function.

    # print(res)
    print('best x =',res.x,'best fun =',res.fun)

#############################################################################
# Together these attributes can be used to visually inspect the results of the
# minimization, such as the convergence trace or the acquisition function at
# the last iteration:

# from skopt.plots import plot_convergence
    plot_convergence(res)
    endOP = time.time()
    # print('times =', endOP-startOP,file=open('x_iter.txt','w'))
    lines = ['best x =',str(res.x),'best fun =',str(res.fun),'times =', str(endOP-startOP)]
    with open('x_iter.txt', 'a+') as g:
        for line in lines:
            g.write(line)
            g.write('\n')
        g.write('\n')
    

    return res.x
    
# RunGO()



