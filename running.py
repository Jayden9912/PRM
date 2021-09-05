import numpy as np
from numpy.core.fromnumeric import size
import pylab as pl
import sys
from PRMController import PRMController
from options import PRMControllerOptions

###
#question given
import environment_2d
pl.ion()
np.random.seed(4)
###

options = PRMControllerOptions()
opts = options.parse()

if __name__ == "__main__":
    prm = PRMController(opts)
    # Initial random seed to try (for generating coordinates, using rng)
    initialRandomSeed = 4
    prm.runPRM(initialRandomSeed)
