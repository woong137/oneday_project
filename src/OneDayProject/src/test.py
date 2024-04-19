import numpy as np
from scipy.stats import multivariate_normal

def p_lc(x): return multivariate_normal.pdf(x, 0, 1)

print(p_lc(0.1))