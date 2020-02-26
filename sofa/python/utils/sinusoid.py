# %matplotlib inline
import numpy as np

def gen_sinusoid(amp, freq, phase, interval):
    t = np.arange(interval[0], interval[1],.01)
    return t, amp*np.cos(2*np.pi*freq*t + phase)
