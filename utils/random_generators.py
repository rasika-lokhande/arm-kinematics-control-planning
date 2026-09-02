
import numpy as np

def random_quaternions(n):
    q = np.random.normal(size=(n, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q