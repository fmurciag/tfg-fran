from functools import partial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pycpd import DeformableRegistration
import numpy as np
import numpy as np
import pycpd


def setPointsClouds(X,Y):
    X = np.loadtxt(X)
    # synthetic data, equaivalent to X + 1
    Y = np.loadtxt(Y)
    return X,


@DeprecationWarning
def cpd_registration(source, target):
    """
    Registra una nube de puntos 3D (source) en otra nube de puntos 3D (target) utilizando el algoritmo CPD.
    """
    # Normalizar la nube de puntos fuente
    source_mean = np.mean(source, axis=0)
    source_std = np.std(source, axis=0)
    source = (source - source_mean) / source_std
    
    # Normalizar la nube de puntos objetivo
    target_mean = np.mean(target, axis=0)
    target_std = np.std(target, axis=0)
    target = (target - target_mean) / target_std#o los quito o los divido por uno
    
    # Configurar los parámetros del algoritmo CPD
    cpd = pycpd.DeformableRegistration(
        tolerance=1e-5, max_iterations=50, 
        w=0.1, n=10, beta=2, 
        sigma2=None, rotation=None,
        X=target,Y=source)
    
    # Realizar el registro CPD
    transformed, _= cpd.register()
    
    # Desnormalizar la nube de puntos registrada
    transformed = transformed * target_std + target_mean
    
    return transformed


def cpd_registration2(source, target):
    """
    Registra una nube de puntos 3D (source) en otra nube de puntos 3D (target) utilizando el algoritmo CPD.
    """
    # Normalizar la nube de puntos fuente
    source_mean = np.mean(source, axis=0)
    source_std = np.std(source, axis=0)
    source_normalized = (source - source_mean) / source_std

    # Normalizar la nube de puntos objetivo
    target_mean = np.mean(target, axis=0)
    target_std = np.std(target, axis=0)
    target_normalized = (target - target_mean) / target_std
    
    # Configurar los parámetros del algoritmo CPD
    cpd = pycpd.DeformableRegistration(
        tolerance=1e-5, max_iterations=50, 
        w=0.1, n=10, beta=10, 
        sigma2=None, rotation=None,
        X=target_normalized, Y=source_normalized)
    
    # Realizar el registro CPD
    transformed_normalized, transformation_matrix = cpd.register()
    
    # Desnormalizar la nube de puntos registrada
    transformed = transformed_normalized * target_std + target_mean
    
    return transformed, transformation_matrix



