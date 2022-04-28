import numpy
import pandas as pd
import tigramite.data_processing as pp
from tigramite.pcmci import PCMCI
from tigramite.independence_tests import GPDC, GPDCtorch
from constants import *

class causal_model:
    def __init__(self, filename, vars_name, alpha):
        """
        Class constructor

        Args:
            filename (str): data file .csv to open
        """
        self.vars_name = vars_name
        self.alpha = alpha
        self.df_traj = pd.DataFrame(columns = ['x', 'y'])
        self.df = pd.read_csv(DATA_DIR + "/" + filename)
        self.inference_dict = dict()
    

    def run_causal_discovery_algorithm(self):
        """
        Run causal discovery algorithm

        Build a dictionary describing the inference between variables

        Convert dictionary to JSON string
        """
        # build tigramite dataset
        vector = numpy.vectorize(float)
        data = vector(self.df)
        dataframe = pp.DataFrame(data = data,
                                 var_names = self.vars_name)

        # init and run pcmci
        cond_ind_test = GPDC(significance = 'analytic', gp_params = None)
        pcmci = PCMCI(dataframe = dataframe,
                      cond_ind_test = cond_ind_test,
                      verbosity = 0)

        result = pcmci.run_pcmci(tau_max = 1,
                                 tau_min = 1,
                                 pc_alpha = 0.05)

        self.inference_dict = self.__build_causal_model(result['p_matrix'], result['val_matrix'])

    
    def __build_causal_model(self, pval_matrix, val_matrix):
        """
        Build a dictionary containing inference information 
        between variables for any time lag previously defined

        Args:
            pval_matrix (ndarray): pvalue matrix returning from causal discovery algorithm
            val_matrix (ndarray): value matrix returning from causal discovery algorithm

        Returns:
            dict_cm (dictionary{str : ndarray}): causal model dictionary
        """

        # number of lags and vars definition
        nlags = val_matrix.shape[-1]
        nvars = val_matrix.shape[0]

        # filter causal model by alpha
        for lag in range(0, nlags):
            for var in range(0, nvars):
                for parent in range(0, nvars):
                    if pval_matrix[var, parent, lag] > self.alpha:
                        val_matrix[var, parent, lag] = 0
        
        # building dictionary
        dict_cm = dict()
        for lag in range(0, nlags):
            inference = numpy.zeros((3,3))
            for var in range(0, nvars):
                inference[var] = val_matrix[var][:,lag].transpose()
            dict_cm[lag] = inference.tolist()
        return dict_cm
