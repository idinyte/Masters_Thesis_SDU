import numpy as np
from scipy.stats import chi2_contingency, kruskal

        
# # Data from the table
# data = {
#     "Success": [75, 38, 44, 28],
#     "Locate Ball": [1, 0, 0, 0],
#     "Grab Ball": [13, 8, 1, 0],
#     "Classify Ball": [0, 53, 52, 67],
#     "Place Ball": [11, 1, 3, 5]
# }

# # Convert data into a contingency table for chi-square test (failure categories by ball type)
# contingency_table = np.array([
#     data["Locate Ball"],
#     data["Grab Ball"],
#     data["Classify Ball"],
#     data["Place Ball"]
# ]).T

# # Perform Chi-Square Test
# chi2_stat, p_val_chi2, dof, expected = chi2_contingency(contingency_table)

# # Prepare success rates for Kruskal-Wallis test
# success_rates = [data["Success"][i] for i in range(4)]
# print(success_rates)

# # Perform Kruskal-Wallis Test on success rates across ball types
# kruskal_stat, p_val_kruskal = kruskal(*success_rates)

# print(chi2_stat, p_val_chi2, dof, expected, kruskal_stat, p_val_kruskal)
