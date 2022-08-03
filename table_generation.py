import pandas as pd
import sys
from popcorn_planner import plan_paths_wadl
from own_planner import plan_paths_own
from optimized_darp_planner import plan_optimized_darp_paths
from gtsp_planner import plan_path_gtsp


EXPERIMENTS_DIR = 'experiments'


CAPTION_TEMPLATE = "CAPTION"
ROWS_TEMPLATE = "ROWS"

ALGORITHMS = (
    'own_1',
    'own_3',
    'popcorn',
    'gtsp_1',
    'darp_1'
)

VALUES_PER_ALGORITHM = (
    'energy',
    'path_time',
    'path_length',
    'computation_time'
)



def strip_after_point(number, digits=2):
    if '.' in number:
        return number[:number.rfind('.') + digits]
    return number


def reformat_data(col_name, data):
    if col_name == 'computation_time':
        # Convert to seconds
        return strip_after_point(str(data / 1e9), 2)

    elif col_name == 'energy':
        # Convert to Wh
        return strip_after_point(str(data / 3600), 3)

    elif col_name == 'path_length':
        # convert to km
        return strip_after_point(str(data / 1000), 2)


    return str(data)



TABLE_TEMPLATE = r"""
\begin{table*}[t] 
   \centering 
   \footnotesize 
   {\renewcommand{\tabcolsep}{4.5pt} 
   %\renewcommand{\arraystretch}{0.6} 
   \caption{CAPTION} 
   \vspace{-1em} 
   \begin{tabular}{ccccccccccccccccc}
     \toprule 
    \multirow{2}{*}{Polygon} & \multicolumn{4}{c}{Our 1 UAV} & \multicolumn{4}{c}{Our 3 UAVs} & \multicolumn{4}{c}{POPCORN} & \multicolumn{4}{c}{GTSP} \\
    \cmidrule(lr){2-5} \cmidrule(lr){6-9}  \cmidrule(lr){10-13} \cmidrule(lr){14-17}
     & E & t & l & s & E & t & l & s & E & t & l & s & E & t & l & s & E & t & l & s \\
    \midrule 
    
    ROWS
    
    \bottomrule 
   \end{tabular} 
      } 
   \vspace{-2.0em}
\end{table*} 
"""


ROW_TEMPLATE = r"\multirow{1}{*}{EXPERIMENT} & % & % & % & % & % & % & % & % & % & % & % & % & % & % & % & % & % & % & % & % \\"


def get_algorithm_stats(df, experiment, algorithm_name):
    df_line = df[(df['experiment'] == experiment) & (df['algorithm'] == algorithm_name)]
    if df_line.empty:
        return ['-'] * len(VALUES_PER_ALGORITHM)
    else:
        stats = []
        for value in VALUES_PER_ALGORITHM:
            stats.append(x if (x := reformat_data(value, df_line[value].iloc[0])) else '-')
        return stats


def make_table(experiments_filename, experiments_to_select):
    if not experiments_to_select:
        return
    df = pd.read_csv(experiments_filename)

    rows = ""

    for experiment in experiments_to_select:
        line = []
        for algorithm in ALGORITHMS:
            line.extend(get_algorithm_stats(df, experiment, algorithm))
        new_row = ROW_TEMPLATE
        for val in line:
            new_row = new_row.replace('%', val, 1)
        new_row = new_row.replace("EXPERIMENT", experiment)
        rows += new_row + '\n'

    table = TABLE_TEMPLATE
    table = table.replace(ROWS_TEMPLATE, rows)

    print(table)


if __name__ == '__main__':
    filename = 'experiments.csv'
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    if len(sys.argv) > 2:
        experiments = list(sys.argv[2:])
    else:
        experiments = ['cape', 'cape_2_directions']

    make_table(filename, experiments)
