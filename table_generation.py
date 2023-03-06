import pandas as pd
import sys

EXPERIMENTS_DIR = 'experiments'

CAPTION_TEMPLATE = "CAPTION"
ROWS_TEMPLATE = "ROWS"

ALGORITHMS = (
    'own_1',
    #'own_3',
    #'popcorn',
    'gtsp',
    #'darp-3'
)

VALUES_PER_ALGORITHM = (
    'max_energy',
    'toppra_max_energy',
    # 'path_time',
    'path_length',
    'computation_time'
)

NICE_ALG_NAME = {
    'own_1': 'Our 1 UAV',
    'own_3': 'Our 3 UAVs',
    'popcorn': 'POPCORN',
    'gtsp': 'GTSP',
    'darp-3': 'DARP 3 UAVS'
}

LATEX_FIELDS_MARK = {
    'max_energy': 'E_{our} [Wh]',
    'energy': 'E [Wh]',
    'toppra_max_energy': 'E_{topp} [Wh]',
    'path_length': 'l [km]',
    'computation_time': 't_c [s]'
}


def get_alg_name(alg):
    return NICE_ALG_NAME[alg] if alg in NICE_ALG_NAME.keys() else alg


def get_one_alg_row():
    return ''.join(map(lambda val: f' & ${LATEX_FIELDS_MARK[val]}$', VALUES_PER_ALGORITHM))


def strip_after_point(number, digits=2):
    if '.' in number:
        return number[:number.rfind('.') + digits]
    return number


def reformat_data(col_name, data):
    if col_name == 'computation_time':
        # Convert to seconds
        return strip_after_point(str(data / 1e9), 2)

    elif col_name == 'energy' or col_name == 'max_energy' or col_name == 'toppra_max_energy':
        # Convert to Wh
        return strip_after_point(str(data / 3600), 2)

    elif col_name == 'path_length':
        # convert to km
        return strip_after_point(str(data / 1000), 2)
    elif col_name == 'path_time':
        return strip_after_point(str(data), 2)

    return str(data)


TABLE_TEMPLATE = r"""
\begin{table*}[t] 
   \centering 
   \footnotesize 
   {\renewcommand{\tabcolsep}{4.5pt} 
   %\renewcommand{\arraystretch}{0.6} 
   \caption{CAPTION} 
   \vspace{-1em} 
   \begin{tabular}{COLUMN_DESC}
     \toprule 
    \multirow{2}{*}{Scenario} & NAMES_ROW  \\
    MULTICOL_DIST
     COLUMN_NAMES \\
    \midrule 
    
    ROWS
    
    \bottomrule 
   \end{tabular} 
      } 
   \vspace{-2.0em}
\end{table*} 
""".replace('COLUMN_NAMES', get_one_alg_row() * len(ALGORITHMS)).replace('COLUMN_DESC', 'c' * (
        len(VALUES_PER_ALGORITHM) * len(ALGORITHMS) + 1)) \
    .replace('NAMES_ROW', ' & '.join(
     map(lambda alg: f"\\multicolumn{{{len(VALUES_PER_ALGORITHM)}}}{{c}}{{{get_alg_name(alg)}}}", ALGORITHMS))) \
    .replace('MULTICOL_DIST', ' '.join(map(lambda
                                               i: f"\\cmidrule(lr){{{i * len(VALUES_PER_ALGORITHM) + 2}-{i * len(VALUES_PER_ALGORITHM) + 1 + len(VALUES_PER_ALGORITHM)}}}",
                                           range(len(ALGORITHMS)))))


ROW_TEMPLATE = r"\multirow{1}{*}{EXPERIMENT} COLS \\".replace('COLS',
                                                              '& % ' * len(ALGORITHMS) * len(VALUES_PER_ALGORITHM))


def readable_experiment(experiment: str):
    return experiment.replace('_', '\\_')


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

    df["max_energy"] = df.apply(lambda row: row["energy"] / 3 if row["algorithm"] == "popcorn" else row["max_energy"], axis=1)
    

    rows = ""

    for experiment in experiments_to_select:
        line = []
        for algorithm in ALGORITHMS:
            line.extend(get_algorithm_stats(df, experiment, algorithm))
        new_row = ROW_TEMPLATE
        for val in line:
            new_row = new_row.replace('%', val, 1)
        new_row = new_row.replace("EXPERIMENT", readable_experiment(experiment))
        rows += new_row + '\n'

    table = TABLE_TEMPLATE
    table = table.replace(ROWS_TEMPLATE, rows)

    print(table)


if __name__ == '__main__':
    filename = 'perf.csv'
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    if len(sys.argv) > 2:
        experiments = list(sys.argv[2:])
    else:
        experiments = ['cape', 'temesvar_nice_8_0', 'rect_8_0', 'temesvar_complex_15', 'temesvar_complex_10', 'temesvar_simple_6']

    make_table(filename, experiments)
