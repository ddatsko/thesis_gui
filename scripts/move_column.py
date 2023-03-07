from sys import argv
import pandas as pd


def main():
    file_from = argv[1]
    column_name_from = argv[2]
    file_to = argv[3]
    column_name_to = argv[4]

    df_f = pd.read_csv(file_from)
    df_t = pd.read_csv(file_to)

    df_t[column_name_to] = df_f[column_name_from]
    df_t.to_csv(file_to)


if __name__ == '__main__':
    main()