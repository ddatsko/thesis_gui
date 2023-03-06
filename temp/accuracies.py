from sys import argv
import pandas as pd


def acc(real, meas):
    return 100 - abs(real - meas) / real * 100


def accuracies(filename1, filename2):
    df1 = pd.read_csv(filename1)
    df2 = pd.read_csv(filename2)

    min_acc = 100
    for index, row in df1.iterrows():
        # print(df2[(df2["experiment"] == row['experiment']) & (df2['algorithm'] == row['algorithm'])]['max_energy'][index])
        this_energy = float(row['max_energy'])
        other_energy = float(df2[(df2["experiment"] == row['experiment']) & (df2['algorithm'] == row['algorithm'])]['max_energy'][index])
        # print("FF: ", this_energy, other_energy)
        min_acc = min(min_acc, acc(this_energy, other_energy))
    print(min_acc)


if __name__ == '__main__':
    accuracies(argv[1], argv[2])
