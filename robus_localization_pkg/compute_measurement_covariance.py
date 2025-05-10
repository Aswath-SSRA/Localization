# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np

CSV_PATH = "/home/aswath/robus_localization/src/robus_localization_pkg/resource/optitrack_data_20250421_002539.csv"

def compute_measurement_covariance(df):
    # Measurement noise R: based on [x, y, yaw]
    measurement_data = df[['X', 'Y', 'Yaw']]
    R = np.cov(measurement_data.T)
    return R

def main():
    df = pd.read_csv(CSV_PATH)
    R = compute_measurement_covariance(df)

    print("Measurement Noise Covariance Matrix R:")
    print(R)

if __name__ == "__main__":
    main()
