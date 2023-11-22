import numpy as np
from pandas import DataFrame


def get_velocity(track_df: DataFrame) -> np.array:
    """
    Calculates velocity given x-velocity and y-velocity

    :param track_df: track data frame of a vehicle
    :return: array of velocities for vehicle
    """
    return np.sqrt(track_df.xVelocity**2 + track_df.yVelocity**2)


def get_orientation(track_df: DataFrame) -> np.array:
    """
    Calculates orientation given x-velocity and y-velocity

    :param track_df: track data frame of a vehicle
    :return: array of orientations for vehicle
    """
    return np.arctan2(-track_df.yVelocity, track_df.xVelocity)


def get_acceleration(track_df: DataFrame, orientation: np.array) -> np.array:
    """
    Calculates acceleration given x-acceleration and y-acceleration

    :param track_df: track data frame of a vehicle
    :return: array of accelerations for vehicle
    """
    return np.cos(orientation) * track_df.xAcceleration + np.sin(orientation) * (
        -track_df.yAcceleration
    )


def get_dt(recording_df: DataFrame) -> float:
    """
    Extracts time step size from data frame

    :param recording_df: data frame of the recording meta information
    :return: time step size
    """
    return float(1.0 / recording_df["frameRate"].iloc[0])
