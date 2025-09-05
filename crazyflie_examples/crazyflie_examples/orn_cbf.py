#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def executeTrajectory(timeHelper, cf, trajpath, rate=100, offset=np.zeros(3), yawrate=0.0):
    traj = Trajectory()
    traj.loadcsv(trajpath)

    start_time = timeHelper.time()
    yaw = 0
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > traj.duration:
            break

        e = traj.eval(t)
        yaw = yaw + yawrate * 1/rate
        omega = e.omega
        omega[2] = yawrate
        cf.cmdFullState(
            e.pos + np.array(cf.initialPosition) + offset,
            e.vel,
            e.acc,
            yaw,
            omega)
        print(yaw)

        timeHelper.sleepForRate(rate)

def executeTrajectory2(timeHelper, cf, rate=100, offset=np.zeros(3), yawrate=0.0):

    start_time = timeHelper.time()
    yaw = 0
    pos = np.zeros(3)
    vel = np.zeros(3)
    acc = np.array([0])
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > 5.0:
            break

        e = traj.eval(t)
        yaw = yaw + yawrate * 1/rate
        omega = e.omega
        omega[2] = yawrate
        cf.cmdFullState(
            e.pos + np.array(cf.initialPosition) + offset,
            e.vel,
            e.acc,
            yaw,
            omega)
        print(yaw)

        timeHelper.sleepForRate(rate)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    rate = 30.0
    Z = 0.5
    yawrate = 2.0
    max_yaw_acc = 5.0
    # max_yaw_rate = 5.0

    # high-level mode test
    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/figure8.csv')
    cf.uploadTrajectory(0, 0, traj1)

    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    # cf.goTo([0.0, 0.0, Z], 1.4, 2.0)
    # timeHelper.sleep(2.0)


    cf.setParam('hlCommander.yawacc', max_yaw_acc)
    cf.setParam('hlCommander.yawrlim', yawrate)
    cf.setParam('hlCommander.yawrate', yawrate)

    # executeTrajectory(timeHelper, cf,
    #                   Path(__file__).parent / 'data/figure8.csv',
    #                   rate,
    #                   offset=np.array([0, 0, 0.5]),
    #                   yawrate=yawrate)
    # cf.startTrajectory(0)
    # timeHelper.sleep(traj1.duration)

    cf.goTo([0.0, 1.0, Z], 0.0, 5.0)
    timeHelper.sleep(5.0)

    
    cf.setParam('hlCommander.yawrate', 0.0)
    timeHelper.sleep(2.0)

    # cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)


if __name__ == '__main__':
    main()
