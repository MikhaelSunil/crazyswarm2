#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node


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

# def executeTrajectory2(timeHelper, cf, rate=100, offset=np.zeros(3), yawrate=0.0):

#     start_time = timeHelper.time()
#     yaw = 0
#     pos = np.zeros(3)
#     vel = np.zeros(3)
#     acc = np.array([0])
#     while not timeHelper.isShutdown():
#         t = timeHelper.time() - start_time
#         if t > 5.0:
#             break

#         e = traj.eval(t)
#         yaw = yaw + yawrate * 1/rate
#         omega = e.omega
#         omega[2] = yawrate
#         cf.cmdFullState(
#             e.pos + np.array(cf.initialPosition) + offset,
#             e.vel,
#             e.acc,
#             yaw,
#             omega)
#         print(yaw)

#         timeHelper.sleepForRate(rate)


def executeTrajectory2(timeHelper, cf,
                        duration: float,
                        rate: float = 100.0,
                        offset: np.ndarray = np.zeros(3),
                        ay: float = 0.0,
                        v0: np.ndarray = np.array([0.0, 0.0, 0.0]),
                        yawrate: float = 0.0,    
                        spin_duration: float = 3.0,
                        spin_yawrate: float | None = None):

    vel = np.array(v0, dtype=float).reshape(3)
    pos = np.zeros(3, dtype=float)

    spin_yawrate = yawrate if spin_yawrate is None else float(spin_yawrate)


    start_time = timeHelper.time()
    last_time = start_time
    yaw = float(0.0)
    omega = np.array([0.0, 0.0, yawrate], dtype=float)
    dt_nominal = 1.0 / float(rate)

    while not timeHelper.isShutdown():
        now = timeHelper.time()
        t = now - start_time
        if t > duration:
            break
        dt = now - last_time
        if dt <= 0: dt = dt_nominal
        elif dt > 2.0 * dt_nominal: dt = 2.0 * dt_nominal
        last_time = now

        # Access latest costmap if available on the node (set in main)
        latest_costmap = getattr(timeHelper.node, 'latest_costmap', None)
        
        # Print costmap info for debugging
        if latest_costmap is not None:
            print(f"Costmap available: resolution={latest_costmap.info.resolution}, "
                  f"width={latest_costmap.info.width}, height={latest_costmap.info.height}")
        else:
            print("No costmap available yet")

        if t < spin_duration:
            acc = np.zeros(3, dtype=float)
            omega = np.array([0.0, 0.0, spin_yawrate], dtype=float)
            yaw += spin_yawrate * dt
            cmd_pos = pos + np.array(cf.initialPosition, dtype=float) + offset
            cf.cmdFullState(cmd_pos, vel*0.0, acc, yaw, omega)

        else:
            # Placeholder: use observation (latest_costmap) to compute acc if desired
            acc = np.array([0.0, float(ay), 0.0], dtype=float) 
            omega = np.array([0.0, 0.0, spin_yawrate], dtype=float)

            vel = vel + acc * dt
            pos = pos + vel * dt
            yaw += yawrate * dt

            cf.cmdFullState(
                pos + np.array(cf.initialPosition, dtype=float) + offset,
                vel,
                acc,
                yaw,
                omega
            )
            timeHelper.sleepForRate(rate)




def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # Subscribe to costmap on the underlying node and store latest
    def _on_costmap(msg: OccupancyGrid):
        setattr(timeHelper.node, 'latest_costmap', msg)
        print(f"Received costmap: resolution={msg.info.resolution}, "
              f"width={msg.info.width}, height={msg.info.height}")
    
    # Subscribe to the costmap topic (note the correct topic name with namespace)
    costmap_subscription = timeHelper.node.create_subscription(
        OccupancyGrid, 
        '/costmap/costmap',  # Updated topic name based on launch file
        _on_costmap, 
        1
    )
    
    # Give some time for the subscription to establish
    print("Waiting for costmap data...")
    timeHelper.sleep(2.0)

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


    executeTrajectory2(timeHelper, cf, 
                    duration=4.0,
                    rate=rate,
                    offset=np.array([0, 0, 0.5]),
                    ay=-0.2,           
                    yawrate=yawrate)


    cf.startTrajectory(0)
    timeHelper.sleep(traj1.duration)

    # cf.goTo([0.0, 1.0, Z], 0.0, 5.0)
    # timeHelper.sleep(5.0)

    
    cf.setParam('hlCommander.yawrate', 0.0)
    timeHelper.sleep(2.0)

    # cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)


if __name__ == '__main__':
    main()
