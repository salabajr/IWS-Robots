# Wireless Timesync Procedure

This document provides instructions on how to wirelessly sync the stations and ros nodes. 

## How to time sync the wireless bridges and AP: 

- First, time sync the AP to grand leader clock (GL) 

	`ssh 10.10.0.211`

	`sudo setup_sync_l2.sh -i eno1 -b slave`

- Then start the wireless sync: 

	`sudo startGptpS.sh` on relevant stations (10.10.0.212-10.10.0.216)

	`sudo startGptpM.sh` on AP 

- To see the output of the wireless sync stream: 

	`sudo screen -ls` 

	`sudo screen -r` insert name of previous output

- Then start a ptp master on the bridging port each of the stations: 

	`sudo setup_sync_l2.sh -i br0 -b master`

## How to time sync the ROS nodes: 

### Supervisor 

Since this machine will be wired, we need to sync it to the GL: 

`sudo setup_sync_l2.sh -i enp0s31f6 -b slave`

To see the output of the ptp4l stream: 

`sudo screen -ls` 

`sudo screen -r ` insert name of ptp4l

### Leader

For wired and wireless: 

`sudo setup_sync_l2.sh -i enp2s0 -b slave` 

### Follower

For wired and wireless: 

`sudo setup_sync_l2.sh -i enp2s0 -b slave`