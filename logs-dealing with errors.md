


### Issue-1
Persistent “Lost messages on topic /odom” Warnings in MATLAB ROS 2 Subscriber

---

When I started to subscribe to `/odom`, retrieve the latest Odometry message, MATLAB terminal throw warnings. I tried to ignore it at first. However, every time I run my script I see repeated warnings in the MATLAB terminal labelling “Lost messages on topic /odom.” For example:

```
Warning: [/matlab_node]: Lost messages on topic /odom. Total lost messages: 123  
```

I have confirmed that:

- It slows down MATLAB and I don't receive the output from `/odom` subscriber
    
- The warnings mostly appear after I add  `/odom` subscriber and call `receive(odomSub,timeout)`
    
- Inserting a long `pause(30)` before calling `receive` suppresses—but does not eliminate—the warnings.
    


Check **how fast** the `/camera/image_raw` is publishing:

```bash

ros2 topic hz /camera/image_raw
average rate: 11.189
	min: 0.037s max: 0.240s std dev: 0.05931s window: 12
average rate: 11.868
	min: 0.037s max: 0.240s std dev: 0.04796s window: 25
average rate: 11.842
	min: 0.037s max: 0.240s std dev: 0.04788s window: 37
average rate: 12.691
	min: 0.036s max: 0.240s std dev: 0.04367s window: 53
average rate: 12.372
	min: 0.036s max: 0.240s std dev: 0.04161s window: 65
average rate: 10.883
	min: 0.036s max: 0.469s std dev: 0.06803s window: 73


```


---
**Solution**:

The tutor noted the root cause could be in the ROS 2 middleware (RMW) implementation rather than MATLAB alone, and asked which RMW was in use. A check via `ros2 doctor --report` revealed the default `rmw_fastrtps_cpp` (Fast DDS) was active.


1. **Installed Cyclone DDS plugin**
    
    ```bash
    sudo apt install ros-humble-rmw-cyclonedds-cpp
    ```
    
2. **Switched RMW to Cyclone DDS** by exporting:
    
    ```bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```
    
3. **Restarted MATLAB** in that same shell so the new RMW loaded. Also add the export command to `/.bashrc` to source it without issues.
    

With Cyclone DDS warnings disappeared.

---

**Key takeaway:**  
When you see DDS queue overflow warnings in MATLAB’s ROS 2 subscriber, verify and if needed switch your RMW implementation (e.g. to Cyclone DDS)