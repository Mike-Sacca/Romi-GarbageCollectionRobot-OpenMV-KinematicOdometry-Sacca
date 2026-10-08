# Final Romi Project for RBE2002 Course

### Notes

* Code for a Romi robot designed to use an Ir sensor to recieve commands and autonomously navigate to any location in 2d space
* System uses its starting point as origin (0,0) and will travel to any point given in a straight line given points (x, y)
* Robot designed to use a small 9 gram servo and load cell to pick up bins and read their weight
* System uses kinematic odometry for coordinate navigation and april tags for local navigation / approach and alignment using an openMV camera
* Robot can be programmed with a designated "drop off" location to bring each collected bin to
