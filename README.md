# LEO-Satellite
LEO Satellite 6-DOF Simulation implemented in C++ with project setup/build in C-make

Featuring Attitude Control, Orbit Control/station-keeping, and optimization for RCS thruster usage.

(Currently In Development), (I expect to be finished by the middle of December 2022)
Calendar progress goals:
  
  -by Nov 15 fully setup attitude portion of sim (does not include testing)
  
  -by Nov 17 have orbit controller/stationkeeping controller setup
  
  -by December 1st be in tuning phase
  
  -finish by Christmas by the absolute latest
  
  [break until thanksgiving break]
  
  During thanksgiving break:
    -setup orbit portion of sim for a simple circular LEO Orbit, no need to setup realistic perturbations just yet
    -setup VUW attitude frame as reference frame
    -work on making an in house r,v to keplerian elements conversion software in C++ to implement in this sim
  


Development progress log:

  -Nov 13: got controller setup to add and convert quaternions into the comparative MRP for the attitude control law, controller seems all setup
