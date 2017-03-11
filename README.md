# Extended Kalman Filter Project

This is a turn-in project for Udacity's Self-Driving car program.  It 
is based on the [Starter Repo](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project), though
has a number of changes in implementation stategy.  



## Build Instructions

```
mkdir build
cd build
cmake ..
make
./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt out.txt
```



## Notes

The coefficients that are selected in `FusionEKF.cpp` were specifically selected to produce
decent results with `sample-laser-radar-measurement-data-1.txt`.  The way they were selected 
was by iteratively adjusting coefficients to reduce the RMSE.  These coefficients most 
definitely do not perform the same for the other data set.  Indeed, they are quite a bit different
in order to produce decent results with it.  It is a bit troubling that there is not a 
one-size-fits-all collection of coefficients that works for both, as one would thing that barring
a significant difference in process generating the two sets of points, the filter ought to
track fairly uniformly.
