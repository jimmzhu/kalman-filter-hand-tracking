Train data set: (5 videos)
v1, v2, v3, v4, v5

Test data set: (8 videos)
v1, v4, v5, v6, v7, v8, v10, v14


Rectangle coordinates:
x y w h
(topLeft.x, topLeft.y, width, height)


In Matlab:
```
load test_v10_labels.mat
```
frames - list of image filenames corresponding to each frame
lhd - (supposedly) left hand driver rectangle coordinates (red)
rhd - (supposedly) right hand driver rectangle coordinates (green)
lhp - (supposedly) left hand passenger rectangle coordinates (magenta)
rhp - (supposedly) right hand passenger rectangle coordinates (blue)

The detection labels aren't consistent in determining which hand is where.
It's possible for hand data to be swapped (rhd coordinates given in lhp, and
vice versa). See v1_detected_001.png and v1_detected_449.png for an example of
this inconsistency.

Visit http://cvrr.ucsd.edu/vivachallenge/index.php/hands/hand-detection/
to download the hands dataset. Unzip and copy the contents of test/pos/ and
train/pos/ into the data/test/pos/ and data/train/pos/ directories.
