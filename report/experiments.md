# Experimental Design

## Experiment 1: Block Frequency

**Setup:**
Sets of 10 blocks are placed in the middle of the belt at decreasing distances between them to increase the frequency of blocks passing through the camera’s path.  The percentage of blocks that get picked up will be measured.

**Hypothesis:**
There will be a maximum frequency of blocks on the belt, in blocks per meter, past which the UR5e will not be able to sustainably pick them all up.

**Variables:**
Block frequency *f*, in blocks per meter

**Metrics:**
Percentage of blocks picked up, Success%, per tested *f*

## Experiment 2: Block Placement

**Setup:**
10 blocks are placed (at a low enough frequency not to interfere with the experiment) at every position across the width of the belt (+- 0.18m from the center of the belt’s width) at a granularity of 0.04m.  For this experiment, the placement of the belt was modified to be centered on the camera position and narrow enough that the whole belt fits on the camera.  The percentage of blocks that get picked up are measured as well as the range of positions that are successfully picked up if some blocks aren’t picked up successfully.

**Hypothesis:**
Regardless of block placement on the belt, the UR5e will be able to pick it up.

**Variables:**
Vertical placement of block across the width of the belt, *y*

**Metrics:**
Percentage of placements that get picked up, `Success%`

## Experiment 3: Acceptable Block Color Sets

**Setup:**
12 uniquely colored blocks are placed at the center width of the belt at a reliable frequency.  The blocks are colored such that each unique color is at least 0.5 away (on an RGB scale from 0-1) from every other on at least 1 channel.  Each trial will test accepting an increasing set of colors from those 12.

**Hypothesis:**
Regardless of the size of the set of accepted block colors, the UR5e will pick up all accepted colors and ignore all nonaccepted colors.

**Variables:**
Cardinality of accepted colors set, `|Accepted|`

**Metrics:**
Percentage of accepted colors picked up, `Success%`, and percentage of nonaccepted colors ignored, `Ignore%`, for each trial

<br>

# Results & Discussion

## Experiment 1

|     Trial Number    |     Spacing (Meters Between)    |     Frequency f (Blocks/Meter)    |     Percentage of blocks grabbed    |
|---------------------|---------------------------------|-----------------------------------|-------------------------------------|
|     1               |     0.5                         |     2                             |     100%                            |
|     2               |     0.45                        |     2.22                          |     100%                            |
|     3               |     0.4                         |     2.5                           |     100%                            |
|     4               |     0.35                        |     2.86                          |     80%                             |
|     5               |     0.30                        |     3.33                          |     50%                             |

As the spacing reached a point where blocks were already in the camera’s view by the time it was ready to pick up another block, the amount of dropped blocks drastically increased.  My hypothesis is that the controller isn’t accurately pinpointing the blocks’ locations if it tries to find them directly after getting back to the starting position because its movement hasn’t completely settled by that point.

## Experiment 2

|     Percent Success           |     80%                                |
|-------------------------------|----------------------------------------|
|     Successful ranges         |     [+0.18m, -0.04], [-0.12, -0.16]    |
|     Unsuccessful positions    |     -0.08m, -0.18m                     |

For the two unsuccessful positions, the manipulator moved to and gripped the block successfully but dropped it before reaching the output belt due to issues gripping and/or potential friction issues if moving before lifting enough.

Note for reproducing: when the block at `-0.08` failed, I paused the simulation to remove the block so the arm wouldn’t try to retrieve it instead of the next block in the sequence.

## Experiment 3

|     Trial Number (\|Accepted\|)    |     Percent Expected Positives    |     Percent Expected Negatives    |
|------------------------------------|-----------------------------------|-----------------------------------|
|     0                              |     N/A                           |     100%                          |
|     1                              |     100%                          |     100%                          |
|     2                              |     100%                          |     100%                          |
|     3                              |     100%                          |     100%                          |
|     4                              |     100%                          |     100%                          |
|     5                              |     100%                          |     100%                          |
|     6                              |     100%                          |     100%                          |
|     7                              |     100%                          |     100%                          |
|     8                              |     100%                          |     100%                          |
|     9                              |     100%                          |     100%                          |
|     10                             |     100%                          |     100%                          |
|     11                             |     100%                          |     100%                          |
|     12                             |     100%                          |     N/A                           |

As hypothesized, the controller was able to distinguish between as many differently sized sets of colors given the constraint that every color was sufficiently different from one another.  Another test could determine the minimum distance between two colors that could be distinguished by the controller.

<br>

# Video Examples

## Experiment 1

Here's a run of the fourth trial of experiment 1, the first example of blocks failing to be picked up:

![type:video](https://www.youtube.com/embed/Fa_NEw6Jo1g?si=mj5D1UZLUvG_k4Yr)

## Experiment 3

Here's a run of the 6th trial of experiment 3, where only the colors corresponding to every other block on the belt were in the accepted set:

![type:video](https://www.youtube.com/embed/ngVO28Vfh7E?si=WDHqWvZVgWzIV79T)