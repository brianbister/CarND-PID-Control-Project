The PID controller is a feedback control system which helps a vehicle
steer back to the center of the road. Its based of the Cross-Track
Error, CTE, which is the distance from the point where the car is
supposed to be.  
P: Proportional  
I: Integral  
D: Derivative  

The (P)roportional component is the CTE multiplied with the proportional
gain. So you steer in proportion to the CTE.

The (D)erivative component takes into account that the we are reducing
the error overtime. This helps prevent the car from oscillating around
the center.

The (I)ntegral component takes into account any systematic bias, like
if the wheels are completely aligned.

# How the Gains were picked.

Before implementing Twiddle, I tried to get some baseline values. I
did a few runs with just the P component of the controller, few values
on different orders of magnitude, 10, 1, .1, .01, and found .1 worked
the best. With just the P componenent the car oscillated quite a bit.

After I implemented the derivative component using the same scheme same
values as the proportional, and determined that a value of 1 worked best,
at this point the car began to oscillate less.

At this point I implemented the the integral componenet, I found that
this didn't have a huge effect on the CTE, I actually found unless I
picked a larger value it made it worse. Maybe because it was a simulator
there was no systematic bias? I wasn't sure so I still put it at a very
small value, .0001.

At this point I then ran my car through Twiddle using initial values of
P-.1, I-.00001, and D-1. Every 500 messages I ran through a step of
Twiddle. I implemented it as a state machine, where we have three states
for initialization, one for testing if a larger value for the parameter
results in less CTE, and one for testing if a lower value results in
less CTE. I also transitioned between each of the three parameters in
the state machine.

After doing this I arrived at final values of .41, .000011, and 3.89.
The car seemed to oscillate a lot still at curves, I think adding speed
to one our parameters would help as it's usually natural to slow down
on big curves.

I also feel rather than picking every 500 points before going to the
next step of twiddle, it would have been more effective to do it
every lap. This is because the total CTE was higher at same points
like the large curves so even if the parameters selected were better,
it wouldn't have kept the values because they were tested on a tricky
portion of the track.
