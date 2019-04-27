# Model reflection


The model runs in simulator for half hour. It looks pretty well.

[Image1]: ./simulator.png "simulator"
![alt text][Image1]

## Model structure

The framework of the model consists of a finite state machine, which contains 3 states:
- Lane Keeping: keep the vehicle running within the lane at a target speed. When there's a vehicle in front of the ego vehicle, it will change to 'Pre-lane chaning' state.
- Pre-lane changing: run the vehicle at the same speed of the nearest front vehicle. And search for oppotunity of lane changine. When the front vehicle no longer blocks ego vehicle, it will return to 'Lane keeping' state. When it finds a valid oppotunity, it will decides to change left or right lane and move to 'Lane changing' state.
- Lane changing: The execution of lane changing. When the vehicle fully runs in the new lane, it will return to 'Pre-lane changing' state.

## Difficulties

At the beginning of the project, I tried to accelerate/brake the vehicle as quick as possible, i.e. to use max jerk and max acc. But it's difficult than I expected. I do noticed that there's 'PID control' topic in the following lessons. I guess it will help to solve this issue.

After seeing the model can meet the project rubrics, I ran the simulator for quite some time to . I noticed that in some particular cases, the vehicle can still have collisions. This may have something to do with:
- The ego vehicle does accelerate during lane changing. If there's a vehiclein target lane running very fast from behind, and ego vehicle has a very low speed, it may rear-end with ego vehicle.
- If a vehicle in next lane suddenly changes to ego current lane, the ego vehicle may not be able to brake in time, it will crash.
- If there's an accident ahead, the ego vehicle may not be able to brake in time.

## Improvement oppotunities

Although the trajectory strategy is feasible, it can still improve in the following scenario:
- If the ego runs in the most left/right lane and there're two slow cars, one running in front of the ego vehicles, one in the middle lane, the ego vehicle cannot slow down and change to middle lane and then to the other side of the road.
- Speed up during lane changing, just like human being does.
- Take emergency actions, when there's an accident.

By the way, it seems the simulator generates a HUUUUUUGE file on my local drive. Please remeber to delete
~/.config/unity3d/Udacity/self_driving_car_nanodegree_program/Player.log

[Image2]: ./huge_file.png "huge file"
![alt text][Image2]
