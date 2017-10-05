# experiment-1

## How to compile

In Linux systems code can be compiled as follows:

```
git clone https://github.com/robotology-playground/experiment-new-grasp.git
cd superquadric-grasp
mkdir build; cd build
ccmake ..
make install
```

## How to run the code
This code can be launched both on the iCub robot and simulator. However, a complete testing can be executed only on the robot, 
since it communicates with several modules that are quite hard to be launched on the simulator.
For running it on the robot, please:
1. Launch the `yarprobotinterface`
2. Launch the basic modules:`iKinGazeCtrl`, `iKinCartsianSolver`- for both right and left arm. 
3. Launch the [`skinManager`](https://github.com/robotology/icub-main/tree/master/src/modules/skinManager) and `skinManagerGui` and connect. Set the `binarization filter` off and the `compensation gain` and the `contact compensation gain` at the minimum values.
4. Launch [`IOL`](https://github.com/robotology/iol ).
5. Launch the [`superquadric-model`](https://github.com/robotology/superquadric-model) and [`superquadric-grasp`](https://github.com/robotology/superquadric-grasp),
the yarpviewers and `experiment-1` module with [this xml](https://github.com/robotology-playground/experiment-new-grasp/blob/master/experiment-1/app/script/experiment-1.xml.template).
The yarpviewers show respectively the estimated superquadric and the computed grasping pose.
6. Connect everything.
7. Interact with the `experiment-1` code.

Here is an example on how to interact with the wrapper code:
```
yarp rpc /experiment-1/rpc
>> set_object_name <object_name>
[ok]
>>acquire_superq
[ok]
>> set_hand_for_computation <hand_for_computation>
>>compute_pose
[ok]
>>set_hand_for_moving <hand_for_moving>
[ok]
>>grasp_object
[ok]
```
- `<object_name>` must be the name of one of the objects recognized by th IOL application.
- `<hand_for_computation>` can be: `right`, `left` or `both`.
- `<hand_for_moving>` can be: `right` or `left`

The command:
- `acquire_superq` executes steps number 1,2,3,4 of the [pipeline](https://github.com/robotology-playground/experiment-new-grasp#experiment-1).
- `compute_pose` executes step 5.
- `grasp_object` executes step 6.


