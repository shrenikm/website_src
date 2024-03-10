---
title: "Setting Up a Manipulator"
date: 2024-03-08T22:11:21-08:00
draft: true
tags: ["robotics", "control", "manipulation", "simulation", "drake"]
---
<!-- Post 7 -->
Let's take a look at how we can set up a framework for streamlined simulation and hardware control of a manipulator. The robot under consideration is Ufactory's Lite6 manipulator and the simulation/setup tool used is Drake.

All the code used in this post can be found in my [repo](https://github.com/shrenikm/manipulation_research). But before we start, some explanations are in order.

Lite6?

The [Lite6](https://www.ufactory.cc/lite-6-collaborative-robot/) is one of the more economical (not to be confused with cheap) manipulators one can buy, especially for personal research where you're not going to be spending tens of thousands of dollars on some of the well established robots out there. It's well built, has surprisingly good specifications for the price point, and comes with an integrated controller. You can get a parallel and vacuum gripper for it, but also comes with hardware interfaces that can support custom grippers. The product is well supported and their API (python and C++) are easy to use. The main pain point is its parallel gripper, which can only open by 1.6 cm. This severely limits the type of objects you can pick/place, but luckily for us, pick and place is only a small part of robotic manipulation!

But overall, it's a pretty well polished product that would serve well for any semi-serious research endeavours.

Drake?

In a world were robotic simulation recommendations are usually one of MuJoCo, Gazebo, PyBullet or IsaacSim, [Drake](https://drake.mit.edu/) [$[1]$](#references) is an extremely underrated piece of software. I've used it a bit previously but coming back to it after 4 years or so has been a pleasant surprise. If you're looking for reasons for using it, here are a few:
* It is still under active development and continues to mature as a robotics toolbox
* Their contact models have evolved over the years and you have more flexibility in choosing the right model and simulation framework for your specific task (More on this later)
* It isn't just a simulation engine. It can do dynamical system design, control, planning and numerical optimization. It comes with out of the box implementations of common robotics algorithms and utilities
* It comes with support for message passing using LCM, which is based on the UDP communication protocol. This enables Drake to be used as a replacement for ROS
* It also has support for sensor and camera integration. There is also a (relatively) new wrapper around [Gymnasium](https://gymnasium.farama.org/) for more RL oriented research
* Drake's python bindings have grown over the years and is at the point where most of the library can be accessed through python 

All in all, Drake gives you a lot for its generous BSD-3 license. To me, it's a no-brainer for any model-based robotics project.


## Robot Model

The first order of business is to get the robot's description files. Description files are used to represent the physical, kinematic and dynamic properties of a robot, usually in an XML format. The most popular formats these days are URDF, SDF and MJCF.

Ufactory provides a URDF xacro for the Lite6, which can be found [here](https://github.com/xArm-Developer/xarm_ros/tree/master/xarm_description). I don't enjoy dealing with xacros in general, as it forces you to use the ROS ecosystem. At the very least, it forces you to spend time on compiling everything and producing URDFs.

The model provided by them has a few problems as well. The collision meshes provided aren't particularly good. For collisions, you typically want low poly conservative approximations of the actual meshes. But some of their geometries aren't really clean: 

<br />
{{< figure src="/posts/7/images/lite6_collision_mesh.png" alt="lite6_collision_mesh" >}}
<br />

But the primary problem, is the model of the parallel gripper. The actual hardware has a parallel gripper actuated by a linear actuator. It's also open loop, in the sense that you cannot control the actual positions of the prismatic joint. The gripper can be opened or closed and nothing else in between. This poses a problem in software, but we'll get to that later. For now, the problem is in the description file. The URDF of the robot (after compiling it from the xacro), does not have actuated parallel grippers. It doesn't even have the parallel gripper links, which is what we would need to simulate any kind of manipulation task.

The only other model of the robot that I could find (at least as of writing this) is this [MJCF from DeepMind's MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/ufactory_lite6). Unfortunately, this one doesn't have a gripper either, and utilizes the same collision meshes. Here's a screenshot from their GitHub page:

<br />
{{< figure src="/posts/7/images/mujoco_lite6.png" alt="mujoco_lite6" >}}
<br />

So, we have our work cut out for us. We'll need to make a few changes to the model, before we can actually use it for anything useful. This is usually going to be the case with most models, as you'll need to make changes to suit your particular application.

### Collision Meshes

Most simulators don't support collision checks for non-convex meshes/geometries as this is more challenging to compute and is more expensive than doing it for convex shapes. Given non-convex meshes, most simulators will either try to approximate the mesh by a set of convex meshes, or compute a convex hull of the mesh. Ideally, we'd like to control the behavior, than let the simulator do it internally.

For our case, we can take one of two approaches:
1. Manually compute a low poly representation of each link as a collection of low poly convex meshes
2. Represent each link as a collection of basic geometries (cuboids, cylinders, ellipsoids, etc.)

The second option is the more computationally tractable one and makes it so that you don't have to deal with the original meshes at all (other than using them as a reference). It has a disadvantage of being more conservative and increasing the likelihood of spurious collisions, but we can be a bit smart about how we build these geometries by not incorporating parts of the meshes that are extremely unlikely to collide with anything.

Here's an example of what this process looks like for a particular link on the robot.


<br />
{{< figure src="/posts/7/images/lite6_collision_mesh_approx.png" alt="lite6_collision_mesh_approx" >}}
<br />


### Parallel Gripper Model

To model the gripper, we do the following:

1. Add the two parallel gripper links according to the dimensions of the actual gripper. The actual prongs of the gripper are rectangular, which makes it easy to specify the visual and collision geometries
2. Add joints to specify the max and min gripper movement, along with actuators to enable simulation. We also make sure to set the effort and velocity limits (taken from empirical measurement using the real hardware) and specify inertial properties as well. The mass in this case will need to be approximated as there's no real way of measuring this without taking the gripper apart

We could model this using a single actuator, with a URDF mimic joint on the other actuator, but Drake doesn't play nicely with this setup. So we use two actuators and deal with any complications through software.

The gripper can also be installed in two ways: normal and reverse. The normal configuration has the gripper width go from 0 to 1.6 cm, while the reverse setup as it go from 2.2 cm to 3.8 cm. As mentioned before, the gripper specs are very limiting, but we try to make the best approximation of the actual hardware.

After modelling, this is what the gripper's reverse configuration looks like:


<br />
{{< figure src="/posts/7/images/gripper.png" alt="gripper" >}}
<br />


### Inspection

We can now put everything together, visualize the robot in Drake and run some final inspections. The first thing we check is that the each link's coordinate frame is placed and oriented correctly.

<br />
{{< figure src="/posts/7/images/lite6_drake_viz_frames.png" alt="lite6_drake_viz_frames" >}}
<br />

Next, we check what our custom collision meshes look like.

<br />
{{< figure src="/posts/7/images/lite6_drake_viz_collision1.png" alt="lite6_drake_viz_collision1" >}}
<br />

We can see how the basic geometries make up for a clean approximation of the original meshes. Finally, Drake also allows us to visualize the inertia tensors of each link as an ellipsoid. This can help us verify that the inertial parameters provided in the original description by the manufacturer is in fact correct.

<br />
{{< figure src="/posts/7/images/lite6_drake_viz_inertia.png" alt="lite6_drake_viz_inertia" >}}
<br />

It's easy to get these values wrong, and is equally difficult to actually fix them if they're incorrect. But in this case, they seem to be accurate. Lucky!

### Drake Specific Setup

Finally, we need to add certain Drake specific information to the URDF file. Drake extends the description file format to include additional tags that it can use to better model the dynamics of robots. Details regarding this can be accessed [here](https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html)

1. We add the `drake:declare_convex` tag to each collision geometry mesh. This tells Drake that it can assume the mesh to be convex and not have to do any convex hull computations
2. For URDFs, Drake doesn't parse the `mechanicalReduction` tags of an `actuator`. Instead, to specify the gear ratio, we add the `drake:gear_ratio` tag
3. For motors with a large gear ratio, the inertia of the rotors can significantly affect the accuracy of the model. If these values are known, they can be added using `drake:rotor_inertia`. Unfortunately, these are not known for the Lite6 manipulator, so I've not included them in any of the models
4. To enable hydroelastic contact, we need to add `drake:proximity_properties` to each model/link that can make contact. More on this in the contact modelling section

<br />

We now have a description model that can be used for all sorts of simulations. My version of the models with compiled URDFs can be found [here](https://github.com/shrenikm/robot_models/tree/main/lite6_description). I have models for each of the following setups:
* Manipulator without gripper
* Manipulator with normal parallel gripper
* Manipulator with reverse parallel gripper
* Manipulator with vacuum gripper
* Separate normal parallel gripper model (without the rest of robot)
* Separate reverse parallel gripper model (without the rest of robot)

There are also separate models for Drake and non-Drake use. Each model includes the simplified collision geometry meshes, along with OpenCAD files containing the CAD source for the simplified collision geometry.



## References

1. **Russ Tedrake, & the Drake Development Team. (2019). Drake: Model-based design and verification for robotics.**
