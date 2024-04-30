# Space Simulator In Unity

![image](https://github.com/Jacob19999/Space_Simulator/assets/26366586/a4be09d7-ec7c-4311-acc0-f5f90dcfc51f)

![image](https://github.com/Jacob19999/Space_Simulator/assets/26366586/67bb5fef-4ff9-4045-be7c-020600aeb7d5)(https://www.youtube.com/watch?v=tQL2w4cLmN0)

# Core Components Repo

Physically based atmosphere  

https://github.com/Jacob19999/unity_physically_based_atmospheric_shader

Orbital Mechanics System

https://github.com/Jacob19999/unity_n-body_runge-kutta

Spaceship Physics System

https://github.com/Jacob19999/unity_inertia_tensor

Modular Ship Construction System

https://github.com/Jacob19999/unity_spacecship_attachment_system/tree/main

Spaceship Control System

https://github.com/Jacob19999/unity_spaceship_control_system/tree/main

# Goal 

The overarching goal of this project is to familiarize with the math and physics aspect of game design, where it governs how objects in a game scene moves and rotates upon the framework of a space simulation game. This is an attempt to demonstrate and bridge the gap between a Computer science major and an informational technology major, the “math” part. Hence endangered orbit's foremost emphasis is on realistic physics, which implements the real scale to those found in the cosmos resulting in a level of depth shall be deeper in many ways compared to the kerbal space program, in the regime of spacecraft movement physics as well as orbital mechanics. The next focus of the game shall be graphical fidelity, involving implementation of mathematical compute shaders that run natively on the GPU as post processing techniques, basically similar to “ray tracing”.

# Overview implementation

There are several constraints that arise from the decision above when developmenting in unity. Unity is built as a cross platform game engine that has an emphasis on small to mid scale first person style combat or mobile games, hence the infrastructure is only meant for those games. In order to make it work as described above, significant modifications have to be made under the hood. Below are areas of challenges.

# Scale of the planetary system

There are two types of scale, one being scale in terms of the pure distance between objects, where the average distance between planets is represented by millions of KM. Secondly the difference in scale of objects where a spaceship can be millions of times smaller in scale than a planetary body. Both these extremes are not supported by unity’s engine. Essentially the problem with unity is precision. Suppose if we want to represent the location of planets and spaceship over a distance over billions of kilometers, using unity’s object transform system, the object’s position being stored in single precision floating point number, we will find ourselves running out of precision, there aren’t enough precision to store the position of a ship. Additionally, the precision in gamespace is not evenly distributed, in unity, much more precision is given to objects closer to the game origin, while lower precision is given to objects further from the game origin. From testing, it seems that beyond 10,000 units or meters from the game origin, precision becomes insufficient. This has probably the greatest impact on unity’s rendering pipeline using vertex positions derived from a ship's position, resulting in errors from floating point rounding, causing vertex positions to jitter rapidly. 

# Scale of Objects

Object scaling is a major issue. If we want to have the spaceship in the same scene as the celestial bodies, we would need to scale a spaceship to millions of times smaller. This once again compounds to the fact that rendering pipeline shaders uses scaling to determine vertex positions, and causes significant jittering due to low precision. Additionally scale determines other properties related to rigid bodies like inertia where calculation is done assuming that one unit in game is 1 meter in real life. 

Several methods are used to solve this issue of precision and scale. This is a 3 prong approach, which are double precision math, floating origin and separate scenes. This allows us to somewhat handle all physics, like position outside of the game, then with various transformations, we can transform game objects from world position (actual physical position) into editor position, where those objects can be scaled and rendered. Floating origin is another set of transformations that transforms the default cartesian grid to be playercentric,  where the player is always in the center of the game origin; this would solve the unequal precision issue. Finally it is impossible to have all objects on the same scene, hence we shall need to use different scenes to render each component separately then composited together into the final game. This is important since perspective, the sense of scale is achieved here. 

# Realistic behavior of Spaceship

![image](https://github.com/Jacob19999/Space_Simulator/assets/26366586/15eb9337-8f23-4f49-b93d-7e573245f63b)

In our game we decided to add the feature of removable and attachable ship parts, each with its own unique shape, weight and set of Reaction control (RCS) thrusters. In order to achieve that and still have the playership behaving in a realistic manner, e.g asymmetric thrust , unity’s rigidbody system built for singular objects does not have the capability to do that. The main issue is that multiple objects parented together does not have a combined inertia, whereas they are treated as a singular object with the weight of all the objects but shaped like the parent object, resulting in unrealistic behaviors. 
	 
In order to achieve realistic ship behaviors,no matter what the player attached together. We recreated unity’s rigidbody system externally while rescuing some of the default rigidbody features to get properties of each object/modules, and combined together to obtain a composite tensor, which allows us to mathematically represent how a object like a ship react to external forces like those from a thrusting engine or a reaction control system.

# Control Of Spaceship

![image](https://github.com/Jacob19999/Space_Simulator/assets/26366586/97c10553-b3fc-4812-9c54-b7c2329f3db7)

Control of a spacecraft in space is a non-trivial task. In space, unlike aircraft, players have to control the spacecraft in all 6 axes, these are translational x,y,z and rotational x,y,z axis. Translation governs how the spacecraft moves in 3 axis space, like forward, up , left and so on while rotational axis governs the pitch roll and yaw of the spacecraft much like an aircraft. Unity has no built in control, these movements are controlled by the firing of tens or even hundreds of thrusters located around the ship. It is impossible for a player to control how a ship moves by micromanaging how each thrusters fires.

We have created a hybrid solution that draws inspiration from various space games and real spacecrafts. Perhaps this is the only space game that uses this control scheme. At its core the player controls the spacecraft pitch and yaw via the mouse. The player simply points to where they want the ship to point, and an proportional integral and derivative system (PID) fires the required thrusters to achieve that orientation. Translational movement can be hence controlled by the keyboard, similar to character movement in other games. The spacecraft uses a 6 closed loop PID controller to control all of the spacecraft’s 6 axes of movement. 

# Modular Ship Construction


In our game, you can build spaceships like you're playing with building blocks in real-time. This idea was inspired by 'Captain Forever,' but we took it a step further by making it in 3D. Each part of the ship, which we call modules, was designed to easily connect with others, forming different shapes and functions. These modules can attach or detach from each other during gameplay. To build a ship you simply click to select a part, drag it over one of the orbs that show up, and then release.

Some problems that taking this idea to 3D created were figuring out where to 'float' a module when you're holding it and designing a control system that lets you manage flying your ship and building it at the same time.

![image](https://github.com/Jacob19999/Space_Simulator/assets/26366586/6e823e86-c267-4005-ae8f-53361b0c1f93)


