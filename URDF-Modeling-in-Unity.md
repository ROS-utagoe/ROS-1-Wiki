###### tags: `ROS` `Unity` ` urdf`

URDF Modeling in Unity
===

# Create a robot
## base_link
`base_link` is namely the **base** link of a robot.
All links are represented as a relative transform to `base_link`.
Generally it doesn't have Visuals or Collisions.

1. Select `3D Object > URDF Model (new)` to create a new URDF model object.
2. In the `Hierarchy`, select `base_link` and open `Inspector` tab. Select `Child Joint Type` and click `Add child link (with joint)`.

 ![](https://i.imgur.com/05xU6uN.png) ![](https://i.imgur.com/UFZIzMm.png)

 Let's add a new child link whose joint type is `Fixed`.
 You'll see a new child link named `link` by default is genetrated. Let's rename it to `body_link`.

## body_link
`body_link` is a main frame of a robot.

### Visuals
In the `Hierarchy`, select `body_link > Visuals`. You'll see `Urdf Visual (Script)` in `Inspector` tab. Select `Type of visual` and click `Add visual`

![](https://i.imgur.com/dZ3Ztos.png)

Let's add visual whose type of visual is `Box`.
You'll see a box in the Scene.

### Transform
Here we describe the points to note to transform.

#### Position
If you want to move the **whole** body object, ++be sure to select `body_link`++, not `body_link > Visuals > unnamed` though the name of the actual box you see is `unnamed`.


This is because the position of a link, `Visuals` and `Collisions` must be the same.
Both `Visuals` and `Collisions` are child objects of the link so the positions will be different if you move only `Visuals` object.

#### Rotation & Scale
In contrast to Position, ++you must select `link > Visuals > unnamed`++ to change Rotation and Scale.
This is because the values of Trasform of `unnamed` represents the relative transform to the link.

### Collisions
Adding Collissions is quite simple if you transform the visual object correctly.

Select `body_link > Visuals > unnamed` and see `Urdf Visual (Script)` in `Inspector` tab.
Just click `Add collision to match visual` to generate the collision object.

![](https://i.imgur.com/8zfWTKa.png)


## right/left_wheel_link
Open `Inspector` tab of `body_link` and select ++`Continuous`++ for `Child Joint Type` in `Urdf Link (Script)` and add a child link.

![](https://i.imgur.com/N7Lc1DN.png)

Let's add two children for right/left wheels.

Select `right_wheel_link > Visuals` and add visual whose type is `Cylinder`.

![](https://i.imgur.com/gNSqZyT.png)

Change Transform to proper position.

## Caster
In order to realize a **caster** on your robot, it is recommended to use a slippery box instead of a real caster because it is costly to implement a real caster with URDF model.
