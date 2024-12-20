# pybullet_controller
This component controls the PyBullet simulator and offers the KinovaArm interface.

## Configuration parameters
As any other component, *pybullet_controller* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <pybullet_controller's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/pybullet_controller config
```
