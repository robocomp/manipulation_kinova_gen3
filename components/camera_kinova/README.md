# camera_kinova
Intro to component here


## Configuration parameters
As any other component, *camera_kinova* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

To show the color and depth images in the component window, add this setting to
the configuration file:
```
Display.display = true
```
If the setting is omitted or set to `false`, the window and image display are
disabled; the camera streams remain available through the component interface.

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <camera_kinova's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/camera_kinova config
```
