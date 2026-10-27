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

Select the camera stream in the configuration file with `Camera.stream`.
Supported values are `both`, `color`, and `depth`; the default is `both`:
```
Camera.stream = depth
```

The RTSP pipelines use one-frame leaky queues and do not synchronize the
appsink to the playback clock, so consumers receive the newest available
frame instead of waiting on buffered frames. If the depth connection drops,
the component retries with a short backoff (starting at 250 ms and capped at
2 seconds). The depth pipeline leaves RTSP transport negotiation to GStreamer,
matching the working standalone pipelines for both streams.
RTSP keep-alives are enabled explicitly because the Gen3 user guide documents
a 30-second stream inactivity timeout.

The Gen3 guide specifies the depth stream as Z16: 16-bit little-endian depth
samples transferred as grayscale. The pipeline preserves this as `GRAY16_LE`,
and `CameraRGBDSimple.getDepth` publishes the raw samples with
`depthFactor = 0.001` meters per unit, matching this component's millimeter
interpretation. Depth samples are not clipped when held or returned by
`getDepth`/`getAll`; the 0-5 m range is applied only to the displayed heatmap.
The guide also notes that the depth unit is configurable in meters; if that
setting is changed on the robot, `depthFactor` must be updated to match it.
Zero-valued (invalid/unmeasured) pixels are displayed as the maximum distance,
so they appear red with the JET palette.
