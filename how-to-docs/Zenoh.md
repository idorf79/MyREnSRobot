# Zenoh

## Start on the Robot

On robot:

```bash
zenoh-bridge-ros2dds -n <namespace>
```

'/n' defines the namespace

Example:

```bash
zenoh-bridge-ros2dds -n /rens02
```

## Start on the Controller

On Controller:

```bash
zenoh-bridge-ros2dds -n <namespace> -e tcp/<ip-of-robot>:<port-number>
```

'/n' defines the namespace

Example:

```bash
zenoh-bridge-ros2dds -e tcp/10.10.45.108:27447 -n /rens02
```

If you use port forwarding, make sure it's using 'TCP'.
