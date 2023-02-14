# Pairing the PS4 Controller with the NVIDIA Jetson Nano

It's not obvious how to pair a PS4 controller with the Jetson Nano. Here's how I've done it: 

```
$ sudo bluetoothctl

agent on
discoverable on
pairable on
default-agent

scan on
```

Press and hold Share and PS button until controller flashes light (It'll double blink)

Wireless Controller shows up, type the next commands in the same session:

```
connect CONTROLLER_MAC
trust CONTROLLER_MAC
```

If the light stays lit, you've successfully paired. You can now exit the bluetoothcl command and run the `control_steering.py`.

## Re-Pairing the Controller

Once I've successfully paired a controller, I've found it difficult to repair afterwards. (IE After a reboot.) In this case, you need to do a:

```
remove CONTROLLER_MAC
```

And then redo the above steps, but instead of `connect CONTROLLER_MAC`, do a `pair CONTROLLER_MAC`, and wait for Bluetooth to ask you if you want to authorize the controller. Once it's authorized, you can then exit and run the program.