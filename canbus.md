# Controlling VESCs with CANBUS via UART (UART CAN forwarding)

Setting up my motors to work in master -> slave with CANBUS was especially challenging, as there really wasn't much on the internet about how to get CANBUS communication over UART working.

I ended first making a 4 pin 2.0mm pitch female to female adapter to connect the master VESC to the slave one. The wires should match the same pins on each VESC. Only the middle two wires are used.

Once I did that, I went into the VESC tool, and set the ID for the Master to 0. I then set the slave's ID to 1. Both of these are configurable via the App Setting.

Once I had that, I then found [a helpful forum post](https://vesc-project.com/node/774). This shows what the messages _should actually look like_.

I set up a new message for PyVesc with the following values:

```python
class SetRPM(metaclass=pyvesc.VESCMessage):
    """
    Sets the RPM on a CANBUS connected device. 
    Messages have to be sent repeatedly, as CANBUS has a timeout that's configurable
    within the VESC application. In my case, I set it to 5 seconds, which means the
    motor only works for 5 seconds after sending this signal.
    """
    id = 34
    fields = [
        ('motor_id', 'B'), # my slave is set to 1
        ('command', 'B'), # 8, thanks to this page: https://www.vesc-project.com/node/774
        ('rpm', 'i') # because we're assuming RPM setting
    ]
```

The Python [struct](https://docs.python.org/3/library/struct.html#format-characters) datatype let me set `B` for an unsigned byte, and `i` for a 32bit integer.

With this, I can do my debug statements in Python of the Bytes:

```python
values = list(encode(SetRPM(1, 8, FRONT_DRIVE_RPM)))
print(*values, "CANBUS MESSAGE")
```

And yes, I can write to the serial port like this:

```python
front_motor.write(encode(SetRPM(1, 8, FRONT_DRIVE_RPM)))
```

But, the CANBUS messages by default only get applied for 1 second! So you'll need to continously send your CAN messages!

You'll notice that we do a `list()` wrapped around our Bytes, and then unpack the list for better viewing. We can then match the values from the forum post. Create an issue on this Github repo if anything is still unclear.