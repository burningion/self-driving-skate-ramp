# Self Driving Skateboard Ramp

[![Image of ramp](https://makeartwithpython.s3.us-west-2.amazonaws.com/ramp-open.jpg)](https://www.makeartwithpython.com/blog/building-a-remote-controlled-skate-ramp/)

[**See the blog post**](https://www.makeartwithpython.com/blog/building-a-remote-controlled-skate-ramp/)

## The Inspiration

I take my dog on runs with my skateboard. While skating around the block, I've always wanted to have ramps to hit. So one day I thought, what if the ramps came with me to where I was going? 

What if we had a mobile, autonomous skatepark?

## How it Works

This project is inspired by the [Jetracer](https://github.com/NVIDIA-AI-IOT/jetracer) project, but is built on an electric skateboard platform, and doesn't actually use any of the code from it.

For steering, it uses a [linear actuator](https://amzn.to/3WPIkHX) hooked up to the truck of the skateboard with an [electric skateboard motor mount](https://amzn.to/40eZ0eV). My original prototype attempted to use a right and left motor differences to control steering, but this just didn't work at all. (The ramp just kind of jumped around.)

The linear actuator is attached to the board with two 1/4" steel plates (originally shelves from Home Depot), which are bolted through to board to a 1/4" steel plate on the other side. Originally I had screws attaching the linear actuator, but these almost immediately pulled out from the forces of steering the board.

To control the actuator, I use a [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) and two relays. I wired the relays so that the polarity of electricity can be reversed. (So positive can become negative, and vice versa. This allows for extending or retracting the actuator.)

My original prototype of the ramp just had small enough of clearance from the ground to be skateable. This proved terrifying, as the ramp was unstable on two axis at the same time.

As an improvement, I added an [electric jack](https://amzn.to/3wKzsse) to raise and lower the ramp. When I want to hit the ramp, I lower it to the ground, so it's stable like a normal skate ramp. When I'm done and want it to move, I raise the jack back up.

The raising and lowering of the jack works in the same way as the linear actuator. I have two relays wired up to again reverse the polarity. These are again, connected to the Teensy, and the two pins can either be set HIGH or LOW, to raise or lower the ramp itself.

## Communication and Control

To control the self driving skate ramp, I use a [Playstation 4 controller](https://amzn.to/40ngJ3C).

I originally used the great [pyPS4Controller](https://github.com/ArturSpirin/pyPS4Controller) library, but wasn't able to do heartbeat on the controller being connected, so I changed to raw hid input, allowing me to do my own heartbeat.

Similarly, I started with the PyVESC library, but now encode and parse the VESC protocol itself via separate threads and short timeouts, with the goal of not blocking anything.

Pairing the controller with Linux over Bluetooth can be a challenge. The Jetson Nano has a built in bluetooth module supported by Linux, but you'll need to use the `bluetoothctl` tool to pair. Instructions are in the [`ps4-controller.md`](https://github.com/burningion/self-driving-skate-ramp/blob/main/ps4-controller.md) file.

## What's Next

Given the description of the project is a self driving ramp, this isn't actually self driving yet.

Ideally we'll get to a point where gestures, voice, or watch controls allow me to dictate what the ramp does, and it otherwise just follows me. (I aim to have a fleet of these.)

## Accompanying Blog Post

Lives at [makeartwithpython.com](https://www.makeartwithpython.com/blog/building-a-remote-controlled-skate-ramp/)

## Adding Self-Driving Capabilities

Right now I'm in the very first stages of building a platform that can navigate forwards and backwards on a sidewalk, autonomously adjusting left and right using the linear actuator and an IMU on the Realsense 455 I've got.

For this I've built a camera.py that records data into `.rrd` rerun files. I plan to run inference and test if we can't keep the ramp in place.

