# Self Driving Skateboard Ramp

[![Image of ramp](https://makeartwithpython.s3.us-west-2.amazonaws.com/ramp-open.jpg)](https://www.makeartwithpython.com/blog/building-a-remote-controlled-skate-ramp/)

[![Open in Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/#https://github.com/burningion/self-driving-skate-ramp)
[**See the blog post**](https://www.makeartwithpython.com/blog/building-a-remote-controlled-skate-ramp/)

## The Inspiration

I take my dog on runs with my skateboard. While skating around the block, I've always wanted to have ramps to hit. So one day I thought, what if the ramps came with me to where I was going? 

What if we had a mobile skatepark?

## How it Works

This project is inspired by the [Jetracer](https://github.com/NVIDIA-AI-IOT/jetracer) project, but is built on an electric skateboard platform, and doesn't actually use any of the code from it.

For steering, it uses a [linear actuator](https://amzn.to/3WPIkHX) hooked up to the truck of the skateboard with an [electric skateboard motor mount](https://amzn.to/40eZ0eV). My original prototype attempted to use a right and left motor to control steering, but this just didn't work at all. (The ramp just kind of jumped around.)

The linear actuator is attached to the board with two 1/4" steel plates (originally shelves from Home Depot), which are bolted through to board to a 1/4" steel plate on the other side. Originally I had screws attaching the linear actuator, but these almost immediately pulled out.

To control the actuator, I use a [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) and two relays. I wired the relays so that the polarity of electricity can be reversed. (So positive can become negative, and vice versa.)

My original prototype of the ramp just had small enough of clearance from the ground to be skateable. This proved terrifying, as the ramp was unstable on two axis at the same time.

As an improvement, I added an [electric jack](https://amzn.to/3wKzsse) to raise and lower the ramp. When I want to hit the ramp, I lower it to the ground, so it's stable like a normal skate ramp. When I'm done and want it to move, I raise the jack back up.

The raising and lowering of the jack works in the same way as the linear actuator. I have two relays wired up to again reverse the polarity. These are again, connected to the Teensy, and the two pins can either be set HIGH or LOW, to raise or lower the ramp itself.

## Communication and Control

To control the self driving skate ramp, I use a [Playstation 4 controller](https://amzn.to/40ngJ3C) and the great [pyPS4Controller](https://github.com/ArturSpirin/pyPS4Controller) library. 

The pyPS4Controller library allows me to define which button presses do what. For now, I've got both of my electric skateboard motors hooked up via USB. I use the [PyVESC](https://github.com/LiamBindle/PyVESC) library to send commands, in kind of a non-deal way. If you look at the code, you'll see I'm doing a bunch of `time.sleep(.1)` in between sending commands. This should be improved with a more rigourous fix, but without the sleeps the program crashes.

Pairing the controller with Linux over Bluetooth can be a challenge. The Jetson Nano has a built in bluetooth module supported by Linux, but you'll need to use the `bluetoothctl` tool to pair. Instructions are in the [`ps4-controller.md`](https://github.com/burningion/self-driving-skate-ramp/blob/main/ps4-controller.md) file.

## What's Next

Given the description of the project is a self driving ramp, this isn't actually self driving. Ideally we'll get to a point where gestures, voice, or watch controls allow me to dictate what the ramp does, and it otherwise just follows me. (I aim to have a fleet of these.)

## Accompanying Blog Post

Lives at [makeartwithpython.com](https://www.makeartwithpython.com/blog/building-a-remote-controlled-skate-ramp/)
