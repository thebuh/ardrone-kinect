ardrone-kinect
==============

Simple app for controlling ARDrone using Kinect written in Processing 

## Requirements
Libraries required for sketch to function 
are SimpleOpenNi and [ArDroneForP5](https://github.com/shigeodayo/ARDroneForP5)

## Preparation
Before launching sketch you have to plug in Kinect sensor 
and connect to ArDrone wifi network.

## How to fly

To take off press "Enter" to land press "Backspace"
Kinect tracking left and right hands movement relative to your neck 
- so to fly forwards you have to move arms forward to fly 
backwards you have to move arms backward.
Same with up and down. To fly left and right you have to move one hand up 
and second hand down.  

## Known bugs
Sometimes sketch losing kinect input altogether.

## Todo
Take off and landing gestures - to get rid of having to press a button

Graceful handling of second skeleton in frame.