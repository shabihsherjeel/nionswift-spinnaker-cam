# nionswift-spinnaker-cam
An example repo demonstrating how to implement a Flir cam in Nionswift.

BOILERPLATE code guide:

The most vital line is line:306 in CameraDeviceSpinnaker.py

You set the xdata object and that should be all. Of course, there are some identifiers you need to set. 

I have a PyspinWrapper class that I use to call the camera's lib functions. 

You can search use them as indicators of what I have added to the base class.
