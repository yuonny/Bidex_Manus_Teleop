## SteamVR for Robotics
In the Bidex paper we use [SteamVR Vive Trackers](https://www.vive.com/us/accessory/tracker3/) as a baseline against the teacher arms.  To set this up we follow the [guide here](https://gist.github.com/DanielArnett/c9a56c9c7cc0def2064z480bca1f6772) and make small edits in our copy seen in `steamvr.md`.  We also copy `triad_openvr` repository since it is quite old and could get deleted.

You can also use this on Windows install Steam/SteamVR normally instead of following the Ubuntu guide.  Then use openvr and triad_openvr like in Ubuntu.

Run tracker_test.py and the data from the trackers will be printed to the terminal.  Then you need to run IK for your target robot arm.
