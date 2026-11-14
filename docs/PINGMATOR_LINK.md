# Pingmator link

The Pingmator link shows the pose you are making in
[Pingmator](https://github.com/pingledingle-tech/Pingmator), an animation
editor for Second Life, on your own avatar in Aurora while you work. The pose
goes through Aurora's local poser: only you see it, nothing is uploaded and
nothing is sent to the region.

## Use it

1. In Aurora, choose **Build › Pingmator link**. The menu entry gets a tick
   and the chat says the link waits for Pingmator.
2. In Pingmator, choose **Options › Live link to Aurora**. The title bar says
   "live in Aurora" once the two are connected, and the chat in Aurora says
   Pingmator connected.
3. Pose a joint or move the frame slider in Pingmator: the Poser window opens
   in Aurora and your avatar takes the pose. Playing the animation in
   Pingmator plays it on your avatar too.

Choose **Build › Pingmator link** again to turn it off. It is off every time
Aurora starts. Stopping the pose in the Poser window gives your avatar back
its normal animations.

Each burst of changes from Pingmator is one undo step in the Poser.

## How it works

Aurora listens on `127.0.0.1`, port **17656**, only while the link is on, and
only programs on the same computer can connect. One client at a time; it
does not share the LiveMesh port, so Blender and Pingmator can be connected
together.

The messages are lines of JSON (protocol version 1):

| From | Message |
|---|---|
| Aurora | `{"type":"hello","app":"Aurora","protocol":1,"features":["pose"]}` |
| Pingmator | `{"type":"hello","app":"Pingmator","version":"…"}` |
| Pingmator | `{"type":"pose","bvh":"HIERARCHY …"}`: a BVH of 3 frames, the first neutral, then the pose twice |
| Aurora | `{"type":"error","message":"…"}` when it cannot read a pose |

The BVH is read like a pose file loaded in the Poser (`parse_bvh_pose`), on
the link's own thread; the frame only takes the newest pose.

## Not done yet

- Play, pause and the frame position following each other in both programs.
- Your avatar's shape sent to Pingmator.
- Poses made in Aurora's Poser sent back to Pingmator.
