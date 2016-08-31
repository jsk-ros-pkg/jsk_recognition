#GrabCut
Input a original image an seed to foreground/background to obtain foreground and background image
![](image/grabCut.png)

## Subscribing Topic
* `~input` (`/multisense/left/image_rect_color`)
* `~input/foreground` (`/multisense/left/image_rect_color/foreground`)
* `~input/background` (`/multisense/left/image_rect_color/background`)

## Publishing Topic
* `~image` (`/grabcut/output/foreground`)
* `~image` (`/grabcut/output/background`)
* `~image` (`/grabcut/output/foreground_mask`)
* `~image` (`/grabcut/output/background_mask`)