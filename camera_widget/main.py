#!/bin/bash

import traitlets
from jetbot import Camera, bgr8_to_jpeg

camera = Camera.instance(width=300, height=300)

image_widget = ipywidgets.Image()
camera_link = traitlets.dlink((camera, 'value'), (image_widget, 'value'), transform=bgr8_to_jpeg)
display(image_widget)

def execute(change):
    global image_widget
    image = change['new']
    
execute({'new': camera.value})
camera.unobserve_all()
camera.observe(execute, names='value')

# time.sleep(1)
# camera.unobserve(execute, names='value')
# camera.stop()
