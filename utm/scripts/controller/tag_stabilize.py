"""
Stabilize fiducial tag target

- Find attitude of quad in roll and pitch
- Subscribe to position of tag
- Compute dx and dy as follows assuming ENU frame:
    - distorted_x = tag_x - tag_x*sin(att_angle)
    - distorted_y = tag_y - tag_y*sin(att_angle)

"""