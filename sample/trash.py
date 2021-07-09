        # Remove background - Set pixels further than clipping_distance to grey
        #grey_color = 153
        #depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        #bg_removed = np.where( (depth_image_3d > clipping_distance) | (depth_image_3d <= 0) , grey_color, color_image)