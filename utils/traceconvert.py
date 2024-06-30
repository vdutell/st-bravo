import numpy as np
import cv2
import os
import matplotlib.pyplot as plt

def adjust_gamma(img, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to adjusted gamma values
    table = np.array([((i / 255.0) ** (1/gamma)) * 255
    for i in np.arange(0, 256)]).astype('uint8')
    # apply gamma correction using the lookup table
    return cv2.LUT(img, table)

def convert_positions_rsrgb_ximea(png_folder, data_folder, ana_folder, pup_pos,
                                  ximea_idx, rsrgb_idx):
    '''
    Because we calibrated in rsrgb space, we need to convert to ximea frame space to find fixation in ximea frames
    '''
    
    #get indexes for sample
    tidx = len(ximea_idx)//2 #middle frame
    ximea_frame = int(ximea_idx[tidx])
    rsrgb_frame = int(rsrgb_idx[tidx])
    
    #get pupil positions
    pos_x = pup_pos[:,0]
    pos_y = pup_pos[:,1]

    #get frames
    ximea_file = os.path.join(png_folder, f'frame_{ximea_frame}.png')
    rsrgb_file = os.path.join(data_folder, f'world.mp4')
    ximea_frame = cv2.imread(ximea_file)
    #get rsrgb frame from movie
    capture = cv2.VideoCapture(rsrgb_file)
    capture.set(cv2.CAP_PROP_POS_FRAMES, rsrgb_frame)
    _, rsrgb_frame = capture.read()
    
    #upsample realsense RGB Image
    rsrgb_frame = cv2.cvtColor(rsrgb_frame, cv2.COLOR_BGR2RGB)
    rsrgb_frame_adj = cv2.resize(rsrgb_frame, dsize=((np.shape(ximea_frame)[:2])[::-1]))
    
    #adjust ximea brightness so SIFT hopefully works better
    ximea_frame = cv2.cvtColor(ximea_frame, cv2.COLOR_BGR2RGB)
    ximea_frame_adj = adjust_gamma(ximea_frame, gamma=5)
    
    #run SIFT
    sift = cv2.xfeatures2d.SIFT_create()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(rsrgb_frame_adj,None)
    kp2, des2 = sift.detectAndCompute(ximea_frame_adj,None)
    
    # FLANN parameters to pick good points only
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5) #5 default
    search_params = dict(checks=1000)
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    good = []
    pts1 = []
    pts2 = []
    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.6*n.distance:
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)
    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    
    #save a plot of the point matches to make sure they are reasonable.
    rsrgb_frame_points = rsrgb_frame_adj.copy()
    ximea_frame_points = ximea_frame_adj.copy()

    #calcualte Homogophy Map
    H = cv2.findHomography(pts1, pts2)
    #test applying homogrophy to img
    rsrgb_transformed = cv2.warpPerspective(rsrgb_frame_adj, H[0], np.shape(rsrgb_frame_adj)[:2][::-1])

                               
    def drawpoints(img1,img2,pts1,pts2):
        ''' img1 - image on which we draw the epilines for the points in img2
        '''
        r,c,_ = img1.shape
        for pt1,pt2 in zip(pts1,pts2):
            color = tuple(np.random.randint(0,255,3).tolist())
            img1 = cv2.circle(img1,tuple(pt1),20,color,-1)
            img2 = cv2.circle(img2,tuple(pt2),20,color,-1)
        return img1,img2

    img5,img6=drawpoints(rsrgb_frame_points, ximea_frame_points, pts1, pts2)
    plt.close()
    plt.figure(figsize=(22,6))
    plt.subplot(131),plt.imshow(img5),plt.title('RSRGB Frame')
    plt.subplot(132),plt.imshow(img6),plt.title('Ximea Frame')
    plt.subplot(133),plt.imshow(rsrgb_transformed),plt.title('Ximea Frame')
    plt.savefig(os.path.join(ana_folder, 'rsrgbToXimPoints.png'))


    #calculate Upsample effect on and eye positions
    upsample_ratio_y = rsrgb_frame_adj.shape[0]/rsrgb_frame.shape[0]
    upsample_ratio_x = rsrgb_frame_adj.shape[1]/rsrgb_frame.shape[1]
    pos_x_up = pos_x * upsample_ratio_x
    pos_y_up = pos_y * upsample_ratio_y
    positions_upsampled = pos_x_up, pos_y_up, np.repeat(1,len(pos_x_up))

    #apply transform matrix to pupil positions
    trans_pupil_pos = np.rint(H[0]@(positions_upsampled))
    
    #take only first two cols (it was 1 padded), and put back in correcrt dims with transpose
    trans_pupil_pos = trans_pupil_pos[:2].T
    
    #we're done!
    return(trans_pupil_pos)


def avg_left_right(trace_array):
    '''
    average the left and right eye positions (in x and y) to get a singular trace
    Params:
        trace_array (array of floats): 2D array of floats with columns {LEFT_X,LEFT_Y,RIGHT_X,RIGHT_Y}
    Returns:
        trace_array: 2D array of floats with columns {X, Y}
    '''
    x = np.mean(np.array((trace_array[:,0], trace_array[:,2])),axis=0)
    y = np.mean(np.array((trace_array[:,1], trace_array[:,3])),axis=0)
    trace_array = np.array((x,y)).T
    return(trace_array)

def process_lr_trace_criterion(trace_array, string_key):
    '''
    For the pilot data, some traces have problems with a single eye. If we have two good eyes, average the two to get better data, otherwise use the good eye. 
    Take in a 4xtimestamps trace_array, and process it with the string key:
    'a': data is good. take the average of two eyes
    'l': right eye is poor. Take the left eye trace data
    'r': left eye is poor. Take the right eye trace data
    
    Params:
        trace_array (array of floats): 2D array of floats with columns {LEFT_X,LEFT_Y,RIGHT_X,RIGHT_Y}
        string_key (char): 'a','l', or 'r' (see above)
    Returns:
        trace_array: 2D array of floats with columns {X, Y}
    '''
    
    if(string_key=='a'):
        trace_array = avg_left_right(trace_array)
    elif(string_key=='l'):
        trace_array = trace_array[:,:1]
    elif(string_key=='r'):
        trace_array = trace_array[:,2:]
    else:
        print('ATTENTION!!! String_key is {string_key}. This is not a, l, or r so I dont know what to do with the trace array!')
        #print('Taking the mean of x and y positions, but make sure this is correct!')
        #trace_array = avg_left_right(trace_array)
    return(trace_array)