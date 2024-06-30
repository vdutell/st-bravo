import pyrealsense2 as rs2
import rospy
import numpy as np
import rosbag
import os
import copy
from scipy.spatial.transform import Rotation
import cv2
import sys
import time

import utils.bins_to_pngs as btp

#VD 6/29/24: added these helper functions to keep running bag alignment until all the frames have been aligned
#this is needed due to bag alignment STILL dropping frames, but it drops a different set each time, and only 10%. 
#assuming there are 20,000 files per trial, we'll need to run this about 4-5 times to get them all, but should be fast on later runs. Not the most elegaant, but we spent MONTHS trying to fix this other ways. This at least works.
#we actually dont need this, we have the dpeth timestamps
def find_largest_framenum(depth_files_folder):
    #take off the last part of the path and go to the pngs folder
    max_frame = 0
    for f in os.listdir(depth_files_folder):
        num = f.replace('frame_','').replace('.png','')
        max_frame = max(max_frame,int(num))
    print('Total Trial Frames: ', max_frame)
    return(max_frame)
          
def list_missing_frames(expected_num_frames, aligned_frames_folder):
    #we have the depth timestamps.
    missing_frames_list = []
    for i in range(expected_num_frames):
        fname =os.path.join(aligned_frames_folder,f'depth_frame_{str(i).zfill(8)}.npy')
        #print(fname)
        if not os.path.isfile(fname):
            missing_frames_list.append(i)
    return(missing_frames_list)


## bug fix VD: fix timing syncronization issue by running a single file at a atime.
#We run this rersively via the align_dpeth_rsrgb_single funciton below this one.
def align_depth_rsrgb_single_framelist(frame_list, bag_in_path, bag_out_rgb_path, output_folder, aligned_rgb_depth_folder, depth_timestamps, depth_frames, depth_fps, depth_dims, rgb_dims):
   
    #loop through frames
    for i, (t, frame, framenum) in enumerate(frame_list):
    
        # .bag file for depth info
        try:
            bag_in = rosbag.Bag(bag_in_path)
            bag_out_rgb = rosbag.Bag(bag_out_rgb_path, 'w')
            #keep messages with metadata about RGB and depth devices
            for topic, msg, t in bag_in.read_messages():

                #if topic is about gyro or accelerirometer (sensor 2), remove it in our new file
                if 'sensor_2' in str(topic):
                    pass

                #don't need image or depth metadata
                elif 'Depth_0/image/metadata' in str(topic):
                    depth_metadata_msg = msg
                    if('Exposure Roi' in str(msg.key)):
                        bag_out_rgb.write(topic, msg, t)
                    else:
                        pass
                elif 'Color_0/image/metadata' in str(topic):
                    color_metadata_msg = msg
                elif 'Depth_0/image/data' in str(topic):
                    depth_data_msg = msg
                elif 'Color_0/image/data' in str(topic):
                    color_data_msg = msg 

                #if topic is depth stream info, ensure correct frame rate and size
                elif '/device_0/sensor_0/Depth_0/info' == str(topic):
                    msg.fps = depth_fps
                    bag_out_rgb.write(topic, msg, t)
                elif '/device_0/sensor_0/Depth_0/info/camera_info' == str(topic):
                    msg.height = depth_dims[1]
                    msg.width = depth_dims[0]
                    bag_out_rgb.write(topic, msg, t)

                #if topic is RGB stream info, ensure correct frame rate and size
                elif '/device_0/sensor_1/Color_0/info' == str(topic):
                    msg.fps = depth_fps  #rgb_fps #these are manually set to the same for this step.
                    bag_out_rgb.write(topic, msg, t)
                elif '/device_0/sensor_1/Color_0/info/camera_info' == str(topic):
                    msg.height = rgb_dims[1]
                    msg.width = rgb_dims[0]
                    bag_out_rgb.write(topic, msg, t)

                else:
                    #keep everything else
                    bag_out_rgb.write(topic, msg, t)

                #if topic is RGB extrinsics, save them so we can combine them with ximea to rgb extrinsics later to get ximea to depth exrinsics
                if '/device_0/sensor_1/Color_0/tf/0' == str(topic):
                    depth_to_rgb_translation_message = msg.translation
                    depth_to_rgb_rotation_q_message = msg.rotation

        except Exception as e:
            print(f'Failed to write rgb frame {i}:',e)

        finally:
        
            #fill up bag with depth & RGB frames
            depth_data_topic = '/device_0/sensor_0/Depth_0/image/data'
            color_data_topic = '/device_0/sensor_1/Color_0/image/data'
            depth_metadata_topic = '/device_0/sensor_0/Depth_0/image/metadata'
            color_metadata_topic = '/device_0/sensor_1/Color_0/image/metadata'

            #write frame info
            #depth_data_msg = copy.deepcopy(sample_depth_data_msg)
            timestamp = rospy.Time.from_sec(time.time())
            #time = rospy.Time(i+1)

            #color
            color_msg = copy.deepcopy(color_data_msg)
            color_msg.height = rgb_dims[1]
            color_msg.width = rgb_dims[0]
            color_msg.header.stamp = timestamp
            color_msg.header.seq = i
            color_msg.data = np.zeros(rgb_dims).tobytes()
            bag_out_rgb.write(color_data_topic, color_msg, timestamp)
            #metadata
            color_meta_msg = copy.deepcopy(color_metadata_msg)
            color_meta_msg.key = 'Frame Counter'
            color_meta_msg.value = str(i)
            bag_out_rgb.write(color_metadata_topic, color_meta_msg, timestamp)
            color_meta_msg = copy.deepcopy(color_metadata_msg)
            color_meta_msg.key = 'Frame Timestamp'
            color_meta_msg.value = str(t)
            bag_out_rgb.write(color_metadata_topic, color_meta_msg, timestamp)

            #depth
            depth_msg = copy.deepcopy(depth_data_msg)
            depth_msg.height = depth_dims[1]
            depth_msg.width = depth_dims[0]
            depth_msg.header.stamp = timestamp
            depth_msg.header.seq = i
            depth_msg.data = frame.astype('<h').tobytes()
            bag_out_rgb.write(depth_data_topic, depth_msg, timestamp)
            #metadata
            depth_meta_msg = copy.deepcopy(depth_metadata_msg)
            depth_meta_msg.key = 'Frame Counter'
            depth_meta_msg.value = str(i)
            bag_out_rgb.write(depth_metadata_topic, depth_meta_msg, timestamp)
            depth_meta_msg = copy.deepcopy(depth_metadata_msg)
            depth_meta_msg.key = 'Frame Timestamp'
            depth_meta_msg.value = str(t)
            bag_out_rgb.write(depth_metadata_topic, depth_meta_msg, timestamp)      
        
            bag_in.close()
            bag_out_rgb.close()
        
            print('*',end='')
            #print('Finished Creating Depth -> RGB Bag File')

        ###########
        #Now use realsense pipeline to read in frames from .bag to run align_to
        try:
            pipe = rs2.pipeline()
            cfg = rs2.config()
            cfg.enable_device_from_file(os.path.join(output_folder,'depth_rgb.bag'), repeat_playback=False)
            profile = pipe.start(cfg)
            playback = profile.get_device().as_playback()
            playback.set_real_time(False)
            
            align_to = rs2.stream.color
            align = rs2.align(align_to)
            #print(f'processing {len(depth_timestamps)} timestamps...')
            #while True:
            #for i in range(len(depth_timestamps)):
            frameset = pipe.wait_for_frames()
            #if(frameset is not None):
            #make sure we aren't starting over again
            aligned_frames = align.process(frameset)
            df = aligned_frames.get_depth_frame()
            #print(df.get_frame_number(), ts)
            aligned_depth_frame = np.array(df.get_data())
            cv2.imwrite(os.path.join(aligned_rgb_depth_folder,
                                    f'depth_frame_{str(framenum).zfill(8)}.png'),
                       aligned_depth_frame.astype(np.uint16))
            np.save(os.path.join(aligned_rgb_depth_folder,
                                  f'depth_frame_{str(framenum).zfill(8)}.npy'),
                                  aligned_depth_frame)
            #pipe.stop()
        except Exception as e:
            print(f'Failed to read rgb frame {i}:',e)

        #print('Finished Writing RGB Aligned Depth Files - Removing Bag file Now!')
    
    os.remove(os.path.join(output_folder,'depth_rgb.bag'))
    return(color_metadata_msg, depth_metadata_msg, depth_to_rgb_rotation_q_message, depth_to_rgb_translation_message)


#Here is where we check for the number of missing frames, then run this over and over until we get all the frames.
#See functions defined at the top of this file for more info on why we do this.
def align_depth_rsrgb_single(bag_in_path, bag_out_rgb_path, output_folder, aligned_rgb_depth_folder, depth_timestamps, depth_frames, depth_fps, depth_dims, rgb_dims):
    
    #first create the full list of frames and numbers
    frame_numbers = list(range(len(depth_timestamps)))
    full_frame_list = list(zip(depth_timestamps, depth_frames, frame_numbers))
    #the expected number of frames is the length of this list
    expected_num_frames = len(depth_timestamps)
    
    #then determine how many frames we should have, then figure out how many are missing.
    missing_frame_list = list_missing_frames(expected_num_frames, aligned_rgb_depth_folder)
    num_runs = 0
    
    print('Full Frame list looks like')
    print(len(full_frame_list))
    print(full_frame_list[0])
    print('Missing frame list looks like')
    print(len(missing_frame_list))
    print(missing_frame_list[0])
    
    while(len(missing_frame_list) > 0):
        print(f'Running RGB bag align, this is iteration {num_runs}, with {len(missing_frame_list)} frames left to align.')
        frame_list = [full_frame_list[i] for i in missing_frame_list]
        print('New frame list looks like')
        print(len(frame_list))
        print(frame_list[0])
        
        color_metadata_msg, depth_metadata_msg, depth_to_rgb_rotation_q_message, depth_to_rgb_translation_message = align_depth_rsrgb_single_framelist(frame_list, bag_in_path, bag_out_rgb_path, output_folder, aligned_rgb_depth_folder, depth_timestamps, depth_frames, depth_fps, depth_dims, rgb_dims)
        
        missing_frame_list = list_missing_frames(expected_num_frames, aligned_rgb_depth_folder)
        num_runs += 1
    
    #when all are complete, rerturn
    return(color_metadata_msg, depth_metadata_msg, depth_to_rgb_rotation_q_message, depth_to_rgb_translation_message)


#Like the RSRGB equivalent funiton above, we now run this repeatedly until all the bag files are completed.
def align_depth_ximea_single_framelist(frame_list, bag_in_path, bag_out_ximea_path, output_folder, aligned_ximea_depth_folder, 
                      depth_timestamps, depth_frames, depth_fps, depth_dims, rgb_dims, ximea_dims,
                      depth_to_ximea_rotation_q, depth_to_ximea_translation,
                      ximea_intrinsics, ximea_distortion, color_metadata_msg, depth_metadata_msg):
    
    #loop through frames
    for i, (t, frame, framenum) in enumerate(frame_list):
    
        # .bag file for depth info
        try:
            bag_in = rosbag.Bag(bag_in_path)             
            bag_out_ximea = rosbag.Bag(bag_out_ximea_path, 'w') 
            #keep messages with metadata about RGB and depth devices
            for topic, msg, t in bag_in.read_messages():

                #if topic is about gyro or accelerirometer (sensor 2), remove it in our new file
                if 'sensor_2' in str(topic):
                    pass
                    
                #don't need image or depth metadata (will replace them)
                elif 'Depth_0/image/metadata' in str(topic):
                    depth_metadata_msg = msg #vasha  add 9/14
                    if('Exposure Roi' in str(msg.key)):
                        bag_out_ximea.write(topic, msg, t)
                    else:
                        pass
                elif 'Color_0/image/metadata' in str(topic):
                    ximea_metadata_msg = msg #vasha add 9/14
                    #pass  #vasah remove 9/14
                #image and depth data we will replace
                elif 'Depth_0/image/data' in str(topic):
                    depth_data_msg = msg
                elif 'Color_0/image/data' in str(topic):
                    ximea_data_msg = msg
                    
                #if topic is depth stream info, ensure correct frame rate and size
                elif '/device_0/sensor_0/Depth_0/info' == str(topic):
                    msg.fps = depth_fps
                    bag_out_ximea.write(topic, msg, t)
                #depth images now have RGB sensor size, intrinsics, and distortion
                elif '/device_0/sensor_0/Depth_0/info/camera_info' == str(topic):
                    msg.height = depth_dims[1]
                    msg.width = depth_dims[0]
                    bag_out_ximea.write(topic, msg, t)

                #if topic is RGB stream info, ensure correct frame rate and size
                elif '/device_0/sensor_1/Color_0/info' == str(topic):
                    msg.fps = depth_fps #ximea_fps #these are manually set to the same for this step.
                    bag_out_ximea.write(topic, msg, t)
                elif '/device_0/sensor_1/Color_0/info/camera_info' == str(topic):
                    msg.height = ximea_dims[1]
                    msg.width = ximea_dims[0]
                    msg.D = [0.0,0.0,0.0,0.0,0.0] #ximea_distortion set to zero so that aglin_to aligns to undistorted frames
                    msg.K = [*ximea_intrinsics.flatten()]
                    #### Somthing may be fishy here. Previous code shows K as distortion and D isn't set? 
                    bag_out_ximea.write(topic, msg, t)

                else:
                    bag_out_ximea.write(topic,msg,t) #move from l406 vasha 9/14

                #if topic is RGB extrinsics, replace them with rgb to depth exrinsics
                if '/device_0/sensor_1/Color_0/tf/0' == str(topic): #change to if from elif vasha 9/14
                    msg.rotation.x = depth_to_ximea_rotation_q[0]
                    msg.rotation.y = depth_to_ximea_rotation_q[1]
                    msg.rotation.z = depth_to_ximea_rotation_q[2]
                    msg.rotation.w = depth_to_ximea_rotation_q[3]
                    msg.translation.x = depth_to_ximea_translation[0]
                    msg.translation.y = depth_to_ximea_translation[1]
                    msg.translation.z = depth_to_ximea_translation[2]
                    #print('!!!',depth_to_ximea_translation)
                    bag_out_ximea.write(topic, msg, t)
        except Exception as e:
            print(f'Failed to write ximea frame {i}:',e)
        finally:
                        
            #fill up bag with depth & ximea frames
            depth_data_topic = '/device_0/sensor_0/Depth_0/image/data'
            ximea_data_topic = '/device_0/sensor_1/Color_0/image/data'
            depth_metadata_topic = '/device_0/sensor_0/Depth_0/image/metadata'
            ximea_metadata_topic = '/device_0/sensor_1/Color_0/image/metadata'

            time = rospy.Time(i+1)

            #ximea 
            ximea_data_msg = copy.deepcopy(ximea_data_msg)
            ximea_data_msg.height = ximea_dims[1]
            ximea_data_msg.width = ximea_dims[0]
            ximea_data_msg.header.stamp = time
            ximea_data_msg.header.seq = i
            ximea_data_msg.data = np.zeros(ximea_dims).tobytes()
            bag_out_ximea.write(ximea_data_topic, ximea_data_msg, time)
            #metadata
            ximea_metadata_msg.key = 'Frame Counter'
            ximea_metadata_msg.value = str(i)
            bag_out_ximea.write(ximea_metadata_topic, ximea_metadata_msg, time)
            ximea_metadata_msg.key = 'Frame Timestamp'
            ximea_metadata_msg.value = str(t)
            bag_out_ximea.write(ximea_metadata_topic, ximea_metadata_msg, time)

            #depth
            depth_data_msg = copy.deepcopy(depth_data_msg)
            depth_data_msg.height = depth_dims[1]
            depth_data_msg.width = depth_dims[0]
            depth_data_msg.header.stamp = time
            depth_data_msg.header.seq = i
            depth_data_msg.data = frame.tobytes()
            bag_out_ximea.write(depth_data_topic, depth_data_msg, time)
            #metadata
            depth_metadata_msg = copy.deepcopy(depth_metadata_msg)
            depth_metadata_msg.key = 'Frame Counter'
            depth_metadata_msg.value = str(i)
            bag_out_ximea.write(depth_metadata_topic, depth_metadata_msg, time)
            depth_metadata_msg = copy.deepcopy(depth_metadata_msg)
            depth_metadata_msg.value = str(t)
            bag_out_ximea.write(depth_metadata_topic, depth_metadata_msg, time)        


            bag_in.close()
            bag_out_ximea.close()

        ###########
        try:
            #Now use realsense pipeline to read in frames from .bag to run align_to
            pipe = rs2.pipeline()
            cfg = rs2.config()
            cfg.enable_device_from_file(bag_out_ximea_path, repeat_playback=False)
            #cfg.enable_device_from_file(os.path.join(output_folder,'depth_ximea.bag'), repeat_playback=False)
            profile = pipe.start(cfg)
            playback = profile.get_device().as_playback()
            playback.set_real_time(False)

            align_to = rs2.stream.color
            align = rs2.align(align_to)

            #print extrinsics to check them.
            # depth_profile = rs2.video_stream_profile(profile.get_stream(rs2.stream.depth))
            # depth_intrinsics = depth_profile.get_intrinsics()
            # color_profile = rs2.video_stream_profile(profile.get_stream(rs2.stream.color))
            # color_intrinsics = color_profile.get_intrinsics()
            # depth_extrinsics = color_profile.get_extrinsics_to(color_profile)
            # color_extrinsics = color_profile.get_extrinsics_to(depth_profile)

            # print(f'processing {len(depth_timestamps)} timestamps...')
            #for f in range(len(depth_timestamps)):
            frameset = pipe.wait_for_frames()
            #make sure we aren't starting over again
            aligned_frames = align.process(frameset)
            df = aligned_frames.get_depth_frame()
            #df = frameset.get_depth_frame() #not aligned - for debugging
            aligned_depth_frame = np.array(df.get_data())
            cv2.imwrite(os.path.join(aligned_ximea_depth_folder,
                                    f'depth_frame_{str(framenum).zfill(8)}.png'),
                       aligned_depth_frame.astype(np.uint16))
            np.save(os.path.join(aligned_ximea_depth_folder,
                                  f'depth_frame_{str(framenum).zfill(8)}.npy'),
                                   aligned_depth_frame)
        except Exception as e:
            print(f'Failed to read ximea frame {i}:',e)
    #     finally:
    #         pipe.stop()

    #print('Finished Writing Ximea Aligned Depth Files - Removing Bag File Now!')
    os.remove(os.path.join(output_folder,'depth_ximea.bag'))
    
    return()


#Here is where we check for the number of missing frames, then run this over and over until we get all the frames.
#See functions defined at the top of this file for more info on why we do this.
def align_depth_ximea_single(bag_in_path, bag_out_ximea_path, output_folder, aligned_ximea_depth_folder, 
                      depth_timestamps, depth_frames, depth_fps, depth_dims, rgb_dims, ximea_dims,
                      depth_to_ximea_rotation_q, depth_to_ximea_translation,
                      ximea_intrinsics, ximea_distortion, color_metadata_msg, depth_metadata_msg):
    
    
    #first create the full list of frames and numbers
    frame_numbers = list(range(len(depth_timestamps)))
    full_frame_list = list(zip(depth_timestamps, depth_frames, frame_numbers))
    
    #the expected number of frames is the length of this list
    expected_num_frames = len(depth_timestamps)
    
    #then determine how many frames we should have, then figure out how many are missing.
    missing_frame_list = list_missing_frames(expected_num_frames, aligned_rgb_depth_folder)
    num_runs = 0
    while(len(missing_frame_list) > 0):
        print(f'Running Ximea bag align, this is iteration {num_runs}, with {len(missing_frame_list)} frames left to align.')
        frame_list = [full_frame_list[i] for i in missing_frame_list]
        align_depth_ximea_single_framelist(frame_list, bag_in_path, bag_out_ximea_path, output_folder, aligned_ximea_depth_folder, 
                      depth_timestamps, depth_frames, depth_fps, depth_dims, rgb_dims, ximea_dims,
                      depth_to_ximea_rotation_q, depth_to_ximea_translation,
                      ximea_intrinsics, ximea_distortion, color_metadata_msg, depth_metadata_msg)
        missing_frame_list = list_missing_frames(expected_num_frames, aligned_rgb_depth_folder)
        num_runs += 1
    
    #when all are complete, rerturn
    return()



def create_aligned_depth_files(recording_folder, output_folder,
                               ximea_distortion, ximea_intrinsics, 
                               rgb_distortion, rgb_intrinsics,
                               rgb_to_ximea_rotation, rgb_to_ximea_translation, bag_in_path='/home/vasha/st-bravo/bag/sample_final.bag',
                              skip_rgb=False,
                              skip_ximea=False):
    '''
    Run offline alignment of depth stream to both the world camera and the ximea camera coordines.
    Steps:
        1. Create a .bag file with depth frames and dummy realsense RGB frames inside. The distortion and intrinsics are those provided by realsense (distortions are zero)
        2. Read in .bag file from step 1, and run align_to to create depth frames which are aligned with the frame of reference of the realsense RGB camera. Save these depth frames as .pngs
        3. Create a .bag file with depth frames (in rgb space, from step 1) and dummy ximea frames inside. The distortion and intrinsics for both depth and ximea are those measured during stereo camera calibration and read in from file.
        4. Read in .bag file from step 3, and run align_to to create depth frames which are aligned with the frame of reference of the ximea camera. Save these depth frames as .pngs
    
    '''
    
    depth_dims = (848, 480)
    depth_fps = 60
    rgb_dims = (960,540)
    ximea_dims = (2064, 1544, 3)
    #instead of using true ximea and rgb fps, upsample and register just at depth framerate
    ximea_fps = depth_fps
    rgb_fps = depth_fps
    #need rotation matrix as quaternion
    rgb_to_ximea_rotation_q = Rotation.from_dcm(rgb_to_ximea_rotation).as_quat()
 
    #file and folder paths
    depth_timestamps = list(np.loadtxt(os.path.join(recording_folder,'depth','timestamps.csv')))
    raw_depth_files_folder = os.path.join(recording_folder,'depth')
    depth_frames = btp.depth_get_all_frames(raw_depth_files_folder)
    depth_frames = depth_frames[:len(depth_timestamps)] #this is a little sus....VD 3/2023
    #VD 6/29/24: I this this is actually OK because we saved depth in grouped files, for which the last file may be full size and be full of zeros.

    bag_out_rgb_path = os.path.join(output_folder,'depth_rgb.bag')
    bag_out_ximea_path = os.path.join(output_folder,'depth_ximea.bag')
        
    aligned_rgb_depth_folder = os.path.join(output_folder,'rgb_aligned_depth')
    os.makedirs(aligned_rgb_depth_folder, exist_ok=True)
    aligned_ximea_depth_folder = os.path.join(output_folder,'ximea_aligned_depth')
    os.makedirs(aligned_ximea_depth_folder, exist_ok=True)
    
    print('Aligning Depth to Realsense RGB')

    #call the function for aligning depth and RGB. Note this will run repeatedly until all frames are aligned.
    color_metadata_msg, depth_metadata_msg, depth_to_rgb_rotation_q_message, depth_to_rgb_translation_message = align_depth_rsrgb_single(bag_in_path, bag_out_rgb_path, output_folder, aligned_rgb_depth_folder, 
                                           depth_timestamps, depth_frames, depth_fps, depth_dims, rgb_dims)

    #convert rgb to depth quaternion to rotation matrix
    depth_to_rgb_rotation = Rotation.from_quat(np.array((depth_to_rgb_rotation_q_message.x,
                                                        depth_to_rgb_rotation_q_message.y,
                                                        depth_to_rgb_rotation_q_message.z,
                                                       depth_to_rgb_rotation_q_message.w))).as_dcm()
    depth_to_rgb_translation = np.array((depth_to_rgb_translation_message.x,
                                        depth_to_rgb_translation_message.y,
                                        depth_to_rgb_translation_message.z))
    #combine rgb to depth with ximea to rgb to get ximea  to depth
    print('rgb2d_mat',depth_to_rgb_rotation)
    print('x2rgb_mat',rgb_to_ximea_rotation)
    depth_to_ximea_rotation = rgb_to_ximea_rotation @ depth_to_rgb_rotation
    depth_to_ximea_translation = rgb_to_ximea_rotation @ depth_to_rgb_translation + rgb_to_ximea_translation

    depth_to_ximea_rotation_q = Rotation.from_dcm(depth_to_ximea_rotation).as_quat()
    
    print(f'RGB -> Depth extrinsics rotations are: {np.array((depth_to_rgb_rotation_q_message.x,depth_to_rgb_rotation_q_message.y,depth_to_rgb_rotation_q_message.z,depth_to_rgb_rotation_q_message.w))}')
    print(f'RGB -> Depth extrinsics translations are: {depth_to_rgb_translation}')
    print(f'Ximea -> Depth extrinsics rotations are: {depth_to_ximea_rotation_q}')
    print(f'Ximea -> Depth extrinsics translations are: {depth_to_ximea_translation}')
    
    #read in new depth files
    #depth_timestamps = list(np.loadtxt(os.path.join(recording_folder,'depth','timestamps.csv'))) #same as above
    #depth_frames = [cv2.imread(os.path.join(aligned_rgb_depth_folder,f'depth_frame_{str(f).zfill(8)}.png')) for f in range(len(depth_timestamps))]
#     depth_frames = [np.load(os.path.join(aligned_rgb_depth_folder,f'depth_frame_{str(f).zfill(8)}.npy')) for f in range(len(depth_timestamps))]
    #depth_frames = depth_frames[:len(depth_timestamps)]
    
    if not skip_ximea:
        #Write Depth -> Ximea Bag File
        print('Aligning Depth to Ximea')
        align_depth_ximea_single(bag_in_path, bag_out_ximea_path, output_folder, aligned_ximea_depth_folder, 
                          depth_timestamps, depth_frames, depth_fps, depth_dims, rgb_dims, ximea_dims,
                          depth_to_ximea_rotation_q, depth_to_ximea_translation,
                          #rgb_intrinsics, rgb_distortion, 
                          ximea_intrinsics, ximea_distortion, color_metadata_msg, depth_metadata_msg)
        #os.remove(os.path.join(output_folder,'depth_ximea.bag')) #already removed
    else:
        print('Skip ximea flag is true. Skipping it!')


    print('Done with .bag Alignment for this trial!')
