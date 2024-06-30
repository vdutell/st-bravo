import pyrealsense2 as rs2
import rospy
import numpy as np
import rosbag
import os
import copy
from scipy.spatial.transform import Rotation
import utils.bins_to_pngs as b2p
import sys
import argparse
import cv2

def create_aligned_depth_files(recording_folder, output_folder, 
                               ximea_distortion, ximea_intrinsics,
                               rgb_distortion, rgb_intrinsics,
                               depth_distortion, depth_intrinsics,
                               depth_to_rgb_rotation,
                               depth_to_rgb_translation,
                               rgb_to_ximea_rotation,
                               rgb_to_ximea_translation,
                               bmax=1e10):
    '''
    Run offline alignment of depth stream to both the world camera and the ximea camera coordines
    '''
    
    depth_dims = (848, 480)
    depth_fps = 90
    rgb_dims = (960, 540, 3)
    #rgb_fps = 30
    ximea_dims = (2064, 1544, 3)
    #ximea_fps = 200
    #instead of using true ximea and rgb fps, upsample and register just at depth framerate
    ximea_fps = depth_fps
    rgb_fps = depth_fps
    depth_frames_per_file = 1000
  
     #intrinsics & extrinsics for ximea camera
#     ximea_cammatrix = [1.81742550e+03, 0.00000000e+00, 1.04625244e+03, 0.00000000e+00, 1.82025698e+03, 7.55093896e+02, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]

#     rgb_to_ximea_rotation = [[9.996304422422012115e-01, -1.885864076914232912e-02, -1.957883068525424763e-02],
# [1.866599409722233260e-02, 9.997760038579057706e-01, -9.976110177782640878e-03],
# [1.976258098083020359e-02, 9.606965090872355076e-03, 9.997585441568973552e-01]]
#     rgb_to_ximea_translation = 1e-3 * np.array((2.853691061569304566e+01, 9.189543619411789932e-02, 2.261175317542426733e+00))

#     #intrinsics & extrinsics for ximea camera
#     rgb_to_ximea_rotation = np.array(((9.993029061967553250e-01, 3.541810020885153426e-02, -1.180084083082719518e-02),
#                                                (-3.511955874400791294e-02, 9.990800166504110180e-01, 2.461172329278595722e-02),
#                                                (1.266168473564812866e-02, -2.418012629020444698e-02, 9.996274322127443046e-01)))
#     rgb_to_ximea_translation = 1e-3 * np.array((2.853691061569304566e+01, 9.189543619411789932e-02, 2.261175317542426733e+00))
#     ximea_distortion = np.array((0,0,0,0,0,0,0,0,0)) #fill this in later plz. (on server)
#     if(indoor):
#         calib_folder = '/hmet_data/calibration/indoor'
#     else:
#         calib_folder = '/hmet_data/calibration/outdoor'

#     ximea_distortion = list(np.loadtxt(os.path.join(calib_folder, 'Distortion_cam2.txt')).flatten()) 
#     ximea_cammatrix = list(np.loadtxt(os.path.join(calib_folder, 'Fundamental_matrix.txt')).flatten())
#     ximea_rectification = list(np.loadtxt(os.path.join(calib_folder, 'Rectification_transform_cam2.txt')).flatten())
#     ximea_projection = list(np.loadtxt(os.path.join(calib_folder, 'Projection_matrix_cam2.txt')).flatten()) 
    
    #print('D:',ximea_distortion)
    #print('K:',ximea_cammatrix)
    #print('R:',ximea_rectification)
    #print('P:',ximea_projection)
    
#     rgb_to_ximea_rotation = np.loadtxt(os.path.join(calib_folder, 'Rotation_matrix.txt'))
#     rgb_to_ximea_translation =  1e-3 * np.loadtxt(os.path.join(calib_folder, 'Translation_vector.txt')) #1e3 *

    timestamps = list(np.loadtxt(os.path.join(recording_folder,'depth','timestamps.csv')))
    print(f'{len(timestamps)} timestamps.')
    depth_frame_folder = os.path.join(recording_folder,'depth')
    #depth_frames = [np.load(os.path.join(recording_folder,'depth',f'depth_frame_{str(f).zfill(8)}.npy')) for f in range(len(timestamps))]
    #depth_frame_paths = s.path.join(recording_folder,'depth',f'depth_frame_{str(f).zfill(8)}.npy')) for f in range(len(timestamps))]
    
    bag_in = rosbag.Bag(os.path.join('/home/vasha/st-bravo_analysis','sample_final.bag'))
    bag_out = rosbag.Bag(os.path.join(output_folder,'depth_rgb.bag'), 'w')
    
    # .bag file for depth info
    try:
        #keep messages with metadata about RGB and depth devices
        for topic, msg, t in bag_in.read_messages():

            #if topic is about gyro or accelerirometer (sensor 2), remove it in our new file
            if 'sensor_2' in str(topic):
                pass

            #don't need image or depth metadata (will replace them)
            elif 'Depth_0/image/metadata' in str(topic):
                pass
            elif 'Color_0/image/metadata' in str(topic):
                pass 

            #image and depth data we will replace
            elif 'Depth_0/image/data' in str(topic):
                depth_data_msg = msg
            elif 'Color_0/image/data' in str(topic):
                color_data_msg = msg

            #if topic is depth stream info, ensure correct frame rate and size
            elif '/device_0/sensor_0/Depth_0/info' == str(topic):
                msg.fps = depth_fps
                bag_out.write(topic, msg, t)
            elif '/device_0/sensor_0/Depth_0/info/camera_info' == str(topic):
                msg.height = depth_dims[1]
                msg.width = depth_dims[0]
                bag_out.write(topic, msg, t)

            #if topic is RGB stream info, ensure correct frame rate and size
            elif '/device_0/sensor_1/Color_0/info' == str(topic):
                msg.fps = rgb_fps
                bag_out.write(topic, msg, t)
            elif '/device_0/sensor_1/Color_0/info/camera_info' == str(topic):
                msg.height = rgb_dims[1]
                msg.width = rgb_dims[0]
                bag_out.write(topic, msg, t)

            else:
                #keep everything else
                bag_out.write(topic, msg, t)

            #if topic is RGB extrinsics, save them so we can combine them with ximea to rgb extrinsics later to get ximea to depth exrinsics
            if '/device_0/sensor_1/Color_0/tf/0' == str(topic):
                depth_to_rgb_translation_message = msg.translation
                depth_to_rgb_rotation_q_message = msg.rotation

        #fill up bag with depth & RGB frames
        depth_data_topic = '/device_0/sensor_0/Depth_0/image/data'
        color_data_topic = '/device_0/sensor_1/Color_0/image/data'
        for i, t in enumerate(timestamps):
            #load in depth frame
            if(i%depth_frames_per_file==0):
                file_number = np.floor(i / depth_frames_per_file)    
                file_start = int(file_number * depth_frames_per_file)
                file_end = int(file_start + depth_frames_per_file - 1)
                filename = os.path.join(depth_frame_folder,f'depth_frames_{str(file_start).zfill(8)}_{str(file_end).zfill(8)}.npy')
                try:
                    depth_frame_array = np.load(filename)
                except:
                    filename = [f for f in os.listdir(depth_frame_folder) if str(file_start) in f][0]
                    depth_frame_array = np.load(os.path.join(depth_frame_folder, filename))
                    
            frame_offset = i % depth_frames_per_file
            depth_frame = depth_frame_array[frame_offset]
           
            #depth_data_msg = copy.deepcopy(sample_depth_data_msg)
            time = rospy.Time(t)
            depth_data_msg.height = depth_dims[1]
            depth_data_msg.width = depth_dims[0]
            depth_data_msg.header.stamp = time
            depth_data_msg.header.frame_id = str(i)
            depth_data_msg.header.seq = i
            depth_data_msg.data = depth_frame.tobytes()
            bag_out.write(depth_data_topic, depth_data_msg, time)
            
            color_data_msg.height = rgb_dims[1]
            color_data_msg.width = rgb_dims[0]
            color_data_msg.header.stamp = time
            color_data_msg.header.frame_id = str(i)
            color_data_msg.header.seq = i
            #color_data_msg.data = np.zeros(rgb_dims).tobytes()
            color_data_msg.data = np.zeros((2,2,3)).tobytes()
            bag_out.write(color_data_topic, color_data_msg, time)
            
            if(i > bmax):
                break
    finally:
        bag_in.close()
        bag_out.close()
    
    print('Finished Creating Depth -> RGB Bag File')
    
    
    print('Creating Depth -> Ximea Bag File')
    #print('Finished Writing RGB Aligned Depth Files')
    
    #convert rgb to depth quaternion to rotation matrix
    depth_to_rgb_rotation = Rotation.from_quat(np.array((depth_to_rgb_rotation_q_message.x, depth_to_rgb_rotation_q_message.y,
                                                         depth_to_rgb_rotation_q_message.z, depth_to_rgb_rotation_q_message.w))).as_dcm()
    depth_to_rgb_translation = np.array((depth_to_rgb_translation_message.x, depth_to_rgb_translation_message.y, depth_to_rgb_translation_message.z))
    #combine rgb to depth with ximea to rgb to get ximea  to depth
    depth_to_ximea_rotation = rgb_to_ximea_rotation @ depth_to_rgb_rotation
    depth_to_ximea_translation = rgb_to_ximea_rotation @ depth_to_rgb_translation + rgb_to_ximea_translation

    depth_to_ximea_rotation_q = Rotation.from_dcm(depth_to_ximea_rotation).as_quat()

    bag_in = rosbag.Bag(os.path.join('/home/vasha/st-bravo_analysis','sample_final.bag'))
    bag_out = rosbag.Bag(os.path.join(output_folder,'depth_ximea.bag'), 'w')

    #Write Depth -> Ximea Bag File
    try:
        #keep messages with metadata about RGB and depth devices
        for topic, msg, t in bag_in.read_messages():

            #if topic is about gyro or accelerirometer (sensor 2), remove it in our new file
            if 'sensor_2' in str(topic):
                pass
            #don't need image or depth metadata (will replace them)
            elif 'Depth_0/image/metadata' in str(topic):
                pass
            elif 'Color_0/image/metadata' in str(topic):
                pass 
            #image and depth data we will replace
            elif 'Depth_0/image/data' in str(topic):
                depth_data_msg = msg
            elif 'Color_0/image/data' in str(topic):
                ximea_data_msg = msg
            #if topic is depth stream info, ensure correct frame rate and size
            elif '/device_0/sensor_0/Depth_0/info' == str(topic):
                msg.fps = depth_fps
                bag_out.write(topic, msg, t)
            elif '/device_0/sensor_0/Depth_0/info/camera_info' == str(topic):
                msg.height = depth_dims[1]
                msg.width = depth_dims[0]
                bag_out.write(topic, msg, t)

            #if topic is RGB stream info, ensure correct frame rate and size
            elif '/device_0/sensor_1/Color_0/info' == str(topic):
                msg.fps = ximea_fps
                bag_out.write(topic, msg, t)
            elif '/device_0/sensor_1/Color_0/info/camera_info' == str(topic):
                msg.height = ximea_dims[1]
                msg.width = ximea_dims[0]
                msg.D = ximea_distortion
                msg.K = ximea_intrinsics
                #msg.R = ximea_rectification
                #msg.P = ximea_projection
                bag_out.write(topic, msg, t)

            #if topic is RGB extrinsics, replace them with ximea to depth exrinsics
            elif '/device_0/sensor_1/Color_0/tf/0' == str(topic):
                msg.rotation.x = depth_to_ximea_rotation_q[0]
                msg.rotation.y = depth_to_ximea_rotation_q[1]
                msg.rotation.z = depth_to_ximea_rotation_q[2]
                msg.rotation.w = depth_to_ximea_rotation_q[3]
                msg.translation.x = depth_to_ximea_translation[0]
                msg.translation.y = depth_to_ximea_translation[1]
                msg.translation.z = depth_to_ximea_translation[2]
            else:
                #keep everything else
                bag_out.write(topic, msg, t)

        #fill up bag with depth & ximea frames
        depth_data_topic = '/device_0/sensor_0/Depth_0/image/data'
        color_data_topic = '/device_0/sensor_1/Color_0/image/data'
        for i, t in enumerate(timestamps):
            #load in depth frame
            if(i%depth_frames_per_file==0):
                file_number = np.floor(i / depth_frames_per_file)    
                file_start = int(file_number * depth_frames_per_file)
                file_end = int(file_start + depth_frames_per_file - 1)
                filename = os.path.join(depth_frame_folder,f'depth_frames_{str(file_start).zfill(8)}_{str(file_end).zfill(8)}.npy')
                try:
                    depth_frame_array = np.load(filename)
                except: #at end of trial filename isn't full
                    filename = [f for f in os.listdir(depth_frame_folder) if str(file_start) in f][0]
                    depth_frame_array = np.load(os.path.join(depth_frame_folder, filename))
            frame_offset = i % depth_frames_per_file
            depth_frame = depth_frame_array[frame_offset]
                        
            time = rospy.Time(t)
            depth_data_msg.height = depth_dims[1]
            depth_data_msg.width = depth_dims[0]
            depth_data_msg.header.stamp = time
            depth_data_msg.header.frame_id = str(i)
            depth_data_msg.header.seq = i
            depth_data_msg.data = depth_frame.tobytes()
            bag_out.write(depth_data_topic, depth_data_msg, time)
            
            #print(depth_data_msg.header)

            
            ximea_data_msg.height = ximea_dims[1]
            ximea_data_msg.width = ximea_dims[0]
            ximea_data_msg.header.stamp = time
            ximea_data_msg.header.frame_id = str(i)
            depth_data_msg.header.seq = i
            #ximea_data_msg.data = np.zeros(ximea_dims).tobytes()
            ximea_data_msg.data = np.zeros((2,2,3)).tobytes()

            bag_out.write(color_data_topic, ximea_data_msg, time)
            
            if(i > bmax):
                break

    finally:
        bag_in.close()
        bag_out.close()
    print(f'Finished Creating Depth -> Ximea Bag File: {i} frames')

    
    
    #print('Finished Writing Ximea Aligned Depth Files')


def run_align_to(bag_file_path, out_folder, multiplier=1):
#     # Create alignment primitive with color as its target stream:
#     pipe = rs2.pipeline()
#     cfg = rs2.config()
#     cfg.enable_device_from_file("../object_detection.bag")
#     profile = pipe.start(cfg)

#     align = rs.align(rs2.stream.color)
#     frameset = align.process(frameset)

#     # Update color and depth frames:
#     aligned_depth_frame = frameset.get_depth_frame()
#     colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())


    try:
        pipe = rs2.pipeline()
        cfg = rs2.config()
        cfg.enable_device_from_file(bag_file_path, repeat_playback=False)
        
        #config.enable_stream(rs.stream.depth, rs.format.z16, 90)
        #config.enable_stream(rs.stream.color, rs.format.z16, 90)
        
        
        profile = pipe.start(cfg)
        playback = profile.get_device().as_playback()
        playback.set_real_time(False)
        last_pos = playback.get_position()
        #depth_timestamps = list(np.loadtxt(depth_timestamp_file))

        align_to = rs2.stream.color
        align = rs2.align(align_to)

        current_frame=0

        while True:
            frameset = pipe.wait_for_frames(timeout_ms=1000)
            #curr_pos = playback.get_position()
            #playback.pause()
            #print('*',end='')
            #try align_to
            aligned_frames = align.process(frameset)
            df = aligned_frames.get_depth_frame()
            #df = frameset.get_depth_frame()
            fnum = df.get_frame_number()
            #if(fnum == current_frame):
            #print('*',fnum)
            #assert(df.get_frame_number() == f)
            if(fnum == current_frame):
                aligned_depth_frame = np.array(df.get_data(),copy=True)*multiplier
                cv2.imwrite(os.path.join(out_folder,f'depth_frame_{str(fnum).zfill(7)}.png'), aligned_depth_frame)
                current_frame +=1

        #             #make sure we aren't starting over again #this is taken care of with repeat_playback=False
        #             ts = frameset.get_timestamp()
        #             if(last_timestamp <= ts):
        #                 last_timestamp = ts
        #             else:
        #                 break
            #playback.resume()

    except Exception as e:
        print(f'Failed to get frame {current_frame}:',e)
    finally:
        pipe.stop()
        print(f'Finished aligning {bag_file_path} to {out_folder}')

    
    
    
#create_aligned_depth_files(recording_folder='/home/vasha/recordings/2021_04_13/000', output_folder='./depth_align_bag/')
    
def main(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--recording_folder", help="path to data")
    parser.add_argument("-o","--output_path", help="path to csv file containing trial info", type=str, 
                        default='~/st-bravo_analysis/trial_list.csv')
    parser.add_argument("-i", "--indoor", help="Is this an indoor trial?", type=bool, default=True)
   
    
    args = parser.parse_args()
    #launch analysis
    create_aligned_depth_files(args.recording_folder, args.output_path, args.indoor);
    
if __name__ == "__main__":
   main(sys.argv[1:])