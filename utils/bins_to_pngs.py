import sys
import cv2
from pathlib import Path
import os
import pandas as pd
import numpy as np
import yaml
import glob

def convert_bin_pngs(filename, first_fnum, save_batchsize, save_folder,  dims=(1544,2064), 
                     original_cam_matrix=None,
                     distortion_matrix=None,
                     new_cam_matrix=None):
    '''
    Take a file saved in .bin format from a ximea camera, and convert it to png images.
    Parameters:
        filename (str): file to be converted
        save_folder (str): folder to save png files
        im_shape (2pule ints): shape of image
        img_format (str): Image format files are saved
    Returns:
        None
    '''
    nbytes = np.prod(dims)
    
    with open(filename, 'rb') as fn:
        bs = fn.read(1)
        for i in range(first_fnum, first_fnum+save_batchsize):
            save_filepath = os.path.join(save_folder, f'frame_{i}.png')
            binary_img = []
            for b in range(nbytes):
                binary_img.append(int.from_bytes(fn.read(1),'big'))
            binary_img = np.array(binary_img)
            cimage = cv2.flip(cv2.cvtColor(np.uint8(binary_img.reshape(dims)),cv2.COLOR_BayerGR2BGR),-1)
            #apply undistort
            if original_cam_matrix is not None:
                cimage = cv2.undistort(cimage, original_cam_matrix, distortion_matrix, None, new_cam_matrix)
            cv2.imwrite(save_filepath, cimage)


def convert_trial_directory(bin_folder, png_folder, save_batchsize, original_cam_matrix=None, camera_distortion=None, ximea_width_height=(2064,1544)):
    ''' Convert a single trial from .bin to .png images This runs for a LONG time'''
    
    print(f'Saving pngs at {png_folder}')
    Path(png_folder).mkdir(parents=True, exist_ok=True)
    print(f'Each * is {save_batchsize} frames...')
    frame_start = 0
    bin_file = os.path.join(bin_folder,f'frames_{frame_start}_{frame_start+save_batchsize-1}.bin')
    if original_cam_matrix is not None:
        print('Using Undistort from Found Camera Intrinsics File!')
        #print(original_cam_matrix)
        #print(camera_distortion)
        #print(ximea_width_height)
        new_cam_matrix, roi = cv2.getOptimalNewCameraMatrix(original_cam_matrix, camera_distortion, ximea_width_height, 1, ximea_width_height)
#         cammat_distmat_newcammat = [original_cam_matrix, distortion_matrix, new_cam_matrix]
#         print(cammat_distmat_newcammat)
    else:
        print('Could not find Camera Intrinsics File. Ignoring.')
        original_cam_matrix = camera_distortion = new_cam_matrix = None
    
    while(os.path.isfile(bin_file)):
        print('*')
        convert_bin_pngs(bin_file, frame_start, save_batchsize, png_folder, dims=(1544,2064),
                         original_cam_matrix = original_cam_matrix,
                         distortion_matrix = camera_distortion,
                         new_cam_matrix = new_cam_matrix)
        frame_start += save_batchsize
        bin_file = os.path.join(bin_folder,f'frames_{frame_start}_{frame_start+save_batchsize-1}.bin')
    print('Done!')

    
    
# def get_depth_frame(bin_folder,frame_num,frames_per_file=1000):
#     file_number = np.floor(frame_num / frames_per_file)    
#     frame_offset = frame_num % frames_per_file
#     file_start = int(file_number * frames_per_file)
#     file_end = int(file_start + frames_per_file - 1)
    
#     filename = os.path.join(bin_folder,f'depth_frames_{str(file_start).zfill(8)}_{str(file_end).zfill(8)}.npy')
#     frames = np.load(filename)
#     return(frames[frame_offset])



def bin_to_im(binfile, nframe, dims=(1544,2064),quickread=True):
    '''
    convert a single image from 8-bit raw bytes to png image.
    Input:
        binfile (str): path to binary file
        dims (2ple int): What are the dimensions of the iamge?
        nframe (int): Which frame number do we want within image?
        '''
    # for uint8
    nbytes = np.prod(dims)
    startbyte = nframe*nbytes
    if(quickread):
        with open(binfile, 'rb') as fn:
            fn.seek(startbyte+1)
            im = fn.read(nbytes)
        im = np.frombuffer(im,dtype='uint8')
    else:
        im = []
        with open(binfile, 'rb') as fn:
            fn.seek(startbyte)
            bs = fn.read(1)
            for i in range(nbytes):
                bs = fn.read(1)
                bs = int.from_bytes(bs,'big')
                im.append(bs)
            im = np.array(im)
    im = im.reshape(dims)
    im = cv2.cvtColor(im, cv2.COLOR_BayerGR2RGB)
    return(im)



def ximea_get_frame(frame_number, save_batchsize=1000, cam_save_folder='.', img_dims=(1544,2064)):
    '''
    Get the filename and offset of a given frame number from the camera.
    Params:
        frame_number (int): number of frame desired
        save_bathsize (int): what was the batchsize during collection?
        cam_save_folder (str): what is the name of the folder?
        img_dims (int, int): dimensions of frame reading in.
    Returns:
        frame (2d numpy array): 2d array of frame from saved file
    '''
    
    file_start = int(np.floor(frame_number/save_batchsize)*save_batchsize)
    file_end = file_start + save_batchsize - 1
    frame_offset = frame_number%file_start if file_start>0 else frame_number
    file_name = f'frames_{file_start}_{file_end}.bin'
    file_path = os.path.join(cam_save_folder, file_name)
    
    frame = bin_to_im(file_path, frame_offset, img_dims)

    return(frame)
    
    
def depth_get_frame(frame_number, cam_save_folder, save_batchsize=1000):
    '''
    Get the filename and offset of a given frame number from the camera.
    Params:
        frame_number (int): number of frame desired
        save_bathsize (int): what was the batchsize during collection?
        cam_save_folder (str): what is the name of the folder?
        img_dims (int, int): dimensions of frame reading in.
    Returns:
        frame (2d numpy array): 2d array of frame from saved file
    '''
    
    file_start = int(np.floor(frame_number/save_batchsize)*save_batchsize)
    file_end = file_start + save_batchsize - 1
    frame_offset = frame_number%file_start if file_start>0 else frame_number
    file_name = f'depth_frames_{str(file_start).zfill(8)}_{str(file_end).zfill(8)}.npy'
    file_path = os.path.join(cam_save_folder, file_name)
    depth_frames_batch = np.load(file_path)
    frame = depth_frames_batch[frame_offset]
    return(frame)

def depth_get_all_frames(cam_save_folder, save_batchsize=1000):
    '''
    Get the filename and offset of a given frame number from the camera.
    Params:
        frame_number (int): number of frame desired
        save_bathsize (int): what was the batchsize during collection?
        cam_save_folder (str): what is the name of the folder?
        img_dims (int, int): dimensions of frame reading in.
    Returns:
        frame (2d numpy array): 2d array of frame from saved file
    '''
    frames = []
    for fnum in range(1000):
        file_start = save_batchsize * fnum
        file_end = file_start + save_batchsize-1
        file_name = f'depth_frames_{str(file_start).zfill(8)}_{str(file_end).zfill(8)}.npy'
        file_path = os.path.join(cam_save_folder, file_name)
        #print(file_path)
        if not os.path.exists(file_path):
            #find last file
            #print(os.path.join(cam_save_folder,f'depth_frames_{str(file_start).zfill(8)}_'))
            file_path = glob.glob(os.path.join(cam_save_folder,f'depth_frames_{str(file_start).zfill(8)}_*'))[0]
            #print(file_name)
            #file_path = os.path.join(cam_save_folder, file_name)
            depth_frames_batch = np.load(file_path)
            [frames.append(f) for f in depth_frames_batch if np.mean(f) != 0]
            return(frames)
        else:
            depth_frames_batch = np.load(file_path)
            [frames.append(f) for f in depth_frames_batch if np.mean(f) != 0]
    return(frames)

    
    
# # idx = int(sys.argv[1])
# # base_dir = '/hmet_data'
# # trial_list_file = '~/st-bravo_analysis/trial_list.csv'
# # data_dir = os.path.join(base_dir, 'raw')
# # png_dir = os.path.join(base_dir, 'pngs')
# # save_batchsize = 4000

    
# # convert_trial_directory(data_dir, png_dir, trial_list_file, idx, save_batchsize)