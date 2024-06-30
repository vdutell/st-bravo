import os
import numpy as np
import cv2
import re
import matplotlib.pyplot as plt
import argparse
import sys
from pathlib import Path
import pandas as pd

#scripts to do the heavy lifting
import utils.bins_to_pngs as bin2png
import utils.timeline as timeline
import utils.tracegen as tracegen
import utils.traceconvert as traceconvert
import utils.plotting as plu
import stftoolkit as stf
import utils.bag_alignment as bag
import utils.gaze_map_matlab.launch_calibration as launch_calibration
import utils.convert_pldata as conpl

def run_analysis(analysis_base_dir,
                 trial_list_path, line_number, 
                 skip_convert_png=True,
                 skip_bag_align=False,
                 skip_calibration=False,
                 skip_calib_bag=False,
                 skip_fourier=True,
                 skip_gaze_dependencies=False,
                 gpu_num=0):
    '''
    Mother script to run analysis of a single trial (line from excel trial_list.csv spreadsheet)
    Params:
        analysis_base_dir(str): path for outputting analysis data (/hmet_analysis or /hmet_analysis_2)
        trial_list_path(str): Path to csv file with trial info
        line_number(int): which line/trial to analyze from trial_list
        skip_convert_png(bool): do we need to convert to pngs?
        skip_bag_align(bool): do we need to run the bag depth -> Ximea Alignment?
        skip_calibration(bool): do we need to calculate the calibration?
        skip_convertskip_fourier_png(bool): do we need to run Fourier analysis?
    Outputs:
        Doesn't return anything, but prints a ton of stuff, writes even more stuff to:
            data_path/pngs
            data_path/analysis
            ./output/
    '''
    
    #parameters that are fixed (note framerates are ideal not necessarily true)
    fps_ximea = 200
    fps_pupil = 200
    fps_depth = 90
    fps_rsrgb = 60
    fps_imu = 200
    resample_fps = 200
    save_batchsize_ximea = 1000
    save_batchsize_depth = 1000
    
    #calibration_types = ['farpoint_eye_cal_pre', 'farpoint_eye_cal_post','left_eye_cal','right_eye_cal', 'val_eye_cal', 'pp_eye_cal', 'color_cal'] #don't have (or need) depth info for farpoint_eye_cal pre and post, pp, or color
    calibration_types = ['left_eye_cal','right_eye_cal', 'val_eye_cal']

    #ximea spec params
    ximea_dims = (1544,2064)
    ximea_horizontal_fov_deg = 61
    degrees_ecc = 30 #for peripheral gaze chunking *not yet implemented*
    
    #Fourier Analysis parameters
    chunk_secs = 1
    chunk_pix = 256 #256
    num_chunks = 250
    cosine_window = True
    spatial_cuttoff_cpd = 14
    temporal_cuttoff_fps = np.inf
    
    #get info from table to get subject name, task, etc.
    print(f'Parsing Trial List line {line_number}....')
    trial_line = pd.read_csv(trial_list_path).iloc[line_number]
    bsdir = trial_line['base_dir']
    date = trial_line['folder']
    subject_name = trial_line['subject']
    trial_num = str(trial_line['trial']).zfill(3)
    task_name = trial_line['task']
    task_iter = str(trial_line['iter'])
    aperature_location = trial_line['aperature_setting'] ####TMP!!!!
    skip_bool = trial_line['skip']
    print(f'This is subject {subject_name}, task {task_name}, iter {task_iter}.')
    if(skip_bool==True):
        print('Skipping this trial because skip boolean is True')
        return()
    
    print(f'Skipping Gaze Dependencies? {skip_gaze_dependencies}')
    print(type(skip_gaze_dependencies))

    #flag this as a human or buddy (fixed camera) trial
    if(trial_line['subject'] in ['bu']):
        print('This is a buddy trial, no fixations.')
        has_fixations = False
    elif skip_gaze_dependencies:
        print('Skipping Gaze Dependencies here.')
        has_fixations = False
    else:
        has_fixations = True
        
    print(f'Trial Has Fixations analyze? {has_fixations}')
    
    #put directory structures together for accessing data
    data_folder = os.path.join(bsdir, subject_name, date, str(trial_line['trial']).zfill(3))
    
    #create directory structure to save analyzed files
    ana_folder = os.path.join(analysis_base_dir, subject_name, task_name, task_iter)
    #make paths that dont exist yet
    print(f'Creating analysis directory structure at {ana_folder}.')
    Path(ana_folder).mkdir(parents=True, exist_ok=True)
    
    bin_folder = os.path.join(data_folder,'ximea','ximea')
    png_folder = os.path.join(ana_folder,'pngs')
        
    #Camera Matrices - distortion, intrinsics, and and extrinsics
    print('Loading Camera Matrix Files (Intrinsics/Extrinsics).')
    #Ximea
    camera_intrinsics_folder = f'/home/vasha/st-bravo_clean/calibration_info/{aperature_location}'
    ximea_distortion = np.loadtxt(os.path.join(camera_intrinsics_folder,'camera2RadialDist.txt'), delimiter=',')
    ximea_distortion = np.array([*ximea_distortion, 0, 0], dtype='float32') #set p1, p2, k3 to zero
    ximea_intrinsics = np.array(np.loadtxt(os.path.join(camera_intrinsics_folder,'Intrinsics_WC.txt'), delimiter=','), dtype='float32')
    print('Ximea Intrinsics are:',ximea_intrinsics)
    #Realsense RGB
    rsrgb_distortion = np.loadtxt(os.path.join(camera_intrinsics_folder,'camera1RadialDist.txt'), delimiter=',')
    rsrgb_distortion = np.array([*rsrgb_distortion, 0, 0], dtype='float32') #set p1, p2, k3 to zero
    rsrgb_intrinsics = np.array(np.loadtxt(os.path.join(camera_intrinsics_folder,'Intrinsics_RS.txt'), delimiter=','), dtype='float32')
    print('RGB Intrinsics are:',rsrgb_intrinsics)

    #Extrinsics (ximea to realsense)
    rsrgb_to_ximea_extrinsics_rotation = np.loadtxt(os.path.join(camera_intrinsics_folder,'Rotation_matrix.txt'), delimiter=',')
    rsrgb_to_ximea_extrinsics_rotation = rsrgb_to_ximea_extrinsics_rotation * np.array(((1,-1,-1),(-1,1,-1),(-1,-1,1)))
    rsrgb_to_ximea_extrinsics_translation = np.loadtxt(os.path.join(camera_intrinsics_folder,'Translation_vector.txt'), delimiter=',')
    rsrgb_to_ximea_extrinsics_translation = 1e-3 * rsrgb_to_ximea_extrinsics_translation 
    
    ##################################################################################
    #Run .bin to png conversion for ximea data
    ##########################################################################################
    if not skip_convert_png:
        print('Running .bin to .png conversion for Ximea.  This will likely take a few days...')
        
        #run conversion script on binary folder
        #print('Skipping bin conversion temporarily, running only calibration folders.')
        bin2png.convert_trial_directory(bin_folder, png_folder, save_batchsize_ximea, ximea_intrinsics, ximea_distortion)
        print(f'Finished .bin to .png conversion for {ana_folder}.')
        
        #run conversion .bin to .png for calibrations if it hasn't been done already.
        if(has_fixations):
            print('This is a human trial with calibrations. Searching for corresponding calibration pngs')
            for caltype in calibration_types:
                #create unique identifier for calibration called calib_id
                folderid = trial_line[caltype]
                calib_id = f'{date}_{folderid}' #can't use calib type in name because pre/post are shared for far point
                calibration_png_folder = os.path.join(analysis_base_dir, subject_name, 'calib', calib_id, 'pngs')
#                 if(Path(os.path.join(calibration_png_folder,'frame_0.png')).exists()):
#                     print(f'Found PNG Calibration folder for corresponding {caltype}.')
#                 else:
                    #print(f'Did not find corresponding PNG calibration for {caltype}. Creating now at {calibration_png_folder}')
                print(f'Creating PNGs for calibration for {caltype}: {calibration_png_folder}')
                calibration_bin_folder = os.path.join(bsdir, subject_name, date, str(trial_line[caltype]).zfill(3),'ximea','ximea')
                bin2png.convert_trial_directory(calibration_bin_folder, calibration_png_folder, save_batchsize_ximea, ximea_intrinsics, ximea_distortion)
        else:
            print('This is a manniquen (buddy) trial, no need to convert calibration trial pngs.')       

    else:
        print('Skipping .bin to .png conversion for Ximea.')
        
    ######################################################################################### 
    # align depth -> RGB & Ximea space using realsense align_to function and .bag files
    ##########################################################################################
    if not skip_bag_align:
        print('Running Depth Alignment to RGB & Ximea Space. This will take a few hours....')
        print(f'Data folder is: {data_folder}')
        print(f'Analysis folder is: {ana_folder}')  
        bag.create_aligned_depth_files(recording_folder=data_folder,
                               output_folder=ana_folder,
                               ximea_distortion=ximea_distortion, 
                               ximea_intrinsics=ximea_intrinsics, 
                               rgb_distortion=rsrgb_distortion, 
                               rgb_intrinsics=rsrgb_intrinsics,
                               rgb_to_ximea_rotation=rsrgb_to_ximea_extrinsics_rotation,
                               rgb_to_ximea_translation=rsrgb_to_ximea_extrinsics_translation,
                                       bag_in_path=f'/home/vasha/st-bravo_analysis/bag/sample_final.bag'
                                       #bag_in_path=f'/home/vasha/st-bravo_analysis/bag/sample_final-Copy{line_number}.bag'
                              )
        ##########################################################################################
        #align depth -> RGB & Ximea for calibrations if it hasn't been done already.
        ##########################################################################################
        if(has_fixations):
            print('This is a human trial with calibrations. Searching for corresponding calibration pngs....')
            for i, caltype in enumerate(calibration_types):
                #create unique identifier for calibration called calib_id
                folderid = trial_line[caltype]
                calib_id = f'{date}_{folderid}' #can't use calib type in name because pre/post are shared for far point
                calibration_raw_folder = os.path.join(bsdir, subject_name, date, str(trial_line[caltype]).zfill(3))
                calibration_ana_folder = os.path.join(analysis_base_dir, subject_name, 'calib', calib_id)
                calibration_bag_file = os.path.join(calibration_ana_folder, 'depth_rgb.bag')
                if(os.path.isfile(calibration_bag_file)):
                    print(f'Skipping {caltype} calibration: its already been created at: {calibration_bag_file}')
                elif(skip_calib_bag):
                    print('Skipping calibration bag alignment because skip_calib_bag is True')
                else:
                    print(f'Running {caltype} calibration: {i+1}/{len(calibration_types)}')
                    print(f'Creating Calibration Bag File for {caltype}: {calibration_bag_file}')
                    #print(calibration_raw_folder)
                    #print(calibration_ana_folder)                    
                    #bag.create_aligned_depth_files(recording_folder=calibration_raw_folder,
                    #bug fix 8/25/23 Vasha - fixing time synchronization bug by writing one frame per bag file.
                    bag.create_aligned_depth_files(recording_folder=calibration_raw_folder,
                               output_folder=calibration_ana_folder,
                               ximea_distortion=ximea_distortion, 
                               ximea_intrinsics=ximea_intrinsics, 
                               rgb_distortion=rsrgb_distortion,
                               rgb_intrinsics=rsrgb_intrinsics,
                               rgb_to_ximea_rotation=rsrgb_to_ximea_extrinsics_rotation,
                               rgb_to_ximea_translation=rsrgb_to_ximea_extrinsics_translation,
                               bag_in_path=f'/home/vasha/st-bravo_analysis/bag/sample_final-Copy{line_number}.bag' #individual bag per trial avoids i/o problems when runing multiple instances of this function
                                                  )
                    #remove bag file if it exists
                    #os.remove(calibration_bag_file)
                             
        else:
            print('This is a manniquen (buddy) trial, no need to convert calibration trial pngs.')       
        
    else:
        print('Skipping Depth Alignment to RGB & Ximea Space')

         
#VD 6/17 not ready to move on with eye tracking calibration until bag convert to ximea & RGB space is done.

#     ##################################################################################
#     # run eye tracking calibration unless we want to skip OR is a manniquen (buddy) trial
#     ##########################################################################################
#     print(skip_calibration, has_fixations)
#     if (not skip_calibration) and (has_fixations):
#         print('Running gaze point mapping for matching calibration trials...')
#         #calibration IDS
#         calib_left_path = os.path.join(bsdir, subject_name, date, str(trial_line['left_eye_cal']).zfill(3))
#         calib_right_path = os.path.join(bsdir, subject_name, date, str(trial_line['right_eye_cal']).zfill(3))
#         calib_val_path = os.path.join(bsdir, subject_name, date, str(trial_line['val_eye_cal']).zfill(3))
#         calib_id_left = f'{date}_{trial_line["left_eye_cal"]}'
#         calib_id_right = f'{date}_{trial_line["right_eye_cal"]}'
#         calib_id_val = f'{date}_{trial_line["val_eye_cal"]}'
#         calib_left_depth = os.path.join(analysis_base_dir, subject_name, 'calib', calib_id_left)
#         calib_right_depth = os.path.join(analysis_base_dir, subject_name, 'calib', calib_id_right)
#         calib_val_depth = os.path.join(analysis_base_dir, subject_name, 'calib', calib_id_val)
#         #calibration filepaths
# #         calibration_map_file_left = os.path.join(analysis_base_dir, subject_name, 'calib', calib_id_left,'calib_map.npy')
# #         calibration_map_file_right = os.path.join(analysis_base_dir, subject_name, 'calib', calib_id_right,'calib_map.npy')
# #         calibration_map_file_val = os.path.join(analysis_base_dir, subject_name, 'calib', calib_id_val,'calib_map.npy')
        
# #         if(os.path.isfile(calibration_map_file_val)):
# #             print(f'Skipping Gaze localization - Validation calibration map file already been created at: {calibration_map_file_val}')
# #         else:
# #             calib.run_gaze_mapper(calib_id_left, calib=True) #write this function - probably need more args
# #             calib.run_gaze_mapper(calib_id_right, calib=True)
# #             calib.run_gaze_mapper(calib_id_val, val=True)

#         #check if gaze mapping has been done for this trial.
#         print('Running Gaze Mapping for Trial')        
#         trial_map_file = os.path.join(ana_folder, 'gaze_map.npy') #this is not the correct path, this will always run
#         if(os.path.isfile(trial_map_file)):
#             print(f'Found Gaze Mapping for this trial at {trial_map_file}. Skipping!')
#         else:
#             print('Didnt find Gaze Mapping for this trial. Runing now...')
#             #first convert pldata file with pupil positions to CSV file.
#             print('First Converting .pldata to .npy for task trial and associated calibration trials')
#             conpl.convert_pldata_csv(os.path.join(data_folder,'offline_data',),
#                                      'offline_pupil.pldata')
#             conpl.convert_pldata_csv(os.path.join(calib_left_path,'offline_data',),
#                                      'offline_pupil.pldata')
#             conpl.convert_pldata_csv(os.path.join(calib_right_path,'offline_data',),
#                                      'offline_pupil.pldata')
#             conpl.convert_pldata_csv(os.path.join(calib_val_path,'offline_data',),
#                                      'offline_pupil.pldata')
#             print('Now Locating Gaze Positions')
#             launch_calibration.run_gaze_mapping(data_folder, 
#                                                 calib_left_path, calib_left_depth,
#                                                 calib_right_path, calib_right_depth, 
#                                                 data_folder, ana_folder,
#                                                 ana_folder, camera_intrinsics_folder)

        
#     else:
#         print('Skipping Eye Tracking & Calibration Analysis')

        
#     ########################################################################################## 
#     # Create Unified Timeline at 200 Hz, upsampling as needed from each stream
#     ##########################################################################################
#     #dont need an option to skip this - it's really quick.
    
#     #convert ximea timestamps into unix timelime (to match other devices)
#     ximea_camera_timestamp_file = os.path.join(data_folder,'ximea', 'timestamps_ximea.tsv')
#     ximea_sync_file = os.path.join(data_folder,'ximea', 'timestamp_camsync_ximea.tsv')
#     print('%%%')
#     print(ximea_camera_timestamp_file)
#     print(ximea_sync_file)
#     ximea_timeline = timeline.convert_ximea_time_to_unix_time(ximea_camera_timestamp_file, ximea_sync_file)[:,1] #first row is framenum, second is timestamp
    
#     #read in other timelines (these are already in unix time)
#     rsrgb_timeline = np.load(os.path.join(data_folder,'world_timestamps.npy'))
#     depth_timeline = np.loadtxt(os.path.join(data_folder, 'depth','timestamps.csv'))

#     timeline_list = [ximea_timeline, rsrgb_timeline, depth_timeline]

#     #if its a human trial need to unify gazepoint and IMU timeline
#     if(has_fixations):
#         #gazepoint_timeline = np.load(os.path.join(ana_folder,'gazepoint_timeline.npy')) #TODO: fix this path
#         left_eye_timeline = np.load(os.path.join(data_folder,'eye0_timestamps.npy'))
#         #right_eye_timeline = np.load(os.path.join(ana_folder,'eye1_timestamps.npy'))
#         head_tracker_timeline = np.load(os.path.join(data_folder,'odometry_head_timestamps.npy'))
#         body_tracker_timeline = np.load(os.path.join(data_folder,'odometry_body_timestamps.npy'))
#         timeline_list.extend([left_eye_timeline, head_tracker_timeline, body_tracker_timeline])
#         #assign timeline with gaze & tracker info
#         common_timeline, idx_matchlist = timeline.assign_common_timeline(timeline_list, target_fps=resample_fps)
#         ximea_framelist, rsrgb_framelist, depth_framelist, gaze_framelist, head_tracker_framelist, body_tracker_framelist = idx_matchlist
#     # if buddy trial only ximea, rsrgb, and depth framelist    
#     else:
#         #assign timeline without gaze & tracker info
#         common_timeline, idx_matchlist = timeline.assign_common_timeline(timeline_list, target_fps=resample_fps)
#         ximea_frame_idx, rsrgb_frame_idx, depth_frame_idx = idx_matchlist
    
        
        
#     ########################################################################################## 
#     # run fourier analysis (Power Spectrum Calculation)    
#     ##########################################################################################
#     if not skip_fourier:
#         print(f'Running Fourier Analaysis for {num_chunks} chunks of size {chunk_pix} pixels....')

#         #make directory
#         fourier_save_path = os.path.join(ana_folder,f'fourier_{chunk_pix}')
#         os.makedirs(fourier_save_path, exist_ok = True) 
        
#         #some calculations for Fourier Analysis
#         chunk_frames = int(chunk_secs*resample_fps)
#         ximea_horizontal_ppd = ximea_dims[1]/ximea_horizontal_fov_deg
#         ppd = ximea_horizontal_ppd
            
#         #assign trace types - Buddy doesn't have foveal trace
#         if(has_fixations):
#             #trace_types = ['fixed_central','fixed_rand_start','foveal','periph_l',
#             #'periph_r','periph_u','periph_d']
#             #trace_types = ['fixed_central', 'foveal', 'fixed_rand_start']
#             trace_types = ['fixed_central', 'foveal',] #skip rand start for now to save time
#             pupil_positions = None ### TODO: Get these from code Ago is writing
#             print('Not Yet implemented Pupil Positions in Ximea Corrdinates. Error is headed your way!')
#         else:
#             #trace_types = ['fixed_central', 'fixed_rand_start']
#             trace_types = ['fixed_central'] #skip rand start for now to save time
#             pupil_positions = None
        
#         #loop over trace types
#         for trace_type in trace_types:
        
#             ##################### save a single example ##########################
#             trace_start_idx, trace_lcorner_xy = tracegen.generate_trace(trace_type=trace_type,
#                                                             chunk_samples=chunk_frames,
#                                                             chunk_dim=chunk_pix, frame_dims=ximea_dims,
#                                                             timeline_len=len(common_timeline),
#                                                             pupil_positions=pupil_positions, ppd=ppd,
#                                                             degrees_eccentricity=degrees_ecc,
#                                                             validation_tstart=None)
#             chunk_frame_indices = ximea_frame_idx[trace_start_idx:trace_start_idx+chunk_frames]
#             #pull out the movie chunk corresponding to this trace
#             movie_chunk = np.zeros((chunk_frames, chunk_pix, chunk_pix, 3))
#             for i, f in enumerate(chunk_frame_indices):
#                 print('*',end='')
#                 frame = cv2.imread(os.path.join(png_folder,f'frame_{f}.png'))
#                 chunk = frame[trace_lcorner_xy[i,1]:trace_lcorner_xy[i,1]+chunk_pix, 
#                                        trace_lcorner_xy[i,0]:trace_lcorner_xy[i,0]+chunk_pix]
#                 movie_chunk[i] = chunk    
#             #get fourier transform of that trace & save output
#             ps_3d, ps_2ds, fqs_space, fqs_time = stf.st_ps(movie_chunk, ppd, resample_fps,
#                                                             cosine_window=cosine_window, rm_dc=True,
#                                                            gpu=gpu_num)
#             np.save(os.path.join(fourier_save_path, f'Example3dPowerSpec_{trace_type}.npy'), ps_3d)
#             np.save(os.path.join(fourier_save_path, f'Example2dPowerSpec_{trace_type}.npy'), ps_2ds)
#             ps_2d_all, ps_2d_vert, ps_2d_horiz, ps_2d_l_diag, ps_2d_r_diag = ps_2ds #ps2ds is list of oriented 2d power specs
#             np.save(os.path.join(fourier_save_path, f'Example3dPowerSpecFreqsSpace_{trace_type}.npy'), fqs_space)
#             np.save(os.path.join(fourier_save_path, f'Example3dPowerSpecFreqsTime_{trace_type}.npy'), fqs_time)

#             ################## NOW LOOP OVER NUM_CHUNKS AND ACCUMULATE #####################
#             print(f'Taking Mean of {num_chunks} traces of type {trace_type}...')
#             #store mean values in arrays
#             ps_3d_mean = np.zeros_like(ps_3d)
#             ps_2d_all_mean = np.zeros_like(ps_2d_all)
#             ps_2d_vert_mean = np.zeros_like(ps_2d_vert)
#             ps_2d_horiz_mean = np.zeros_like(ps_2d_horiz)
#             ps_2d_l_diag_mean = np.zeros_like(ps_2d_l_diag)
#             ps_2d_r_diag_mean = np.zeros_like(ps_2d_r_diag)
            
#             for i in range(num_chunks):    
#                 trace_start_idx, trace_lcorner_xy = tracegen.generate_trace(trace_type=trace_type,
#                                                                             chunk_samples=chunk_frames,
#                                                                             chunk_dim=chunk_pix, frame_dims=ximea_dims,
#                                                                             timeline_len=len(common_timeline),
#                                                                             pupil_positions=pupil_positions, ppd=ppd,
#                                                                             degrees_eccentricity=degrees_ecc,
#                                                                             validation_tstart=None)
#                 chunk_frame_indices = ximea_frame_idx[trace_start_idx:trace_start_idx+chunk_frames]
#                 #pull out the movie chunk corresponding to this trace
#                 movie_chunk = np.zeros((chunk_frames, chunk_pix, chunk_pix, 3))
#                 for i, f in enumerate(chunk_frame_indices):
#                     print('*',end='')
#                     frame = cv2.imread(os.path.join(png_folder,f'frame_{f}.png'))
#                     chunk = frame[trace_lcorner_xy[i,1]:trace_lcorner_xy[i,1]+chunk_pix, 
#                                            trace_lcorner_xy[i,0]:trace_lcorner_xy[i,0]+chunk_pix]
#                     movie_chunk[i] = chunk    
#                 #get fourier transform of that trace & save output
#                 ps_3d, ps_2ds, fqs_space, fqs_time = stf.st_ps(movie_chunk, ppd, resample_fps,
#                                                             cosine_window=cosine_window, rm_dc=True, gpu=gpu_num)
#                 ps_2d_all, ps_2d_vert, ps_2d_horiz, ps_2d_l_diag, ps_2d_r_diag = ps_2ds

#                 ps_3d_mean += ps_3d
#                 ps_2d_all_mean += ps_2d_all
#                 ps_2d_vert_mean += ps_2d_vert
#                 ps_2d_horiz_mean += ps_2d_horiz
#                 ps_2d_l_diag_mean += ps_2d_l_diag
#                 ps_2d_r_diag_mean += ps_2d_r_diag
    
#             #take mean of all trace types for this trial
#             print(f'Finished Calculating Power Spectra for Trace Type {trace_type}. Now saving...')
#             np.save(os.path.join(fourier_save_path, f'Mean3dPowerSpec_{trace_type}.npy'), ps_3d_mean)
#             np.save(os.path.join(fourier_save_path, f'Mean2dAllPowerSpec_{trace_type}.npy'), ps_2d_all_mean)
#             np.save(os.path.join(fourier_save_path, f'Mean2dVertPowerSpec_{trace_type}.npy'), ps_2d_vert_mean)
#             np.save(os.path.join(fourier_save_path, f'Mean2dHorizPowerSpec_{trace_type}.npy'), ps_2d_horiz_mean)
#             np.save(os.path.join(fourier_save_path, f'Mean2dLDiagPowerSpec_{trace_type}.npy'), ps_2d_l_diag_mean)
#             np.save(os.path.join(fourier_save_path, f'Mean2dRDiagPowerSpec_{trace_type}.npy'), ps_2d_r_diag_mean)
#             np.save(os.path.join(fourier_save_path, f'Mean3dPowerSpecFreqsSpace_{trace_type}.npy'), fqs_space)
#             np.save(os.path.join(fourier_save_path, f'Mean3dPowerSpecFreqsTime_{trace_type}.npy'), fqs_time)    
#             stf.da_plot_power(ps_2d_all_mean, fqs_space, fqs_time,
#                       figname=f'MeanPowerSpec_{num_chunks}_chunks_{trace_type}', 
#                       saveloc=fourier_save_path, show_onef_line=True, logscale=True,
#                               nsamples = 7,legend_loc=1,
#                               cmap_p='binary_r', cmap_s='summer', cmap_t='autumn')

        
#     else:
#         print('Skipping Fourier Analysis')
        
    ########################################################################
    ## FINISHED
    ###############################################################################
    print(f'All Done with analysis for subject: {subject_name}, task: {task_name}, iter: {task_iter}!')
    

def main():
    args=sys.argv[1:]
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--analysis_path", help="path to analysis (output of this script)", dest="analysis_path", type=str, 
                        default='/hmet_analysis_2')
    parser.add_argument("-t","--trial_list_path", help="path to csv file containing trial info", dest="trial_list_path", type=str, 
                        default='~/st-bravo_clean/trial_list.csv')
    parser.add_argument("-l", "--line_number", help="line number in trial_list to analyze", dest="line_number", type=int)
    parser.add_argument("-p", "--skip_convert_png", help="skip convert ximea .bin to pngs", dest="skip_convert_png", default=True)
    parser.add_argument("-b", "--skip_bag_align", help="skip bag alignment", dest="skip_bag_align", default=False)
    parser.add_argument("-c", "--skip_calibration", help="skip eye tracking calibration", dest="skip_calibration", default=True)
    parser.add_argument("-x", "--skip_calib_bag", help="bag alignment for calibration", dest="skip_calib_bag", default=True)
    parser.add_argument("-f", "--skip_fourier", help="skip fourier analysis", dest="skip_fourier", default=True)
    parser.add_argument("-y", "--skip_gaze_dependencies", help="dont run code that needs gaze", dest="skip_gaze_dependencies", default=True)
    parser.add_argument("-g", "--gpu_num", help="specify gpu number for cupy (default 0)", dest="gpu_num", type=int,default=0)
    #parser.add_argument("-s", "--stop_time", help="time to stop analysis")
   
    
    args = parser.parse_args(args)
    print(f'analyzing line {args.line_number} of {args.trial_list_path}')
    
    print(f'Skipping PNG conversion? {args.skip_convert_png}')
    print(f'Skipping BAG alignment? {args.skip_bag_align}')
    print(f'Skipping Calibration Trial? {args.skip_calibration}')
    print(f'Skipping Fourier Analysis? {args.skip_fourier}')
    print(f'Skipping Gaze Dependencies? {args.skip_gaze_dependencies}')

    #launch analysis
    run_analysis(args.analysis_path, 
                 args.trial_list_path, args.line_number, 
                 eval(str(args.skip_convert_png)), eval(str(args.skip_bag_align)), 
                 eval(str(args.skip_calibration)), eval(str(args.skip_calib_bag)), eval(str(args.skip_fourier)),
                 eval(str(args.skip_gaze_dependencies)), args.gpu_num)

    
if __name__ == "__main__":
   main()