import numpy as np

def convert_ximea_time_to_unix_time(timestamp_file, sync_file):
    '''
    Convert the ximea camera times to unix timestamps
    Params:
        timestamp_file (str): path to .csv file with ximea timesstamps
        sync_file (str): path to .csv file with sync information
    Returns:
        unix_timestamp_array (2d np array): Unix Timestamps Inferred
    '''
    with open(timestamp_file, 'r') as f:
        ts_table=list(zip(line.strip().split('\t') for line in f))
    ts_table = np.squeeze(np.array(ts_table[1:]).astype(np.double))

    with open(sync_file, 'r') as f:
        sync_table=list(zip(line.strip().split('\t') for line in f))
        
    (unix_pre, cam_pre, _) = np.array(sync_table[1][0][1:]).astype(np.double)
    (unix_post, cam_post, _) = np.array(sync_table[2][0][1:]).astype(np.double)

    #how far off have the computer and camera timestamps drifted?
    t_elapsed_unix = np.double(unix_post) - np.double(unix_pre)
    t_elapsed_cam = np.double(cam_post) - np.double(cam_pre)
    drift = np.abs(t_elapsed_unix - t_elapsed_cam)
    print(f'Ximea: Time Elapsed: {t_elapsed_unix} seconds')
    print(f'Ximea: Time Drift pre to post: {drift} seconds')
    
    # We assume here that time.time() in Linux's 0.001s precision is better than camera's.
    # Convert Camera timestamps to Unix timestamps.
    #first convert to [0,1]
    t_cam_converted = (ts_table[:,2] - cam_pre) / (cam_post - cam_pre)
    #then convert to wall time
    t_cam_converted = (t_cam_converted * (unix_post - unix_pre)) + unix_pre
    
    #assume time in camera is linear, and just change offset at pre.
    #t_cam_converted = ts_table[:,2] - ts_table[0,2] + unix_pre
    
    #print(f'Start at {t_cam_converted[0]}, end at {t_cam_converted[-1]}')
    
    #dont add unix time to data, just send unix times and frame numbers
    t_cam_converted = np.append(np.expand_dims(ts_table[:,0],1), np.expand_dims(t_cam_converted,1),axis=1)
    
    return(t_cam_converted)



def calc_olap_fraction(list_of_timeseries):
    '''
    Sanity check funtion to make sure there is a decent overlap between all the time series. 
    Arguments:
        list_of_timeseries (list of 1d arrays): list of all the timeseries to be comapred
    Returns:
        olap (list of floats): The fraction overap for each item in the list as compared to the full span all all timeseries in list
    '''
    starts = [t[0] for t in list_of_timeseries]
    ends = [e[-1] for e in list_of_timeseries]
    full = np.arange(np.min(starts), np.max(ends), 1) #make a full timeseries to calc percentage
    olapped = np.arange(np.max(starts), np.min(ends),1) #make a timeseries with same step size only where all timeseries overlap
    olap = len(olapped)/len(full)
    polap = olap*100
    print(f'{len(list_of_timeseries)} Sensors: {len(olapped)} shared seconds for {len(full)} total seconds of collection. This is {polap:.2f}% overlap!')
    return(olap)

def calc_dropped_frames(timeseries, target_fps, sensor_name='Unamed Sensor'):
    '''
    Sanity check funtion to make sure there are not too many dropped frames
    Arguments:
        timeseries (1d array): list that is a timeseries
        target_fps (float/int):the target frame rate
    Returns:
        frac_dropped (float): The fraction of frames dropped
    '''
    print(f'{sensor_name} goes from {timeseries[0]} to {timeseries[-1]} and has {len(timeseries)} timepoints')
    full = np.arange(timeseries[0], timeseries[-1], 1./target_fps) #make a full timeseries to calc percentage
    num_dropped = len(full)-len(timeseries)
    if(num_dropped < 0):
        print(f'{sensor_name} had {-1*num_dropped} extra frames. If this is a small number its a rounding error and there were very few or zero frames dropped, yay!')
        frac_dropped = 0
    else:
        frac_dropped = 1- (np.float(len(timeseries))/len(full))
        perc_dropped = frac_dropped * 100
        print(f'{sensor_name}: {num_dropped} dropped out of {len(full)} expected. This is {perc_dropped:.2f}% dropped!')
    return(frac_dropped)

def assign_common_timeline(timeline_list, target_fps, start_at_zero=False, starttime=0, endtime=''):
    '''
    Readin timestamps from all devices and match them to a common timestamp, repeating frames if needed.
    
    Params:
        timeline_list (list of float lists): list of timelines for various devices (length n_devices)
        target_fps (int): The sample rate in frames per second for sampling (should me max of all devices)
        start_at_zero (bool): should we start timeline at zero?
        starttime (int or ''): seconds from beginning of timeline to start (when beginning of collection has bug)
        endtime (int or ''): seconds from start of timeline to end (when end of collection has bug)
        
    Returns:
        common_timelines (list of floats): common timeline for all devices (starting at zero)
        sampleidxmatch_list (list of int lists): list of sample number of device assigned to each time in common_timeline. (list has length n_devices, each item in list has length of common_timeline)
        
    '''
    
    #create common timeline
    #print(len(timeline_list))
    #[print(len(t)) for t in timeline_list]
    #[print(timeline[-1]) for timeline in timeline_list]
    start_timestamp = np.max([timeline[0] for timeline in timeline_list])
    end_timestamp = np.min([timeline[-1] for timeline in timeline_list])
    
    #adjust start and end time if applicable
#     if not np.isnan(starttime):
#         start_timestamp = start_timestamp + int(starttime)
#     if not np.isnan(endtime):
#         end_timestamp = start_timestamp + int(endtime)
    
    common_timeline = np.arange(start_timestamp, end_timestamp, 1./target_fps)
    sampleidxmatch_list = [np.zeros_like(common_timeline) for _ in timeline_list]
    
    for i, sample_timeline in enumerate(timeline_list):
        sampleidxmatch_list[i] = [np.argmin(np.abs(sample_timeline - t)) for t in common_timeline]

#     for i, t in enumerate(common_timeline):
#         common_timeline_match
#         ximea_common_timeline_match[i] = np.argmin(np.abs(timestamps_ximea - t))
#         pupil_eye0_common_timeline_match[i] = np.argmin(np.abs(pupil_ts_eye0 - t))
#         pupil_eye1_common_timeline_match[i] = np.argmin(np.abs(pupil_ts_eye1 - t))
        
    if(start_at_zero):
        common_timeline = common_timeline - common_timeline[0]
        
    return(common_timeline, sampleidxmatch_list)

            
#     common_timeline_table = np.array((common_timeline, ximea_common_timeline_match, pupil_eye0_common_timeline_match, pupil_eye1_common_timeline_match, during_task, during_calibration)).T
#     common_timeline_table_colnames = 'common_timeline\tximea_frame\tpupil_eye0_frame\tpupil_eye1_frame\tduring_task\tduring_calibration'
#     common_timeline_file = os.path.join(analysis_folder,'common_timeline.tsv')            
#     common_timeline_file_human = os.path.join(analysis_folder,'common_timeline.tsv')
#     np.savetxt(common_timeline_file_human, common_timeline_table, delimiter='\t', header=common_timeline_table_colnames)
#     common_timeline_file = os.path.join(analysis_folder,'common_timeline.npy')
#     np.save(common_timeline_file, common_timeline_table)
#     return(common_timeline_table)