import numpy as np
#generate a trace
#generate a trace
def generate_trace(trace_type, chunk_samples, chunk_dim, frame_dims, timeline_len, pupil_positions, ppd, degrees_eccentricity=None, validation_tstart=None):
    '''
    Generate a single trace (chunk) through a frame.
    Params:
        trace type (str): what type of trace should be generated
            'fixed_central': just take the center of the frame at a random time
            'fixed_rand_start': take a fixed random place in the frame at a random time
            'foveal: use eye trace data at a random time, and choose a box around this point (position in frame is variable in time)
            'periph_l: use eye trace data at a random time, but take 15 degrees left of fixation and choose a box around this point (position in frame is variable in time)
            'periph_r: use eye trace data at a random time, but take 15 degrees right of fixation and choose a box around this point (position in frame is variable in time)
            'periph_u: use eye trace data at a random time, but take 15 degrees up from fixation (upper visual field) and choose a box around this point (position in frame is variable in time)
            'periph_d: use eye trace data at a random time, but take 15 degrees down from fixation (lower visual field) and choose a box around this point (position in frame is variable in time)
            'foveal: use eye trace data at a random time, and choose a box around this point (position in frame is variable in time)
        chunk_samples (int): number of samples in a chunk (fps*seconds)
        chunk_dim (int): edge dims of chunk in pixels
        frame_dims (int tuple): size of full frame in pixels
        timeline_len (int): length of timeline (should be length of pupil positions but this is sometimes None for buddy trials)
        pupil_positions (2D array): array with two columns, xpositions, ypositions
        ppd: pixels per degree (for peripheral trace finding)
        degrees_eccentrity (how far to go out in eccentricity)
        validation_tstart(int): for testing/validation purposes, specify a start index 

    '''
    
    #how many pixels do we go out from the fixation to match degrees specified in periphery?
    if(degrees_eccentricity is not None):
        periph_pixels = np.float(degrees_eccentricity)*ppd #how many pixels do we go out for an eccentricity trace
    #print(pupil_positions)
    #print(pupil_positions.shape)
    #print(periph_pixels)
    print('^',end='')
    
    #pupil_positions shape is [17253,2]
    #frame_dims is shape      [1544,2064] (y,x)
    
    #locate in time randomly
    if(validation_tstart is None):
        trace_start_idx = np.random.randint(0, timeline_len - chunk_samples)
    else:
        trace_start_idx = validation_tstart

    #locate in space depending on trace type
    if(trace_type == 'fixed_central'):
        trace_lcorner_xy = np.tile([frame_dims[1]//2-chunk_dim//2,
                                    frame_dims[0]//2-chunk_dim//2],
                                   (chunk_samples,1)) #top left corner of chunk
        
    if(trace_type == 'fixed_rand_start'):
        #locate in space
        trace_lcorner_xy = np.tile([np.random.randint(0,frame_dims[1]-chunk_dim),
                                   np.random.randint(0,frame_dims[0]-chunk_dim)],
                                  (chunk_samples,1)) #top left corner of chunk 
    
    elif(trace_type in(('foveal','periph_l','periph_r','periph_u','periph_d'))):
        found_valid_fixation = False
        while(found_valid_fixation==False):
            print('*',end='') #indicates attempt at fixation

            trace_foveal_positions = pupil_positions[trace_start_idx:trace_start_idx+chunk_samples,:]
            #locate in space
            if(trace_type=='foveal'):
                trace_x_center = np.rint(trace_foveal_positions[:,0]).astype(int)
                trace_y_center = np.rint(trace_foveal_positions[:,1]).astype(int)
            elif(trace_type=='periph_l'):
                trace_x_center = np.rint(trace_foveal_positions[:,0]-periph_pixels).astype(int)
                trace_y_center = np.rint(trace_foveal_positions[:,1]).astype(int)
            elif(trace_type=='periph_r'):
                trace_x_center = np.rint(trace_foveal_positions[:,0]+periph_pixels).astype(int)
                trace_y_center = np.rint(trace_foveal_positions[:,1]).astype(int)
            elif(trace_type=='periph_u'):
                trace_x_center = np.rint(trace_foveal_positions[:,0]).astype(int)
                trace_y_center = np.rint(trace_foveal_positions[:,1]-periph_pixels).astype(int)
            elif(trace_type=='periph_d'):
                trace_x_center = np.rint(trace_foveal_positions[:,0]).astype(int)
                trace_y_center = np.rint(trace_foveal_positions[:,1]+periph_pixels).astype(int)
            else:
                print('Trace Type not found!! Error for you!!!!')
                
            #once we have the xy and y central position, get the upper left corner of the trace for easy extraction from movie
            trace_lcorner_xy = np.array(np.array((trace_x_center, trace_y_center)).T-(chunk_dim//2))
            #print(trace_x_center,trace_y_center)
            #print(trace_xy_lcorner.shape)
            
            #check to make sure we don't exit the frame (this is likely to happen if degrees_eccentricity is high)
            x_dims_ok = (np.all((trace_lcorner_xy[:,0]>0)) and np.all((trace_lcorner_xy[:,0]+chunk_dim<frame_dims[1])))
            y_dims_ok = (np.all((trace_lcorner_xy[:,1]>0)) and np.all((trace_lcorner_xy[:,1]+chunk_dim<frame_dims[0])))
            
            #and make sure pupil positions are not nan during this time
            no_nans = not(np.isnan(np.sum(trace_foveal_positions)))
            
            if all((no_nans, x_dims_ok, y_dims_ok)):
                found_valid_fixation = True
            else:
                print('#',end='')
                #we need a different temporal location because we failed with the current one
                trace_start_idx = np.random.randint(0, timeline_len - chunk_samples)
#                 info about why we failed
#                 print(f'FAILED! Fixation type: {trace_type }')
#                 print(f'NaNs? {no_nans}')
#                 print(f'ydims OK? {x_dims_ok}')
#                 print(f'xdims OK? {y_dims_ok}')
                    
#                 print('Shape of data')
#                 print(timeline_len, chunk_dim)
#                 print('Start index')
#                 print(trace_start_idx)
#                 print('Trace Centers')
#                 print(trace_x_center, trace_y_center)
#                 print('Trace Corners')
#                 print(trace_xy_lcorner)
                #found_valid_fixation = False #FOR NOW
    return(trace_start_idx, trace_lcorner_xy)

