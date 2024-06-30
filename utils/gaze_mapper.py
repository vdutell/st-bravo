def run_gaze_mapper(ana_folder, calib=False, val=False):
    ''' Run Gaze Mapping Code (Makes Matlab Calls)
    Run the gaze mapping routines, calculating calibration map if  calib=True, and returning validation if val=True. If Both are false, this is a trial, we should not expect target positions, and we will simply return gaze positions.
    '''
    
    if calib:
        print(f'Running Calibration for: {ana_folder}')
    if val:
        print(f'Running Validation for: {ana_folder}')
    else:
        print(f'Running Gaze Mapper for: {ana_folder}')
    
    