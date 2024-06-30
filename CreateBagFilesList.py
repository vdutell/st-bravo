import pandas as pd
import os
import csv

def createbagfileslist():
    
    #open csv file
    output_csv = './bag_file_list.csv'
    with open(output_csv,'w',newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['anadir','subject','task','calfolder','rgb_ximea','nframes'])
        #keep track of calibration output folders we've already reported
        calibration_folders = []
    
        #skip buddy trials for now
        for triallistlinenumber in range(42):

            trial_list_path = '~/st-bravo_analysis/trial_list.csv'
            trial_line = pd.read_csv(trial_list_path).iloc[triallistlinenumber]
            bsdir = trial_line['base_dir']
            date = trial_line['folder']
            subject_name = trial_line['subject']
            trial_num = str(trial_line['trial']).zfill(3)
            task_name = trial_line['task']
            task_iter = str(trial_line['iter'])

            if(subject_name == 'dd'):
                anadir = '/hmet_analysis'
            elif(subject_name == 'ad'):
                anadir= '/hmet_analysis'
            else:
                anadir = '/hmet_analysis_2'

            #main trials        
            for frames in ['rgb','ximea']:
                frames_folder = os.path.join(anadir,subject_name,task_name,task_iter,f'{frames}_aligned_depth')
                
                #get number of frames
                nframes = os.popen(f'ls {frames_folder} | wc -l')
                writer.writerow([anadir,subject_name,task_name, 'task', frames, nframes.read()])


            #calibration
            for task in ['left_eye_cal','right_eye_cal','val_eye_cal']:
                calib_number = trial_line[task]
                calibration_folder = f'{date}_{calib_number}'
                if calibration_folder not in calibration_folders:
                    calibration_folders.append(calibration_folder)
                    for frames in ['rgb','ximea']:
                        frames_folder = os.path.join(anadir,subject_name,'calib',calibration_folder,f'{frames}_aligned_depth/')
                        #print(triallistlinenumber, anadir, subject_name, task, frames)
                        #print(frames_folder)

                        #get number of frames
                        nframes = os.popen(f'ls {frames_folder} | wc -l')
                        writer.writerow([anadir,subject_name,task,calibration_folder, frames, nframes.read()])

    
    return()

createbagfileslist()