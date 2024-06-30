import os
import subprocess
import glob

# Utility to invoke ffmpeg (external program) to compile frames into a movie file
def compile_frames_mp4_lossless(frames_base_dir, movie_type='metamer', movie_suffix='', movie_fps=30, codec='libx265', verbose=True, remove_pngs=False, movie_base_dir=None, crf_quality=20):
    
    #if haven't specified export directory for movie, make it same as frames.
    if movie_base_dir is None:
        movie_base_dir = frames_base_dir
    
    out_video_name = os.path.join(movie_base_dir, f'{movie_type}.mp4')
    inputpattern=os.path.join(frames_base_dir, f'{movie_type}_Frame_%03d.png')
    
    print(f'fps: {movie_fps}')
    # Run ffmpeg using commandline to compile frames to mp4 movie
    cmd = ['ffmpeg', 
           '-y',                        # allow overwriting movie file if it already exists
           '-framerate', f'{movie_fps}',         # input should come in at frame rate, output will inheret
           '-i', f'{inputpattern}',          # filename pattern, used to find frame image files
           '-c:v', f'{codec}',
           '-pix_fmt', 'yuv420p',
           '-crf', f'{crf_quality}',
           '-vsync', '0']                   # don't drop or skip frames
    
    cmd.append(out_video_name)
    # execute command, check return value is not an error
    if verbose:
        subprocess.check_output(cmd)  # prints command output and errors to terminal
    else:
        subprocess.run(cmd,check=True,capture_output=True)  # run command silently, but check return value for errors
    print(f'movie compiled to {out_video_name}')
    
    if remove_pngs:
        pngs_pattern = os.path.join(movie_base_dir, f'{movie_type}_Frame*.png')
        pngs = glob.glob(pngs_pattern)
        _ = [os.remove(png) for png in pngs]       
        print('Deleted Pngs')