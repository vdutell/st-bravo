import numpy as np


def cos_filter_3d(movie):
    #filter first with cosine window
    (dim1,dim2,dimt) = np.shape(movie)
    d1 = np.hanning(dim1)
    d1 = np.transpose(np.tile(d1,(dim2,dimt,1)),axes=(2,0,1))

    d2 = np.hanning(dim2)
    d2 = np.transpose(np.tile(d2,(dimt,dim1,1)),axes=(1,2,0))

    dt = np.hanning(dimt)
    dt = np.tile(dt,(dim1,dim2,1))

    cosfilter = d1*d2*dt
    
    filtered_movie = cosfilter * movie[:,:,:]

    return(np.array(filtered_movie))


# from https://code.google.com/archive/p/agpy/downloads
def azimuthalAverage(image, nyquist, center=None, bin_in_log=False):
    """      
    image - The 2D image (2d power spectrum)
    nyquist - max frequency value (assume same for x and y)
    center - The [x,y] pixel coordinates used as the center. The default is 
             None, which then uses the center of the image (including 
             fracitonal pixels).
    
    """
    # Calculate the indices from the image
    y, x = np.indices(image.shape)
    num_bins = np.min(image.shape)

    if not center:
        center = np.array([(x.max()-x.min())/2.0, (y.max()-y.min())/2.0])
    #ASSUME HERE THAT MAX FREQUENCY IS EQUAL ON BOTH AXES & GRANULARITY VARIES ***
    normalized = ((x-center[0])/np.max(x),(y-center[1])/np.max(y))
    r = np.hypot(normalized[0], normalized[1])
    #don't calculate corners
    keep_circle = np.where(r<=np.max(y))
    r = r[keep_circle]
    image = image[keep_circle]

    # number of bins should be equivalent to the number of bins along the shortest axis of the image.
    if(bin_in_log):
        bin_edges = np.histogram_bin_edges(np.log(r), num_bins)
        bin_edges = np.exp(bin_edges)
    else:
        bin_edges = np.histogram_bin_edges(r,num_bins)
    
    r_binned = np.digitize(r, bin_edges)
    binmean = np.zeros(num_bins)
    for i in range(num_bins):
        #if(len(r_binned[r_binned==i+1])>0):
        binmean[i] = np.mean(image[np.where(r_binned==i+1)])
        #else:
        #    binmean[i] = 0
    bin_centers = bin_edges[:-1] + ((bin_edges[1]-bin_edges[0])/2)
    bin_centers = (bin_centers/np.max(bin_centers))*nyquist

    return(binmean, bin_centers)

def cubify(arr, newshape):
    
    '''
        chunks array (movie) into equal smaler cubes.
        Taken directly From: https://stackoverflow.com/questions/42297115/numpy-split-cube-into-cubes
        
    '''
    oldshape = arr.shape

    #print(oldshape, newshape)
    
    #make sure we aren't asking for longer movies than we started with!
    assert oldshape[0]>=newshape[0], f"desired movie length is longer than original: {oldshape[0]} < {newshape[0]}!"
    
    repeats = (np.array(oldshape) / np.array(newshape)).astype(int)
    
    #print(repeats)
    tmpshape = np.column_stack([repeats, newshape]).astype('int').ravel()
    #print(tmpshape)
    order = np.arange(len(tmpshape))
    order = np.concatenate([order[::2], order[1::2]])
    # newshape must divide oldshape evenly or else ValueError will be raised
    new = arr.reshape(tmpshape).transpose(order).reshape(-1, *newshape)
    return new


def average_3d_ps(movies, chunkshape, fps, ppd):

    """ 
    Calculate the mean power spectrum for a list of many movies
    
    Parameters
    ----------
    movies:      list of 3d numpy arrays defining movies for 3d fourier transform analysis.
    chunklen:   integer number of frames defining the length of movie 'chunks'
    fps:        frame rate of movie
    ppd:        pixels per degree of movie
    
    Returns:
    --------
    psmean:     mean spatiotemporal fourier transform
    fq1d:       spatial frequency spectrum (dimensions of 0 axis of psmean)
    fqt:        temporal frequency spectrum (dimensions of 1 axis of psmean)
    
    """
    
    #keep a mean
    nmovies = len(movies)
    pssum = 0
    
    #ftspecs = []
    for movie in movies:
        
        ps, fq1d, fqt = st_ps(movie, chunkshape, fps, ppd)
        #print(ft.shape, fq1d.shape, fqt.shape)
        #ftspecs.append(ft)
        pssum = ftsum + ps
    
    psmean = pssum / nmovies
    #print(len(ftspecs), ftspecs[1].shape)
    #ftmean = np.mean(np.array(ftspecs),axis=0)
    #print(ftmean.shape)
    return(psmean, fq1d, fqt)


# def st_ps(movies, chunkshape, fps, ppd, cardinals=False):
#     '''
#     Calculate the spatiotemporal power spectrum of a movie.
    
#     Parameters
#     ----------
#     movies:      3d numpy array definig movie for 3d fourier transform analysis.
#     chunkshape:  3ple defining shape (frames,x,y) of movie 'chunks'
#     fps:         frames per secton of movie
#     ppd:        pixels per degree of movie
#     cardinals:  return cardinal averaged PS also
    
#     Returns:
#     --------
#     ps:       either one power spectrum averaged, or a list with the total averaged, top, bottom, left, right directions avgd
#     fq1d:       spatial frequency spectrum (dimensions of 0 axis of psmean)
#     ft1d:       temporal frequency specturm (dimensions of 1 axis of psmean)
    
#     '''
    
#     #If movie is in color, average three color channels to get greyscale
#     #if(movie.ndim > 3):
#     #    movie = np.mean(movie,axis=3)
    
#     #in case we want to average over chunks rather than the whole movie
#     movies = cubify(movies, chunkshape)
    
#     psmovies = []
#     for movie in movies:
#         print('*',end='')
#         #remove mean (DC component)
#         movie = movie - np.mean(movie)       
#         #3d ft for each chunk
#         psmovies.append(np.abs(np.fft.fftn(movie))**2)
        
#     del(movies) #save space: we are done with raw movies
#     psmovies=np.array(psmovies)
    
#     #take mean over all ftchunks to get one ftchunk
#     mean_psmovie = np.mean(psmovies,axis=0)
#     #do fft shifts to make football
#     mean_psmovie = np.fft.fftshift(mean_psmovie)

#     #array to hold azmaverage
#     azmovie = []
#     ##spin 360 degrees to get spatial mean (temporal still separate)
#     for f in range(np.shape(mean_psmovie)[0]):
#         azmovie.append(azimuthalAverage(mean_psmovie[f],ppd/2.))

#     #only take positive side (real)
#     azmovie = np.abs(np.array(azmovie[int(chunkshape[0]/2):])).T
    
#     del(mean_psmovie)
    
#     #get the sampling rate for azmavgd
#     freqspace1d = np.fft.fftfreq(chunkshape[1], d=1./ppd)[0:int(np.floor(chunkshape[1]/2))-1]
#     #get the sampling rates
#     freqspacefull = np.fft.fftshift(np.fft.fftfreq(chunkshape[1], d=1./ppd))
#     # we didn't cicularly average in temporal space, so take only the positive half
#     freqtime = np.fft.fftshift(np.fft.fftfreq(chunkshape[0], d=1./fps))[int(chunkshape[0]/2):]
    
#     #remove DC component
#     #azmovie = azmovie[1:,1:]
#     #freqspace1d = freqspace1d[1:]
#     #freqtime = freqtime[1:]    
    
#     return(azmovie, freqspace1d, freqtime)


# def azm_movie(ps, fps, ppd):
#     '''
#     Take the azimuthal average for a movies 3d power spectrum
    
#     Params:
#         ps (d array) power spectrum to be averaged
#         fps (int): framerate of movie
#         ppd (float): pixels per degree of movie
#     Returns:
#         az_ps (2d array): azimuthally averaged power spectrum
#         fq_sp_1d (1d array): spatial frequencies
#         fq_time: temporal frequencies
    
#     '''
#     ps_shape = np.shape(ps)
#     print(ps_shape)
    
#     az_ps = []
#     #spin
#     for f in range(np.shape(ps)[0]):
#         az_ps.append(azimuthalAverage(ps[f], ppd/2.))
#     #only take positive side (real)
#     az_ps = np.abs(np.array(az_ps[int(ps_shape[0]/2):])).T
      
#     #get the sampling rate for azmavgd
#     fq_sp_1d = np.fft.fftfreq(ps_shape[1], d=1./ppd)[0:int(np.floor(ps_shape[1]/2))-1]
#     # we didn't cicularly average in temporal space, so take only the positive half
#     fq_time = np.fft.fftshift(np.fft.fftfreq(ps_shape[0], d=1./fps))[int(ps_shape[0]/2):]
    
#     return(az_ps, fq_sp_1d, fq_time)

# def st_ps(movie, fps, ppd):
#     '''
#     Calcaulte the spatiotemporal power spectrum of a movie
    
#     Params:
#         movie (3 or 4d array) movie to be transformed
#         fps (int): framerate of movie
#         ppd (float): pixels per degree of movie
    
    
#     Returns:
#         ps
#         spatial_fq
#         temporal_fq
#     '''
    
#     #If movie has color channel, remove it.
#     if(movie.ndim > 3):
#         movie = np.mean(movie,axis=3)
    
#     movieshape = np.shape(movie)
    
#     #remove DC component
#     movie = movie - np.mean(movie)
#     #3d ft for each chunk
#     ps = np.abs(np.fft.fftn(movie))**2
#     spatial_fq = np.fft.fftshift(np.fft.fftfreq(movieshape[1], d=1./ppd))
#     temporal_fq = np.fft.fftshift(np.fft.fftfreq(movieshape[0], d=1./fps))
                                  
#     return(ps, spatial_fq, temporal_fq)
    

# # def st_ps(movie, chunkshape, fps, ppd):
# #     '''
# #     Calculate the spatiotemporal power spectrum of a movie.
    
# #     Parameters
# #     ----------
# #     movies:      list of 3d numpy arrays definig movie for 3d fourier transform analysis.
# #     chunkshape:  3ple defining shape (frames,x,y) of movie 'chunks'
# #     fps:         frames per secton of movie
# #     ppd:        pixels per degree of movie
    
# #     Returns:
# #     --------
# #     mftchunk
# #     azmchunk
# #     freqspace1d
# #     freqspacefull
# #     freqtime
    
# #     '''
    
# #     #If movie is in color, average three color channels to get greyscale
# #     if(movie.ndim > 3):
# #         movie = np.mean(movie,axis=3)
    
# #     #in case we want to average over chunks rather than the whole movie
# #     movies = cubify(movie, chunkshape)
# #     del(movie) #save memory we dobn't need the movie anymore
    
# #     psmovies = []
# #     for movie in movies:
# #         #remove mean (DC component)
# #         movie = movie - np.mean(movie)       
# #         #3d ft for each chunk
# #         psmovies.append(np.abs(np.fft.fftn(movie))**2)
        
# #     del(movies) #save space: we are done with raw movies
# #     psmovies=np.array(psmovies)
    
# #     #take mean over all ftchunks to get one ftchunk
# #     mean_psmovie = np.mean(psmovies,axis=0)
# #     #do fft shifts to make football
# #     mean_psmovie = np.fft.fftshift(mean_psmovie)

# #     #array to hold azmaverage
# #     azmovie = []
# #     ##spin to get mean
# #     for f in range(np.shape(mean_psmovie)[0]):
# #         azmovie.append(azimuthalAverage(mean_psmovie[f]))
# #     del(mean_psmovie)
# #     #only take positive side (real)
# #     azmovie = np.abs(np.array(azmovie[int(chunkshape[0]/2):])).T
# #     #print(f'azmovie: {azmovie.shape}')
        
# #     #azmchunk = (azmchunk[int(chunklen/2):] + azmchunk[int(chunklen/2):0:-1]) / 2
      
# #     #get the sampling rate for azmavgd
# #     freqspace1d = np.fft.fftfreq(chunkshape[1], d=1./ppd)[0:int(np.floor(chunkshape[1]/2))-1]
# #     #get the sampling rates
# #     freqspacefull = np.fft.fftshift(np.fft.fftfreq(chunkshape[1], d=1./ppd))
# #     # we didn't cicularly average in temporal space, so take only the positive half
# #     freqtime = np.fft.fftshift(np.fft.fftfreq(chunkshape[0], d=1./fps))[int(chunkshape[0]/2):]
    
# #     #remove DC component
# #     #azmovie = azmovie[1:,1:]
# #     #freqspace1d = freqspace1d[1:]
# #     #freqtime = freqtime[1:]    

# #     # not sure this is right. what were we doing here??
# #     # normalize the fft based on dx and dy
# #     #dspace = freqspace1d[1] - freqspace1d[0]
# #     #dtime = freqtime[1] - freqtime[0]
# #     #azmchunk = azmchunk - np.mean(azmchunk)
# #     #azmovie *= np.real(np.abs(dspace*dtime))
    
# #     return(azmovie, freqspace1d, freqtime)

    
# def average_3d_ps(movies, chunkshape, fps, ppd):

#     """ 
#     Calculate the mean power spectrum for a list of many movies
    
#     Parameters
#     ----------
#     movies:      list of 3d numpy arrays defining movies for 3d fourier transform analysis.
#     chunklen:   integer number of frames defining the length of movie 'chunks'
#     fps:        frame rate of movie
#     ppd:        pixels per degree of movie
    
#     Returns:
#     --------
#     psmean:     mean spatiotemporal fourier transform
#     fq1d:       spatial frequency spectrum (dimensions of 0 axis of psmean)
#     fqt:        temporal frequency spectrum (dimensions of 1 axis of psmean)
    
#     """
    
#     #keep a mean
#     nmovies = len(movies)
#     pssum = 0
    
#     #ftspecs = []
#     for movie in movies:
        
#         ps, fq1d, fqt = st_ps(movie, chunkshape, fps, ppd)
#         #print(ft.shape, fq1d.shape, fqt.shape)
#         #ftspecs.append(ft)
#         pssum = ftsum + ps
    
#     psmean = pssum / nmovies
#     #print(len(ftspecs), ftspecs[1].shape)
#     #ftmean = np.mean(np.array(ftspecs),axis=0)
#     #print(ftmean.shape)
#     return(psmean, fq1d, fqt)
    

    
# def calc_velocity_spec(spectrum, fqspace, fqtime, nbins=20):
#     """
#     A spatiotemporal power spectrum has a corresponding velocity spectrum.
#     This is calculated by dividing the temporal frequencies by the spatial frequencies.
#     This function converts a joint spatiotemporal amplitude spectrum into a 1D velocity spectrum.
    
#     Parameters:
#     spectrum: 2d numpy array of spatio/temporal amlpitude spectrum
#     fqspace: 1d numpy array defining spatial frequencies of spectrum
#     fqtime: 1d numpy array defining temporal frequencies of spectrum
#     nbins: integer number of bins in which to group velocity values
    
#     Returns:
#     bins: 1d numpy array defining bins
#     v_spectrum: 1d numpy array of velocity amplitdue spectrum (mean logvelocity amplitude in bin)
    
#     """
    
#     #remove dc
#     fqtime = fqtime[1:]
#     fqspace = fqspace[1:]
#     spectrum = spectrum[1:,1:]
    
#     xx, yy = np.meshgrid(fqtime, fqspace) #remove dc
#     v = np.log10(yy/xx) #bin velocities in log space
    
#     counts, bins = np.histogram(v.flatten(), bins=nbins)
    
#     spectrum_flat = spectrum.flatten() #remove DC

#     mask = np.array([np.digitize(v.flatten(), bins) == i for i in range(nbins+1)])
    
#     v_spectrum = [np.mean(spectrum_flat[i]) for i in mask]
    
#     #move back to true velocity bin values (undo our log)
#     bins = np.exp(bins)
    
#     return(bins, v_spectrum)
