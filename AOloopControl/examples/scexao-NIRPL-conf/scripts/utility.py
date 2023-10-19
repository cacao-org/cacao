# a bunch of python scripts for configuring and setting up stuff.

from pyMilk.interfacing.isio_shmlib import SHM as shm
import matplotlib.pyplot as plt
import numpy as np

def locate_spectra(streamname, stack_num, num_spec = 3, width = 6, plot = True):
    """ Find spectral traces, from Jon's code """

    stream = shm(streamname)
    im = np.mean(stream.multi_recv_data(stack_num,1),axis=0)

    im_column_stack = np.mean(im, axis=1)
    ylocs = np.zeros(num_spec, dtype=int)

    for i in range(num_spec):
        _yloc = np.argmax(im_column_stack)
        ylocs[i] = int(_yloc)
        im_column_stack[_yloc - width: _yloc + width] = 0

    ylocs = np.sort(ylocs)

    if plot:
        plt.imshow(im)
        for i in range(num_spec): plt.axhspan(ylocs[i] - width, ylocs[i] + width, alpha=0.2, color='white')
        plt.show()

    return ylocs

def make_spectral_mask(instreamname,outstreamname,stack_num=300,width=6):
    """ find spectral traces and a write a mask covering the traces to shared memory """
    ylocs = locate_spectra(instreamname,stack_num,width=width,plot=False)
    instream = shm(instreamname)
    _outstreamdata = np.zeros_like(instream.get_data(False,False))
    outstreamdata = []
    for y in ylocs:
        _outcopy = np.copy(_outstreamdata)
        _outcopy[y-width:y+width,:] = 1
        outstreamdata.append(_outcopy)
    outstreamdata = np.transpose(np.array(outstreamdata),(1,2,0))
    shm(outstreamname,outstreamdata)