import numpy as np
import h5py

def group_to_dict(group):
    "deep copy hdf5 structure to dictionary"
    out = {}
    for key in group.keys():
        print key
        if hasattr(group[key],"value"):
            out[key] = group[key].value
        elif hasattr(group[key],"shape"):
            out[key] = np.asarray(group[key])
        else:
            out[key] = group_to_dict(group[key])
    return out