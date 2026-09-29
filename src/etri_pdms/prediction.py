"""Load trusted VAD/py123d plan_results without the mmcv runtime."""
import io, pickle
import numpy as np

class ConfigDict(dict):
    def __getattr__(self,key):
        try: return self[key]
        except KeyError: raise AttributeError(key)

class UnusedLiDARBoxes:
    """Allow reading prediction pickles without importing MMDetection3D.

    PDMS discards predicted 3D boxes and never calls methods on this object.
    """

class CPUUnpickler(pickle.Unpickler):
    def find_class(self,module,name):
        if (module,name)==('mmcv.utils.config','ConfigDict'): return ConfigDict
        if (module,name)==('torch.storage','_load_from_bytes'):
            import torch
            return lambda data: torch.load(io.BytesIO(data),map_location='cpu',weights_only=False)
        if (module,name)==('mmdet3d.core.bbox.structures.lidar_box3d','LiDARInstance3DBoxes'):
            return UnusedLiDARBoxes
        return super().find_class(module,name)

def load_pickle(path):
    # Pickle executes code: only load your own trusted outputs / dataset infos.
    with open(path,'rb') as f: return CPUUnpickler(f).load()

def array(x):
    if hasattr(x,'detach'): x=x.detach().cpu().numpy()
    return np.asarray(x,dtype=float)

def load_plans(path, return_report=False):
    from .inputs import load_collection
    plans, report = load_collection(path, 'planning')
    return (plans, report) if return_report else plans


def select_prediction(entry,cfg,return_metadata=False):
    if isinstance(entry,dict):
        pred=array(entry['ego_fut_preds'])
        scores=array(entry['plan_cls_preds']).reshape(-1)
        command=array(entry['ego_fut_cmd']).reshape(-1)
        if pred.shape!=(18,6,2) or scores.shape!=(18,):
            raise ValueError(f'Unsupported unified planning shape {pred.shape}, scores {scores.shape}; expected (18,6,2) and (18,)')
        if command.shape!=(6,) or not np.isfinite(command).all() or not np.allclose(np.sort(command),[0,0,0,0,0,1]):
            raise ValueError('ego_fut_cmd must be a one-hot vector of length 6')
        if not np.isfinite(scores).all(): raise ValueError('nonfinite plan_cls_preds')
        mode=int(command.argmax())
        start=mode*3  # py123d command-major layout: three candidates per command.
        selected_index=start+int(scores[start:start+3].argmax())
        selected=pred[selected_index]
        metadata={'planning_format':'unified','selection_policy':'command_group_confidence_argmax',
                  'candidate_layout':'command_major','selected_candidate_index':selected_index,
                  'selected_confidence':float(scores[selected_index])}
    else:
        if not isinstance(entry,(list,tuple)) or len(entry)!=2: raise ValueError('entry must be [prediction, command]')
        pred,command=array(entry[0]),array(entry[1]).reshape(-1)
        if pred.shape==(6,2): mode=0; selected=pred
        elif pred.shape==(3,6,2):
            if command.shape!=(3,) or not np.isfinite(command).all() or not np.allclose(np.sort(command),[0,0,1]):
                raise ValueError('command must be a one-hot vector of length 3')
            mode=int(command.argmax()); selected=pred[mode]
        else: raise ValueError(f'Unsupported planning shape {pred.shape}; expected (3,6,2) or (6,2)')
        metadata={'planning_format':'legacy','selection_policy':'already_selected' if pred.shape==(6,2) else 'command_index',
                  'selected_candidate_index':mode,'selected_confidence':None}
    if not np.isfinite(selected).all(): raise ValueError('nonfinite prediction')
    if cfg.representation=='step_offsets': selected=selected.cumsum(axis=0)
    if cfg.axes=='x_right_y_forward': selected=np.column_stack([selected[:,1],-selected[:,0]])
    if return_metadata: return selected,mode,metadata
    return selected,mode
