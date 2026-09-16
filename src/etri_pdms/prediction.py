"""Load trusted VAD/py123d plan_results without the mmcv runtime."""
import io, pickle
import numpy as np

class ConfigDict(dict):
    def __getattr__(self,key):
        try: return self[key]
        except KeyError: raise AttributeError(key)

class CPUUnpickler(pickle.Unpickler):
    def find_class(self,module,name):
        if (module,name)==('mmcv.utils.config','ConfigDict'): return ConfigDict
        if (module,name)==('torch.storage','_load_from_bytes'):
            import torch
            return lambda data: torch.load(io.BytesIO(data),map_location='cpu',weights_only=False)
        return super().find_class(module,name)

def load_pickle(path):
    # Pickle executes code: only load your own trusted outputs / dataset infos.
    with open(path,'rb') as f: return CPUUnpickler(f).load()

def array(x):
    if hasattr(x,'detach'): x=x.detach().cpu().numpy()
    return np.asarray(x,dtype=float)

def load_plans(path):
    obj=load_pickle(path)
    if not isinstance(obj,dict) or not isinstance(obj.get('plan_results'),dict):
        raise ValueError('Expected dict with plan_results[token] = [prediction, command]')
    return obj['plan_results']

def select_prediction(entry,cfg):
    if not isinstance(entry,(list,tuple)) or len(entry)!=2: raise ValueError('entry must be [prediction, command]')
    pred,command=array(entry[0]),array(entry[1]).reshape(-1)
    if pred.shape==(6,2): mode=0; selected=pred
    elif pred.shape==(3,6,2):
        if command.shape!=(3,) or not np.isfinite(command).all() or not np.allclose(np.sort(command),[0,0,1]):
            raise ValueError('command must be a one-hot vector of length 3')
        mode=int(command.argmax()); selected=pred[mode]
    else: raise ValueError(f'Unsupported planning shape {pred.shape}; expected (3,6,2) or (6,2)')
    if not np.isfinite(selected).all(): raise ValueError('nonfinite prediction')
    if cfg.representation=='step_offsets': selected=selected.cumsum(axis=0)
    if cfg.axes=='x_right_y_forward': selected=np.column_stack([selected[:,1],-selected[:,0]])
    return selected,mode
