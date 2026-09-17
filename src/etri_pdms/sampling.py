"""Deterministic timestamp sampling, independent of scores."""
from collections import defaultdict
import math
from .data import SCALE


def select_interval(tokens,dataset,interval):
    if len(set(tokens))!=len(tokens):raise ValueError('Duplicate tokens in evaluation selection')
    if interval==0:
        return list(tokens),{'interval_s':0.,'policy':'exact_input_order','selected_count':len(tokens)}
    groups=defaultdict(list);unmatched=[]
    for token in tokens:
        if token not in dataset.infos:
            unmatched.append(token);continue
        if dataset.cfg.dataset=='nuscenes':
            entry=dataset.tables['sample'].get(token)
            if entry is None:unmatched.append(token);continue
            t=float(entry['timestamp'])*1e-6
        else:t=float(dataset.infos[token]['timestamp'])*SCALE[dataset.cfg.info_timestamp_unit]
        if not math.isfinite(t):raise ValueError('nonfinite sample timestamp')
        groups[dataset.scenario_for(token)].append((t,str(token)))
    selected=[];records=[]
    for scene in sorted(groups):
        last=None
        for timestamp,token in sorted(groups[scene]):
            if last is None or timestamp-last>=interval-1e-6:
                selected.append(token);last=timestamp
                records.append({'token':token,'scene':scene,'timestamp_s':timestamp})
    selected.extend(sorted(unmatched))
    return selected,{'interval_s':interval,'policy':'first_common_then_minimum_elapsed',
                     'input_count':len(tokens),'selected_count':len(selected),
                     'interval_excluded_count':len(tokens)-len(selected),
                     'unmatched_tokens':sorted(unmatched),'selected_samples':records}
