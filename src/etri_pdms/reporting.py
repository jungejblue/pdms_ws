"""One JSON object per scenario, with coverage and invalid samples preserved."""
from collections import defaultdict,Counter
from pathlib import Path
import json

METRICS=('NC','DAC','EP','TTC','C','PDMS')

def scenario_records(rows):
    grouped=defaultdict(list)
    for row in rows:grouped[row['scenario']].append(row)
    records=[]
    for name,items in sorted(grouped.items()):
        valid=[r for r in items if r['valid']]
        total=len(items);n=len(valid);complete=n==total
        means={k:sum(r[k] for r in valid)/n if n else None for k in METRICS}
        record={'scenario_id':name,'status':'complete' if complete else ('partial' if n else 'invalid'),
                'num_samples':total,'num_valid':n,'num_invalid':total-n,'coverage':n/total,
                **{k:(means[k] if complete else None) for k in METRICS},
                'valid_sample_mean':means,'reference_failure_count':sum(bool(r.get('reference_failure')) for r in valid),
                'map_quality_counts':dict(Counter(r.get('map_quality','unknown') for r in valid)),
                'invalid_reasons':dict(Counter(r.get('invalid_reason','unknown') for r in items if not r['valid']))}
        records.append(record)
    return records

def write_scenario_reports(out,rows):
    out=Path(out);records=scenario_records(rows)
    lines=[json.dumps(r,ensure_ascii=False,allow_nan=False,separators=(',',':')) for r in records]
    # Valid JSON array, with one physical line per scenario for convenient diffing.
    (out/'scenario_scores.json').write_text('[\n'+',\n'.join(lines)+'\n]\n',encoding='utf-8')
    (out/'scenario_scores.jsonl').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    import pandas as pd
    pd.DataFrame([{k:v for k,v in r.items() if not isinstance(v,dict)} for r in records]).to_csv(out/'scenario_scores.csv',index=False)
    return records
