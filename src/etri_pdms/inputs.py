"""Deterministic, nonrecursive discovery of trusted dataset/prediction pickles."""
from pathlib import Path
import hashlib
import numpy as np


def same(a, b):
    if hasattr(a, 'detach'): a = a.detach().cpu().numpy()
    if hasattr(b, 'detach'): b = b.detach().cpu().numpy()
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        try: return bool(np.array_equal(a, b, equal_nan=True))
        except TypeError: return bool(np.array_equal(a, b))
    return a == b


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()


def load_collection(source, kind):
    from .prediction import load_pickle
    path = Path(source).expanduser().resolve()
    directory = path.is_dir()
    files = sorted(p for p in path.iterdir() if p.is_file() and p.suffix.lower() in ('.pkl', '.pickle')) if directory else [path]
    merged = {}; origins = {}
    report = {'source': str(path), 'kind': kind, 'recursive': False,
              'selected_files': [], 'skipped_files': [], 'duplicate_tokens': 0}
    for file in files:
        try: content = load_pickle(file)
        except Exception as exc:
            raise ValueError(f'Cannot read PKL {file}: {type(exc).__name__}: {exc}') from exc
        if kind == 'planning':
            recognized = isinstance(content, dict) and 'plan_results' in content
            entries = content.get('plan_results') if recognized else None
            if recognized and not isinstance(entries, dict):
                raise ValueError(f'{file}: plan_results must be a dictionary')
            items = entries.items() if recognized else ()
        else:
            recognized = isinstance(content, dict) and 'infos' in content
            entries = content.get('infos') if recognized else content
            if not recognized:
                recognized = isinstance(entries, (list, tuple)) and bool(entries) and all(isinstance(i, dict) and 'token' in i and 'timestamp' in i for i in entries)
            if recognized and not isinstance(entries, (list, tuple)):
                raise ValueError(f'{file}: infos must be a list')
            items = []
            if recognized:
                for entry in entries:
                    if not isinstance(entry, dict) or 'token' not in entry or 'timestamp' not in entry:
                        raise ValueError(f'{file}: each info requires token and timestamp')
                    token = str(entry['token'])
                    # Only these fields are consumed by the evaluator.
                    value = {'token': token, 'scene_token': str(entry.get('scene_token', token.rsplit('_', 1)[0])),
                             'timestamp': float(entry['timestamp'])}
                    # Preserve the coordinate contract and pose instead of silently discarding them.
                    for key in ('conversion_meta','ego2global_rotation','ego2global_translation',
                                'map_ego2global_rotation','map_ego2global_translation'):
                        if key in entry:value[key]=entry[key]
                    if not np.isfinite(value['timestamp']): raise ValueError(f'{file}: nonfinite timestamp')
                    items.append((token, value))
        if not recognized:
            if not directory: raise ValueError(f'{file}: not a {kind} PKL')
            report['skipped_files'].append({'path': str(file), 'reason': f'not {kind} schema'})
            continue
        count = 0
        for token, value in items:
            token = str(token); count += 1
            if token in merged:
                if not same(merged[token], value):
                    raise ValueError(f'Conflicting {kind} token {token!r}: {origins[token]} <-> {file}')
                report['duplicate_tokens'] += 1
                continue
            merged[token] = value; origins[token] = str(file)
        report['selected_files'].append({'path': str(file), 'sha256': digest(file), 'entries': count})
    if not merged: raise ValueError(f'No usable {kind} tokens in {path}; scanned {len(files)} PKL files')
    report['unique_tokens'] = len(merged)
    report['token_sources'] = origins
    return merged, report


def load_infos(source):
    entries, report = load_collection(source, 'infos')
    return list(entries.values()), report
