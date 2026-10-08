"""Offline evidence extraction; no serial port or network access."""
import sys, re, json, hashlib
from pathlib import Path
from collections import Counter

def analyze(path):
    raw=path.read_bytes(); lines=raw.decode('utf-8-sig').splitlines()
    result={'file':path.name,'sha256':hashlib.sha256(raw).hexdigest(),'lines':len(lines),
            'undefined_commands':0,'run_commands':{},'pulse_results':{},'drives':{},'evidence':[]}
    run=Counter();done=Counter()
    for number,line in enumerate(lines,1):
        if '>> service pulse' in line and 'undefined' in line:result['undefined_commands']+=1
        m=re.search(r'HE200 (H[12]|V[12]) addr=\d RUN (FWD|REV)',line)
        if m:run[' '.join(m.groups())]+=1
        m=re.search(r'@SERVICE_PULSE state=DONE (.+)',line)
        if m:done[m[1]]+=1
        m=re.search(r'@HE200 name=(H[12]|V[12]) (.+)',line)
        if m:
            v=dict(re.findall(r'(\w+)=(\S+)',m[2]));d=result['drives'].setdefault(m[1],{'samples':0,'max_run001':0,'max_outV':0,'DI':[],'faults':[]})
            d['samples']+=1;d['max_run001']=max(d['max_run001'],int(v['run001']));d['max_outV']=max(d['max_outV'],int(v['outV']))
            for key,source in [('DI','di'),('faults','fault')]:
                if v[source] not in d[key]:d[key].append(v[source])
        if ('@SERVICE_PULSE state=DONE' in line or 'SAFETY LIMIT monitoring DISABLED' in line or 'name=PREFLIGHT result=' in line):
            result['evidence'].append({'line':number,'text':line})
    result['run_commands']=dict(run);result['pulse_results']=dict(done)
    return result
if __name__=='__main__':
    print(json.dumps([analyze(Path(p)) for p in sys.argv[1:]],ensure_ascii=False,indent=2))
