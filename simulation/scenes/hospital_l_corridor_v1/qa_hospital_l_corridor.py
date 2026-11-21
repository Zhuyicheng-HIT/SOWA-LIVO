#!/usr/bin/env python3
"""Verify authored L geometry, actual RGB inspection captures and albedo-only control."""
import argparse,copy,hashlib,json,pathlib,xml.etree.ElementTree as ET
import numpy as np
from PIL import Image

def main(a):
    scene=pathlib.Path(a.scene);rich=pathlib.Path(a.rich_preview);low=pathlib.Path(a.low_preview)
    manifests=[json.loads((p/'preview_manifest.json').read_text()) for p in [rich,low]]
    assert all(m['status']=='preview_complete' and len(m['views'])==6 for m in manifests)
    roots=[]
    for name,m in zip(['rich','low'],manifests):
        f=scene/(name+'.sdf');assert hashlib.sha256(f.read_bytes()).hexdigest()==m['source_world_sha256']
        root=ET.parse(f).getroot()
        for v in root.findall('.//visual'):
            for mat in list(v.findall('material')):v.remove(mat)
        roots.append(ET.tostring(root))
    geometry_identical=roots[0]==roots[1];assert geometry_identical
    pairs=[]
    for rv,lv in zip(manifests[0]['views'],manifests[1]['views']):
        assert rv['world_pose_xyz_yaw']==lv['world_pose_xyz_yaw']
        label=rv['label'];rr=np.load(rich/(label+'_ranges.npz'))['ranges'];lr=np.load(low/(label+'_ranges.npz'))['ranges']
        equal=bool(np.array_equal(rr,lr,equal_nan=True));assert equal
        ri=np.asarray(Image.open(rich/rv['file']));li=np.asarray(Image.open(low/lv['file']))
        assert ri.shape==li.shape==(720,960,3)
        assert not np.array_equal(ri,li)
        pairs.append({'view':label,'range_samples':int(rr.size),'ranges_bit_identical':equal,
                      'rich_corners':rv['shi_tomasi_corners'],'low_corners':lv['shi_tomasi_corners'],
                      'rgb_mean_absolute_difference_dn':float(np.abs(ri.astype(float)-li).mean())})
    result={'status':'passed','geometry_identical_after_material_removal':geometry_identical,
            'actual_rgb_preview_views_per_condition':6,'actual_range_samples_compared':sum(p['range_samples'] for p in pairs),
            'paired_ranges_all_identical':all(p['ranges_bit_identical'] for p in pairs),'pairs':pairs,
            'scope':'Scene geometry and fixed-pose RGB/flash-LiDAR inspection; no trajectory accuracy evaluation in this QA'}
    (scene/'preview_qa.json').write_text(json.dumps(result,indent=2)+'\n')
    m=json.loads((scene/'scene_manifest.json').read_text());m['status']='actual_rgb_preview_and_albedo_control_verified'
    m['preview_qa']='preview_qa.json';m['rich_preview']=str(rich);m['low_preview']=str(low)
    m['asset_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((scene/'assets').glob('*.png'))}
    (scene/'scene_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--rich-preview',required=True);p.add_argument('--low-preview',required=True);main(p.parse_args())
