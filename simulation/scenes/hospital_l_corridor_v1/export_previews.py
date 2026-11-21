#!/usr/bin/env python3
import argparse,json,pathlib,shutil
from PIL import Image,ImageDraw,ImageFont

p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--preview',required=True);p.add_argument('--output',required=True);a=p.parse_args()
scene=pathlib.Path(a.scene);preview=pathlib.Path(a.preview);out=pathlib.Path(a.output);out.mkdir(parents=True,exist_ok=True)
m=json.loads((preview/'preview_manifest.json').read_text())
for v in m['views']:shutil.copy2(preview/v['file'],out/v['file'])
for n in ['README.md','preview_qa.json','scene_manifest.json']:shutil.copy2(scene/n,out/n)
shutil.copy2(preview/'preview_manifest.json',out/'preview_manifest.json')
font='/mnt/c/Windows/Fonts/msyh.ttc';title=ImageFont.truetype(font,32);body=ImageFont.truetype(font,23)
im=Image.new('RGB',(1440,1270),'#15262e');d=ImageDraw.Draw(im)
d.text((24,12),'医院风格 L 型长走廊 · 实际 Gazebo RGB',font=title,fill='white')
d.text((24,56),'140 m 主直段 + 90 m 转角后直段  |  净宽 5.2 m  |  36 扇关闭门 · 94 处告示',font=body,fill='#afcbd4')
selected=[(m['views'][0],'主走廊入口'),(m['views'][2],'接近 L 型转角'),(m['views'][3],'转角后走廊'),(m['views'][5],'关闭门、扶手与海报细节')]
for i,(v,label) in enumerate(selected):
    x=(i%2)*720;y=108+(i//2)*580
    im.paste(Image.open(preview/v['file']).resize((720,540)),(x,y));d.text((x+16,y+546),label,font=body,fill='white')
im.save(out/'hospital_l_corridor_preview.png')
print(out/'hospital_l_corridor_preview.png')
