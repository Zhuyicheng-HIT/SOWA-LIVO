#!/usr/bin/env python3
"""Hospital-style L corridor; flat texture richness is separated from geometry."""
import argparse, hashlib, json, math, pathlib, shutil, xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def sub(p, tag, text=None, **attrs):
    q=ET.SubElement(p,tag,attrs)
    if text is not None:q.text=str(text)
    return q

def material(v, color='1 1 1 1', texture=None, emissive=None):
    m=sub(v,'material');sub(m,'ambient',color);sub(m,'diffuse',color)
    sub(m,'specular','0.08 0.08 0.08 1')
    if emissive:sub(m,'emissive',emissive)
    if texture:
        texture_name=pathlib.Path(texture).name
        uri=f'model://hospital_l_corridor_v1/assets/{texture_name}'
        p=sub(sub(m,'pbr'),'metal');sub(p,'albedo_map',uri)
        sub(p,'metalness',0);sub(p,'roughness',.78)

def box(link,name,p,size,color,yaw=0,collision=True):
    pose=' '.join(map(str,[*p,0,0,yaw]))
    if collision:
        c=sub(link,'collision',name=name+'_collision');sub(c,'pose',pose)
        sub(sub(sub(c,'geometry'),'box'),'size',' '.join(map(str,size)))
    v=sub(link,'visual',name=name);sub(v,'pose',pose)
    sub(sub(sub(v,'geometry'),'box'),'size',' '.join(map(str,size)));material(v,color)
    return v

def plane(link,name,p,size,normal,texture,yaw=0,emissive=None):
    v=sub(link,'visual',name=name);sub(v,'pose',' '.join(map(str,[*p,0,0,yaw])))
    g=sub(sub(v,'geometry'),'plane');sub(g,'normal',normal);sub(g,'size',' '.join(map(str,size)))
    material(v,texture=str(texture),emissive=emissive)
    return v

def tiled(patch,size):
    out=Image.new('RGB',size)
    for y in range(0,size[1],patch.height):
        for x in range(0,size[0],patch.width):out.paste(patch,(x,y))
    return out

def load_font(size,bold=False):
    names=['DejaVuSans-Bold.ttf','arialbd.ttf'] if bold else ['DejaVuSans.ttf','arial.ttf']
    roots=[pathlib.Path('/usr/share/fonts/truetype/dejavu'),pathlib.Path('/usr/share/fonts/truetype/liberation2'),
           pathlib.Path('/mnt/c/Windows/Fonts'),pathlib.Path('C:/Windows/Fonts')]
    for root in roots:
        for name in names:
            candidate=root/name
            if candidate.exists():return ImageFont.truetype(str(candidate),size)
    return ImageFont.load_default()

def main(a):
    here=pathlib.Path(__file__).resolve().parent
    out=pathlib.Path(a.output).resolve();assets=out/'assets';assets.mkdir(parents=True,exist_ok=True)
    if out.name!='hospital_l_corridor_v1':
        raise SystemExit('Output directory must be named hospital_l_corridor_v1 so model:// texture URIs resolve.')
    hospital=here;poster_root=here/'assets'/'poster_source'
    template=pathlib.Path(a.template).resolve() if a.template else (here.parents[1]/'templates'/'hospital_world_template.sdf').resolve()
    ppm=180;h=3.3;w=5.2;rng=np.random.default_rng(1029)
    font=load_font(25);small=load_font(18);bold=load_font(34,bold=True)
    files={
      'floor':hospital/'assets/source_floor.png',
      'ceiling':hospital/'assets/source_ceiling.png',
      'wood':hospital/'assets/source_wood.png',
      'wall':hospital/'assets/source_wall.png'}
    # Crop inside material islands, excluding black unused UV atlas space.
    source_floor=Image.open(files['floor']).convert('RGB')
    arr=np.asarray(source_floor).astype(float).mean(2);candidates=[]
    for yy in range(4,1400,32):
        for xx in range(4,1950,32):
            patch=arr[yy:yy+64,xx:xx+64]
            if 70<patch.mean()<210:candidates.append((float(patch.std()),xx,yy))
    _,fx,fy=min(candidates);floor_crop=[fx,fy,fx+64,fy+64]
    floor_patch=source_floor.crop(floor_crop).resize((216,216))
    # Ceiling grain comes from the atlas; the grid below has an explicit 0.6 m pitch.
    ceiling_patch=Image.open(files['ceiling']).convert('RGB').crop((785,210,805,230)).resize((108,108))
    wood_patch=Image.open(files['wood']).convert('RGB').crop((8,15,55,180))
    floor_patch.save(assets/'hospital_floor_patch.png');ceiling_patch.save(assets/'hospital_ceiling_patch.png')
    poster_names=['user_provided_notice.png','cdc_clean_hands.png','cdc_superpower.png',
                  'authored_directory_0.png','authored_directory_1.png','authored_directory_2.png','authored_directory_3.png']
    posters=[Image.open(poster_root/n).convert('RGB') for n in poster_names]
    # A wall's local y follows its tangent; local x is the inward normal.
    walls=[
      {'id':'main_west','a':[-7.6,-35.],'b':[-7.6,107.6],'n':[1.,0.]},
      {'id':'main_east','a':[-2.4,102.4],'b':[-2.4,-35.],'n':[-1.,0.]},
      {'id':'return_north','a':[-7.6,107.6],'b':[85.,107.6],'n':[0.,-1.]},
      {'id':'return_south','a':[85.,102.4],'b':[-2.4,102.4],'n':[0.,1.]},
      {'id':'start_end','a':[-2.4,-35.],'b':[-7.6,-35.],'n':[0.,1.]},
      {'id':'return_end','a':[85.,107.6],'b':[85.,102.4],'n':[-1.,0.]}]
    doors=[];notices=[]
    for wi,wall in enumerate(walls):
        aa=np.array(wall['a']);bb=np.array(wall['b']);wall['length']=float(np.linalg.norm(bb-aa));wall['t']=((bb-aa)/wall['length']).tolist()
        wall['doors']=[];wall['notices']=[]
        if wall['length']<20:continue
        for j,s in enumerate(np.arange(7.+(wi%2)*3.,wall['length']-5.,12.)):
            item={'wall':wall['id'],'s':float(s),'number':f'{wi+1}{j+1:02d}'};wall['doors'].append(item);doors.append(item)
        for j,s in enumerate(np.arange(3.,wall['length']-1.,3.8)):
            if min([abs(s-d['s']) for d in wall['doors']]+[100])<1.35:continue
            item={'wall':wall['id'],'s':float(s),'poster':(wi*3+j)%len(posters)}
            wall['notices'].append(item);notices.append(item)
    for wall in walls:
        length=wall['length'];count=math.ceil(length/10);seg=length/count;wall['segments']=count
        for j in range(count):
            s0=j*seg;s1=(j+1)*seg;ww=round(seg*ppm);hh=round(h*ppm)
            for cond in ['rich','low']:
                im=Image.new('RGB',(ww,hh),(231,231,221));d=ImageDraw.Draw(im)
                # Hospital cream walls, continuous blue/grey dado and coved skirting.
                d.rectangle((0,int((h-1.03)*ppm),ww,hh),fill=(187,202,201))
                d.rectangle((0,int((h-1.10)*ppm),ww,int((h-1.035)*ppm)),fill=(143,168,168))
                d.rectangle((0,int((h-.12)*ppm),ww,hh),fill=(108,126,130))
                if cond=='rich':
                    for p in wall['notices']:
                        cx=(p['s']-s0)*ppm
                        if -.6*ppm<cx<ww+.6*ppm:
                            pic=posters[p['poster']];pw=.72 if p['poster']<3 else .92;ph=1.02 if p['poster']<3 else .68
                            pic=ImageOps.fit(pic,(round(pw*ppm),round(ph*ppm)))
                            xx=round(cx-pic.width/2);yy=round((h-1.58-ph/2)*ppm)
                            d.rectangle((xx-4,yy-4,xx+pic.width+4,yy+pic.height+4),fill=(245,245,240))
                            im.paste(pic,(xx,yy))
                    # Sparse horizontal wayfinding graphics, physically flush with wall.
                    if j%3==1:
                        d.rounded_rectangle((20,53,min(ww-20,470),116),radius=8,fill=(37,103,124))
                        d.text((34,65),'OUTPATIENT  /  WARD  >',font=font,fill='white')
                im.transpose(Image.ROTATE_90).save(assets/f'{wall["id"]}_{j}_{cond}.png')
    # Closed timber doors use the original hospital timber texture, not open apertures.
    for d in doors:
        im=wood_patch.resize((198,405));draw=ImageDraw.Draw(im)
        draw.rectangle((22,54,176,93),fill=(42,111,133));draw.text((45,57),d['number'],font=bold,fill='white')
        draw.rectangle((158,188,168,227),fill=(180,185,184));draw.line((138,202,164,202),fill=(222,225,224),width=5)
        draw.rectangle((15,360,183,396),fill=(154,159,158));im.transpose(Image.ROTATE_90).save(assets/f'door_{d["number"]}.png')
    # Non-overlapping floor/ceiling rectangles cover the L union, with no hidden cross-wall.
    regions=[{'id':'main','center':[-5.,33.7],'size':[w,137.4]},
             {'id':'return','center':[38.7,105.],'size':[92.6,w]}]
    surfaces=[]
    for reg in regions:
        long_axis=int(reg['size'][0]>reg['size'][1]);length=reg['size'][1-long_axis];n=math.ceil(length/10);seg=length/n
        for j in range(n):
            size=list(reg['size']);size[1-long_axis]=seg
            center=list(reg['center']);center[1-long_axis]+= -length/2+(j+.5)*seg
            ident=f'{reg["id"]}_{j}';surfaces.append({'id':ident,'size':size,'center':center})
            iw,ih=[round(s*ppm) for s in size]
            floor=tiled(floor_patch,(iw,ih));dr=ImageDraw.Draw(floor)
            band=round(.14*ppm);margin=round(.25*ppm)
            if long_axis==0:
                for x in [margin,iw-margin-band]:dr.rectangle((x,0,x+band,ih),fill=(181,199,192))
            else:
                for y in [margin,ih-margin-band]:dr.rectangle((0,y,iw,y+band),fill=(181,199,192))
            # Delicate blue terrazzo and calm edge bands; no giant checkerboard.
            floor.save(assets/f'floor_{ident}_rich.png')
            Image.new('RGB',(iw,ih),(164,177,182)).save(assets/f'floor_{ident}_low.png')
            ceil=tiled(ceiling_patch,(iw,ih));cd=ImageDraw.Draw(ceil);grid=round(.6*ppm)
            for x in range(0,iw,grid):cd.line((x,0,x,ih),fill=(179,180,175),width=2)
            for y in range(0,ih,grid):cd.line((0,y,iw,y),fill=(179,180,175),width=2)
            ceil.save(assets/f'ceiling_{ident}_rich.png')
            Image.new('RGB',(iw,ih),(229,229,219)).save(assets/f'ceiling_{ident}_low.png')
    worlds={}
    for cond in ['rich','low']:
        tree=ET.parse(template);world=tree.getroot().find('world');world.set('name','hospital_l_corridor_v1')
        for q in list(world.findall('model'))+list(world.findall('include'))+list(world.findall('light')):world.remove(q)
        scene=world.find('scene');scene.find('ambient').text='.82 .82 .80 1';scene.find('shadows').text='false'
        light=sub(world,'light',name='hospital_fill',type='directional');sub(light,'pose','0 0 10 0 0 0')
        sub(light,'direction','.12 .18 -1');sub(light,'diffuse','.38 .38 .36 1');sub(light,'specular','.03 .03 .03 1');sub(light,'cast_shadows','false')
        model=sub(world,'model',name='hospital_l_shell');sub(model,'static','true');link=sub(model,'link',name='shell')
        for wall in walls:
            aa=np.array(wall['a']);tt=np.array(wall['t']);nn=np.array(wall['n']);center=(aa+np.array(wall['b']))/2
            yaw=math.atan2(nn[1],nn[0]);outside=center-.1*nn
            box(link,wall['id']+'_structure',[*outside,h/2],[.2,wall['length'],h],'.88 .89 .86 1',yaw)
            for j in range(wall['segments']):
                seg=wall['length']/wall['segments'];p=aa+tt*(j+.5)*seg+.0005*nn
                plane(link,f'{wall["id"]}_paint_{j}',[*p,h/2],[h,seg],'1 0 0',assets/f'{wall["id"]}_{j}_{cond}.png',yaw)
            # Continuous slim rails preserve the original hospital visual language.
            cuts=[0.]
            for d in wall['doors']:cuts.extend([d['s']-.60,d['s']+.60])
            cuts.append(wall['length'])
            for ri,(lo,hi) in enumerate(zip(cuts[::2],cuts[1::2])):
                for z,depth,thick,col in [(.10,.004,.10,'.42 .48 .49 1'),(.88,.012,.045,'.56 .66 .65 1')]:
                    p=aa+tt*(lo+hi)/2+nn*(depth/2+.001)
                    box(link,f'{wall["id"]}_rail_{ri}_{z}',[*p,z],[depth,hi-lo,thick],col,yaw)
            for d in wall['doors']:
                p=aa+tt*d['s'];doorid='door_'+d['number']
                box(link,doorid,[*(p+nn*.0015),1.125],[.003,1.1,2.25],'.66 .55 .39 1',yaw)
                plane(link,doorid+'_wood',[*(p+nn*.0032),1.125],[2.25,1.1],'1 0 0',assets/(doorid+'.png'),yaw)
                for side in [-1,1]:
                    pp=p+tt*side*.565+nn*.004
                    box(link,doorid+f'_jamb_{side}',[*pp,1.145],[.008,.035,2.29],'.58 .60 .58 1',yaw)
                box(link,doorid+'_lintel',[*(p+nn*.004),2.29],[.008,1.165,.035],'.58 .60 .58 1',yaw)
        for s in surfaces:
            cx,cy=s['center'];sx,sy=s['size'];ident=s['id']
            box(link,'slab_'+ident,[cx,cy,-.10],[sx,sy,.2],'.57 .65 .67 1')
            plane(link,'floor_'+ident,[cx,cy,.0003],[sx,sy],'0 0 1',assets/f'floor_{ident}_{cond}.png')
            box(link,'ceiling_slab_'+ident,[cx,cy,h+.10],[sx,sy,.2],'.9 .9 .88 1')
            plane(link,'ceiling_'+ident,[cx,cy,h-.0003],[sx,sy],'0 0 -1',assets/f'ceiling_{ident}_{cond}.png',emissive='.10 .10 .09 1')
        # Flat ceiling light panels and actual fill lights, no hanging obstacles.
        lamps=[(-5.,y,math.pi/2) for y in np.arange(-30.,103.,6.)]+[(x,105.,0) for x in np.arange(0.,84.,6.)]
        for i,(x,y,yaw) in enumerate(lamps):
            v=box(link,f'light_panel_{i}',[x,y,h-.001],[1.2,.30,.002],'.96 .97 .96 1',yaw,collision=False)
            sub(v.find('material'),'emissive','.7 .72 .70 1')
            light=sub(world,'light',name=f'ceiling_fill_{i}',type='point');sub(light,'pose',f'{x} {y} {h-.10} 0 0 0')
            sub(light,'diffuse','.32 .33 .32 1');sub(light,'specular','.03 .03 .03 1');sub(light,'cast_shadows','false')
            att=sub(light,'attenuation');sub(att,'range',12);sub(att,'constant',1);sub(att,'linear',.15);sub(att,'quadratic',.04)
        target=out/(cond+'.sdf');ET.indent(tree,space='  ');tree.write(target,encoding='utf-8',xml_declaration=True);worlds[cond]=sha(target)
    for name in ['AWS_HOSPITAL_LICENSE','AWS_HOSPITAL_NOTICE']:
        source=here/name
        if source.exists() and out!=here:shutil.copy2(source,out/name)
    if out!=here:shutil.copy2(here/'model.config',out/'model.config')
    meta={'scene':'hospital_l_corridor_v1','status':'authored_pending_actual_rgb_preview',
      'geometry':{'main_centerline_m':140,'return_centerline_m':90,'total_centerline_m':230,'inside_width_m':w,'height_m':h,
                  'window_openings':0,'open_doorways':0,'closed_doors':len(doors),'door_panel_relief_m':.003,'door_frame_relief_m':.008,
                  'flat_notices':len(notices),'floor_wall_ceiling_relief':'continuous planar shell; texture grids are flush'},
      'source_assets':{k:{'path':f'assets/source_{k}.png','sha256':sha(p)} for k,p in files.items()},
      'source_crops_px':{'floor':floor_crop,'ceiling':[785,210,805,230],'wood':[8,15,55,180]},
      'poster_sources':'assets/poster_source/ (source acknowledgements in README.md)',
      'world_sha256':worlds,'doors':doors,'notices':notices,'walls':walls,
      'rich_low_control':'identical physical geometry, lighting, door appearance and calibration; only wall notice, floor and ceiling albedo maps differ',
      'evaluation_segments':'long straight before corner; turn; straight after corner evaluated separately',
      'axis_prior_online':False,'algorithm_status':'existing long-corridor metrics are summarized at ../../../results/long_l_baselines.csv'}
    (out/'scene_manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps({k:v for k,v in meta.items() if k not in ['doors','notices','walls','source_assets']},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',default='hospital_l_corridor_v1')
    p.add_argument('--template');main(p.parse_args())
