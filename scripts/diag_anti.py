"""Luat ANTI: vat dong that co dau discrepancy DAO CHIEU giua map qua khu va map tuong lai
(vi no di chuyen don dieu theo thoi gian). Nhieu/parallax thi khong co tinh chat do."""
import numpy as np, diag_sign as d

def run(scan_idx, maps):
    poses=d.load_poses(d.POSES_PATH); Tr=d.load_tr(d.SEQ_DIR+'/calib.txt')
    sx=d.load_xyz(scan_idx); n=len(sx); sp=poses[scan_idx]@Tr; sg=d.ransac_ground(sx,seed=0)
    lab=d.load_labels(scan_idx); gt=np.isin(lab,list(d.MOVING_CLASSES))
    nlv=len(d.LEVELS)
    diffs=[np.full((len(maps),n),np.nan) for _ in range(nlv)]
    obs=[np.zeros(n,np.int32) for _ in range(nlv)]
    for mi_,m in enumerate(maps):
        mx=d.load_xyz(m); T=np.linalg.inv(poses[m]@Tr)@sp; sim=d.transform(sx,T); mg=d.ransac_ground(mx,seed=0)
        for li,(h,w) in enumerate(d.LEVELS):
            si,row,col,val=d.build_range_image(sim,h,w,sg); mimg,_,_,_=d.build_range_image(mx,h,w,mg)
            px=row*w+col; s=np.where(val,si[px],np.inf); mp=np.where(val,mimg[px],np.inf)
            both=np.isfinite(s)&np.isfinite(mp)
            with np.errstate(invalid="ignore"):  # inf - inf o pixel trong -> NaN, da loc bang `both`
                diffs[li][mi_][both]=(mp-s)[both]
            obs[li]+=val&np.isfinite(mp)
    past=np.array([m<scan_idx for m in maps]); fut=~past
    def dyn_abs(li):
        v=(np.abs(diffs[li])>d.THRESHOLD).sum(axis=0)
        return (obs[li]>0)&(v/np.maximum(obs[li],1)>d.VOTE_THRESHOLD)
    def dyn_anti(li):
        D=diffs[li]
        pos_p=np.nansum(np.where(past[:,None],(D>d.THRESHOLD),0),axis=0)
        neg_p=np.nansum(np.where(past[:,None],(D<-d.THRESHOLD),0),axis=0)
        pos_f=np.nansum(np.where(fut[:,None],(D>d.THRESHOLD),0),axis=0)
        neg_f=np.nansum(np.where(fut[:,None],(D<-d.THRESHOLD),0),axis=0)
        # dao chieu: qua khu nghieng mot dau, tuong lai nghieng dau con lai
        return ((pos_p>neg_p)&(neg_f>pos_f))|((neg_p>pos_p)&(pos_f>neg_f))
    def stack(l0,rev):
        keep=~sg&l0(0)
        for li in range(1,nlv): keep&=rev(li)
        return keep
    out={}
    out['ABS (mo phong code hien tai)']=stack(dyn_abs,dyn_abs)
    out['ANTI @L0 + ABS revert']=stack(dyn_anti,dyn_abs)
    out['ANTI moi thang']=stack(dyn_anti,dyn_anti)
    out['ABS @L0 + ANTI revert']=stack(dyn_abs,dyn_anti)
    return out,gt

if __name__=='__main__':
    import sys
    tot={}
    cases=[(50,[48,49,51,52]),(100,[98,99,101,102]),(150,[148,149,151,152]),
           (200,[198,199,201,202]),(250,[248,249,251,252])]
    for scan,maps in cases:
        out,gt=run(scan,maps)
        print(f'--- scan {scan} ---')
        for k,pred in out.items():
            p,r,f1,tp,fp,fn=d.prf(pred,gt)
            print(f'  {k:30s} P={p:.3f} R={r:.3f} F1={f1:.3f}  TP={tp:5d} FP={fp:6d}')
            tot.setdefault(k,[]).append(f1)
    print('\n=== F1 trung binh 5 scan (mo phong, cung ground mask cho moi luat) ===')
    for k,v in tot.items(): print(f'  {k:30s} {np.mean(v):.3f}')
