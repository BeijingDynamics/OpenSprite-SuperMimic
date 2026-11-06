"""Fetch selected original BVH entries with HTTP ranges, not the full 1GB archive."""
from pathlib import Path
import io,json,urllib.request,zipfile,hashlib
URL='https://theorangeduck.com/media/uploads/Geno/lafan1-resolved/bvh.zip'
ROOT=Path(__file__).resolve().parents[1]
class RemoteZip(io.RawIOBase):
    def __init__(self):
        req=urllib.request.Request(URL,method='HEAD')
        with urllib.request.urlopen(req,timeout=60) as r:self.size=int(r.headers['Content-Length'])
        self.pos=0
    def seekable(self):return True
    def tell(self):return self.pos
    def seek(self,offset,whence=0):
        self.pos=offset if whence==0 else self.pos+offset if whence==1 else self.size+offset
        return self.pos
    def read(self,count=-1):
        if count<0:count=self.size-self.pos
        if not count:return b''
        stop=min(self.size,self.pos+count)-1
        req=urllib.request.Request(URL,headers={'Range':f'bytes={self.pos}-{stop}'})
        with urllib.request.urlopen(req,timeout=180) as r:
            if r.status!=206:raise RuntimeError('Server ignored byte range; refusing full download')
            payload=r.read()
        self.pos+=len(payload);return payload
def main():
    out=ROOT/'data/sources/lafan1-resolved';out.mkdir(parents=True,exist_ok=True)
    manifest=[]
    with zipfile.ZipFile(RemoteZip()) as z:
        available=[i for i in z.infolist() if 'walk' in i.filename.lower() and i.filename.endswith('.bvh')]
        print(json.dumps([dict(name=i.filename,bytes=i.file_size) for i in available]),flush=True)
        for name in ['walk1_subject1.bvh','walk2_subject1.bvh']:
            matches=[i for i in available if Path(i.filename).name==name]
            if len(matches)!=1:raise ValueError((name,len(matches)))
            target=out/name
            if not target.exists():target.write_bytes(z.read(matches[0]))
            content=target.read_bytes();assert len(content)==matches[0].file_size
            manifest.append(dict(file=name,source_url=URL,archive_entry=matches[0].filename,bytes=len(content),sha256=hashlib.sha256(content).hexdigest()))
            print(json.dumps(manifest[-1]),flush=True)
    (out/'walk_sources_manifest.json').write_text(json.dumps(manifest,indent=2))
if __name__=='__main__':main()
