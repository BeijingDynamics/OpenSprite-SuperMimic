"""Fetch selected original BVH entries with HTTP ranges, not the full 1GB archive."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse,io,json,time,urllib.request,zipfile,hashlib
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
        if stop-self.pos+1>1024*1024:
            ranges=[(begin,min(begin+512*1024-1,stop)) for begin in range(self.pos,stop+1,512*1024)]
            with ThreadPoolExecutor(max_workers=4) as pool:
                payload=b''.join(pool.map(lambda bounds:self._range(*bounds),ranges))
        else:payload=self._range(self.pos,stop)
        self.pos+=len(payload);return payload

    def _range(self,start,stop):
        req=urllib.request.Request(URL,headers={'Range':f'bytes={start}-{stop}'})
        with urllib.request.urlopen(req,timeout=30) as r:
            if r.status!=206:raise RuntimeError('Server ignored byte range; refusing full download')
            if r.headers.get('Content-Range')!=f'bytes {start}-{stop}/{self.size}':
                raise RuntimeError('Archive range identity changed')
            chunks=[];received=0;started=time.monotonic();last_log=started
            while received<stop-start+1:
                chunk=r.read(min(65536,stop-start+1-received))
                if not chunk:raise EOFError('Incomplete archive byte range')
                chunks.append(chunk);received+=len(chunk)
                now=time.monotonic()
                if now-last_log>10:
                    print(json.dumps(dict(event='archive_range_progress',offset=start,received=received,requested=stop-start+1,seconds=round(now-started,1))),flush=True)
                    last_log=now
                if now-started>600:raise TimeoutError('Archive range exceeded ten-minute transfer budget')
            payload=b''.join(chunks)
        return payload
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--list-only',action='store_true')
    parser.add_argument('--names',nargs='+',default=['walk1_subject1.bvh','walk2_subject1.bvh'])
    parser.add_argument('--output',type=Path,default=ROOT/'data/sources/lafan1-resolved')
    args=parser.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=True)
    manifest=[]
    with zipfile.ZipFile(RemoteZip()) as z:
        available=[i for i in z.infolist() if any(kind in Path(i.filename).name.lower() for kind in ('walk','run')) and i.filename.endswith('.bvh')]
        index=[dict(name=i.filename,bytes=i.file_size,compressed_bytes=i.compress_size) for i in available]
        print(json.dumps(index),flush=True)
        (out/'locomotion_archive_index.json').write_text(json.dumps(index,indent=2)+'\n')
        if args.list_only:return
        for name in args.names:
            if Path(name).name!=name:raise ValueError('Expected a plain archive basename')
            matches=[i for i in available if Path(i.filename).name==name]
            if len(matches)!=1:raise ValueError((name,len(matches)))
            target=out/name
            if not target.exists():target.write_bytes(z.read(matches[0]))
            content=target.read_bytes();assert len(content)==matches[0].file_size
            manifest.append(dict(file=name,source_url=URL,archive_entry=matches[0].filename,bytes=len(content),sha256=hashlib.sha256(content).hexdigest()))
            print(json.dumps(manifest[-1]),flush=True)
    (out/'walk_sources_manifest.json').write_text(json.dumps(manifest,indent=2))
if __name__=='__main__':main()
