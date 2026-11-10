"""Check English publication surfaces without rewriting historical evidence."""
from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[1]
SOURCE_SUFFIXES={'.py','.pyi','.c','.h','.cpp','.hpp','.sh','.ps1','.js','.ts','.tsx'}
EXCLUDED={'third_party','.git','.local','.venv','venv','node_modules','__pycache__','outputs',
          'downloads','release-assets','build','dist','reports','captures','logs'}
CJK=re.compile(r'[\u3400-\u4dbf\u4e00-\u9fff\uf900-\ufaff]')


def main():
    violations=[];checked=0
    for path in sorted(ROOT.rglob('*')):
        if not path.is_file():continue
        relative=path.relative_to(ROOT)
        if any(part in EXCLUDED or part.startswith('.venv') for part in relative.parts):continue
        if relative.as_posix()!='README.md' and path.suffix not in SOURCE_SUFFIXES:continue
        checked+=1
        for number,line in enumerate(path.read_text(encoding='utf-8-sig').splitlines(),1):
            if CJK.search(line):violations.append(f'{relative.as_posix()}:{number}')
    if violations:
        print('English publication policy failed:\n'+'\n'.join(violations),file=sys.stderr)
        return 1
    print(f'English publication policy passed: {checked} source/README files checked.')
    return 0


if __name__=='__main__':raise SystemExit(main())
