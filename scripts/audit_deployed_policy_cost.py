"""Inspect ONNX inference graphs; estimates exclude non-linear operator costs."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import onnx


def inspect(path):
    model = onnx.load(path)
    onnx.checker.check_model(model)
    tensors = {t.name: t for t in model.graph.initializer}
    layers = []
    for node in model.graph.node:
        if node.op_type not in ('Gemm', 'MatMul'):
            continue
        weight = tensors.get(node.input[1])
        if weight is None or len(weight.dims) != 2:
            raise ValueError('MAC estimate requires a constant matrix: '+node.name)
        layers.append(dict(name=node.name, weight_shape=list(weight.dims),
                           macs_batch_one=math.prod(weight.dims)))
    total = sum(r['macs_batch_one'] for r in layers)
    return dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                bytes=path.stat().st_size, operators=dict(Counter(n.op_type for n in model.graph.node)),
                initializer_elements=sum(math.prod(t.dims) for t in tensors.values()),
                dense_macs_batch_one=total, dense_flops_batch_one=total*2,
                dense_macs_per_second_50hz=total*50,
                layers=layers,
                limitation='Dense operation count only; excludes activation, normalization, memory traffic, thread synchronization and graph overhead. Initializers include normalization statistics and may share tied values.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--models', type=Path, nargs='+', required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    results = [inspect(model) for model in args.models]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2)+'\n', encoding='utf-8')
    for r in results:
        print(json.dumps({k:r[k] for k in ['path','sha256','bytes','dense_macs_batch_one','initializer_elements']}))


if __name__ == '__main__':
    main()
