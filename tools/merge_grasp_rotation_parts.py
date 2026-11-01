"""Merge disjoint/complementary offline sweep parts after workers stop."""
import argparse
import json
from pathlib import Path
import shutil
from explore_grasp_rotations import digest, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parts', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--x-indices', nargs=2, type=int, default=[10, 24])
    parser.add_argument('--y-indices', nargs=2, type=int, default=[-6, 8])
    args = parser.parse_args()
    manifests = [json.loads((p/'manifest.json').read_text()) for p in args.parts]
    reference = manifests[0]
    generator_key = 'tools/explore_grasp_rotations.py'
    archived_generator = ROOT/'.setup/rotation-search-original-solver/explore_grasp_rotations.py'
    available_generators = {digest(ROOT/generator_key)}
    if archived_generator.exists():
        available_generators.add(digest(archived_generator))
    for manifest in manifests:
        if manifest['inputs'][generator_key] not in available_generators:
            raise ValueError('unavailable historical generator source')
    for other in manifests[1:]:
        for key in ('schema', 'rotations', 'reference_rotation', 'ready_joints_28_rad',
                    'contact_z_m', 'velocity_m_s', 'contact_s', 'grid_step_m'):
            if other[key] != reference[key]:
                raise ValueError('incompatible sweep parts: '+key)
        if ({k:v for k,v in other['inputs'].items() if k != generator_key}
                != {k:v for k,v in reference['inputs'].items() if k != generator_key}):
            raise ValueError('incompatible model/configuration inputs')
    records = {}; owners = {}; variants = {}
    def rank(record, generator):
        if record['accepted']:
            checks = record['checks']
            return (0, checks['acceleration_limit_ratio'], checks['velocity_limit_ratio'],
                    checks['position_error_m'], record['sha256'])
        return (1, 0 if generator == digest(ROOT/generator_key) else 1)
    for part, manifest in zip(args.parts, manifests):
        generator = manifest['inputs'][generator_key]
        for record in manifest['records']:
            key = (round(record['xy_m'][0]/.03), round(record['xy_m'][1]/.03), record['rotation_id'])
            versions = variants.setdefault(key, {})
            if generator in versions and versions[generator][0] != record:
                raise ValueError('nonidentical same-generator duplicate '+str(key))
            versions[generator] = (record, part)
            if key not in records or rank(record, generator) < rank(records[key], owners[key][1]):
                records[key] = dict(record); owners[key] = (part, generator)
    expected = {(x, y, r['id']) for x in range(args.x_indices[0], args.x_indices[1]+1)
                for y in range(args.y_indices[0], args.y_indices[1]+1) for r in reference['rotations']}
    if set(records) != expected:
        raise ValueError(f'incomplete coverage: missing={len(expected-set(records))}, extra={len(set(records)-expected)}')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'sweep_parts').mkdir(exist_ok=True)
    receipts = []
    for index, (part, manifest) in enumerate(zip(args.parts, manifests)):
        receipt = args.output/'sweep_parts'/f'part-{index:02d}.json'
        receipt.write_text(json.dumps(manifest, indent=2))
        receipts.append(dict(source=str(part.resolve()), snapshot=str(receipt.relative_to(args.output)), sha256=digest(receipt)))
    for key, record in records.items():
        if not record['accepted']:
            continue
        source = owners[key][0]/record['file']; target = args.output/record['file']
        if digest(source) != record['sha256']:
            raise ValueError('source trajectory hash')
        if source.resolve() != target.resolve():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        if digest(target) != record['sha256']:
            raise ValueError('copied trajectory hash')
    for key, record in records.items():
        record['selected_generator_sha256'] = owners[key][1]
        record['search_variants'] = [dict(generator_sha256=sha, source_part=str(part.resolve()),
                                         accepted=variant['accepted'],
                                         **({k:variant[k] for k in ('file', 'sha256', 'checks')} if variant['accepted']
                                            else dict(reason=variant['reason'])))
                                     for sha, (variant, part) in sorted(variants[key].items())]
    reference.update(records=[records[key] for key in sorted(records)], complete=True,
                     x_indices=args.x_indices, y_indices=args.y_indices, merged_parts=receipts)
    reference['inputs'] = dict(reference['inputs'])
    reference['inputs'][generator_key] = digest(ROOT/generator_key)
    if archived_generator.exists():
        reference['inputs'][str(archived_generator.relative_to(ROOT))] = digest(archived_generator)
    reference['search_algorithm_provenance'] = (
        'Per-part generator hashes and complete records are preserved in sweep_parts. '
        'Original-solver candidates are retained as alternatives. The refined solver uses task-space early convergence '
        'and a 0.001 previous-knot joint regularizer to resolve redundant IK. '
        'For each contact-cell/rotation pair, a passing path is preferred, then lowest peak acceleration ratio. '
        'Both use the same full-path acceptance checks; every accepted saved path is independently rechecked.')
    (args.output/'manifest.json').write_text(json.dumps(reference, indent=2))
    print(json.dumps(dict(complete=True, combinations=len(records), accepted=sum(r['accepted'] for r in records.values()))))


if __name__ == '__main__':
    main()
