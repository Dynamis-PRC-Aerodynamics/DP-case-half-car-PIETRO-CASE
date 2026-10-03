#!/usr/bin/env python3
"""Writes system/controlDict_surfaces, the cutting planes sampled after a run.

Usage:  python3 postProcessor/generate_surfaces.py [x] [y] [z] [--whole]

With no axis, all three are written. The run scripts then pass the dictionary
to simpleFoam -postProcess, which writes one VTK file per plane in
postProcessing/surfaces/<time>/.

The plane positions must match the ones aeroTools expects when it renders the
slices: change them here and in aeroTools/generate_surfaces.py together.
"""

import argparse
import os
import re

import numpy as np

# The case root is the parent of this folder, wherever the script is run from.
scriptDir = os.path.dirname(os.path.abspath(__file__))
caseDir = os.path.dirname(scriptDir)

# Fields loaded from the latest time and sampled on every plane. Cp, CpT and
# the velocity components are derived from these in ParaView.
READ_FIELDS = ['p', 'U', 'k', 'omega', 'pMean', 'UMean', 'vorticity', 'Lambda2']
SAMPLED_FIELDS = ['pMean', 'UMean', 'vorticity', 'Lambda2']

NORMALS = {'x': '(1 0 0)', 'y': '(0 1 0)', 'z': '(0 0 1)'}
AXES = ['x', 'y', 'z']


def planePositions(whole):
    """Plane coordinates per axis, in metres, in the Aero frame."""

    # x: one plane at -0.95, then 20 mm steps from -0.90 to 2.00, then 50 mm
    # steps up to 2.30. Built from integers so that zero is exactly 0.000.
    x = np.concatenate((
        np.array([-0.95]),
        np.around(np.arange(-45, 101) * 0.02, decimals=3),
        np.around(np.arange(41, 47) * 0.05, decimals=3),
    ))

    # y: 25 mm steps up to 0.75, from -0.80 on a whole car. The plane at y = 0
    # is moved to 0.01 because it would coincide with the symmetry plane.
    if whole:
        y = np.around(np.arange(-32, 31) * 0.025, decimals=3)
        y[y == 0.0] = 0.01
    else:
        y = np.around(np.arange(0, 31) * 0.025, decimals=3)
        y[0] = 0.01

    # z: one plane at 0.01, then 20 mm steps from 0.02 to 1.10, then 50 mm
    # steps up to 1.45.
    z = np.concatenate((
        np.array([0.01]),
        np.around(np.arange(1, 56) * 0.02, decimals=3),
        np.around(np.arange(23, 30) * 0.05, decimals=3),
    ))

    return {'x': x, 'y': y, 'z': z}


def readIterations():
    """endTime of the run, from initialConditions. Falls back to 1, which is
    harmless because the dictionary starts from latestTime."""
    try:
        with open(os.path.join(caseDir, 'initialConditions'), 'r') as f:
            match = re.search(r'^iterations\s+(\d+)', f.read(), re.MULTILINE)
        return int(match.group(1))
    except (OSError, AttributeError):
        return 1


def writeControlDict(axes, whole):
    positions = planePositions(whole)

    lines = [
        '/*--------------------------------*- C++ -*----------------------------------*\\',
        '| =========                 |                                                 |',
        '| \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox           |',
        '|  \\\\    /   O peration     | Version:  v2312                                 |',
        '|   \\\\  /    A nd           | Website:  www.openfoam.com                      |',
        '|    \\\\/     M anipulation  |                                                 |',
        '\\*---------------------------------------------------------------------------*/',
        'FoamFile',
        '{',
        '    version     2.0;',
        '    format      ascii;',
        '    class       dictionary;',
        '    object      controlDict;',
        '}',
        '',
        'application     simpleFoam;',
        'startFrom       latestTime;',
        'startTime       0;',
        'stopAt          endTime;',
        'endTime         %d;' % readIterations(),
        'deltaT          1;',
        'writeControl    timeStep;',
        'writeInterval   1;',
        'purgeWrite      0;',
        'writeFormat     binary;',
        'writePrecision  6;',
        'writeCompression off;',
        'timeFormat      general;',
        'timePrecision   6;',
        'runTimeModifiable true;',
        '',
        'functions',
        '{',
        '    readFields',
        '    {',
        '        libs        ("libfieldFunctionObjects.so");',
        '        type        readFields;',
        '        fields      (%s);' % ' '.join(READ_FIELDS),
        '    }',
        '',
        '    surfaces',
        '    {',
        '        type            surfaces;',
        '        libs            (sampling);',
        '        writeControl    onEnd;',
        '        fields          (%s);' % ' '.join(SAMPLED_FIELDS),
        '        interpolationScheme cellPoint;',
        '        surfaceFormat   vtk;',
        '',
        '        surfaces',
        '        {',
    ]

    nPlanes = 0
    for axis in axes:
        for value in positions[axis]:
            coord = '%.3f' % value
            # Plane name: x_m0p950, y_0p100, z_0p050.
            name = '%s_%s' % (axis, coord.replace('-', 'm').replace('.', 'p'))
            point = ['0', '0', '0']
            point[AXES.index(axis)] = coord

            lines += [
                '            %s' % name,
                '            {',
                '                type        cuttingPlane;',
                '                point       (%s);' % ' '.join(point),
                '                normal      %s;' % NORMALS[axis],
                '                interpolate true;',
                '            }',
            ]
            nPlanes += 1

    lines += ['        }', '    }', '}', '']

    outPath = os.path.join(caseDir, 'system', 'controlDict_surfaces')
    with open(outPath, 'w', newline='\n') as f:
        f.write('\n'.join(lines))

    return outPath, nPlanes


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Write system/controlDict_surfaces for this case.')
    parser.add_argument('axes', nargs='*', metavar='axis',
                        help='axes to slice along, among x y z (default: all)')
    parser.add_argument('--whole', action='store_true',
                        help='whole car: y planes on both sides of the car')
    args = parser.parse_args()

    invalid = [a for a in args.axes if a not in AXES]
    if invalid:
        parser.error('unknown axis: %s (use x, y, z)' % ' '.join(invalid))
    axes = args.axes or AXES

    outPath, nPlanes = writeControlDict(axes, args.whole)
    print('%d planes on %s written to %s' % (nPlanes, ', '.join(axes), outPath))
