import os, sys, math
import numpy as np
from math import atan, degrees

# Get the script path
script_path = os.path.abspath(__file__)

# Get the directory path
setupDir = os.path.dirname(script_path)
caseDir = os.path.dirname(setupDir)
os.chdir(caseDir)

# ================ PARAMETERS =============

# Geometries that follow the sprung mass: they take the pitch of the car.
# Wheels, plinths and uprights stay on the ground and are left alone.
# Fan_Battery_* is not listed by name: the battery fans are produced by
# splitAssemblies() and are picked up from BODY_PREFIX below, so the list does
# not have to be edited when the number of fan groups changes.
BODY = ["Monocoque", "Monocoque_Kick", "Monocoque_Duct", "Radiator_Ducts",
        "Susps", "FW", "RW", "RW_Upper", "Winglets", "T_Tray", "Diffuser", "UT",
        "Porous_Radiators", "Porous_Battery", "Fan_Radiators"]

# Every file whose name starts with one of these follows the sprung mass too.
BODY_PREFIX = ["Fan_Battery_"]

# Assemblies the CAD hands over as one file and the case needs split.
SPLIT_BY_PLANE      = {"Wheels": ("Wheels_Front", "Wheels_Rear")}
# One file per connected component. The number is how many components are
# expected INSIDE the domain, not how many the CAD exports: the battery has
# four fans and the symmetry plane cuts two of them out.
SPLIT_BY_COMPONENTS = {"Fan_Battery": 2}

# The CAD exports the whole car; the straight line case meshes only y > 0, the
# rest is cut away by the domain. Bodies are left alone - snappy ignores what
# falls outside - but a body that becomes its own cellZone cannot: a fan disc
# entirely at y < 0 would produce an empty cellZone and a fvOptions source that
# silently does nothing. Those are dropped here instead.
SYMMETRY_Y = 0.0

# Expected content of constant/triSurface after the split.
N_SURFACES_EXPECTED = 21

# Below this, a pitch/steer angle or a height offset counts as zero and the
# whole transform is dropped instead of being composed. Angles are in
# degrees, lengths in metres; the OBJ files carry ~6 significant digits, i.e.
# about 1e-5 m on a 1.5 m car, so anything under 1e-9 is far below the noise.
TOL_ANGLE = 1.0e-9
TOL_LENGTH = 1.0e-9

file_path = os.path.join(setupDir, "setup.txt")
with open(file_path, "r") as file:
		lines = file.readlines()

FRHline = lines[2].split("=")
if len(FRHline) > 1:
	FRH = float(FRHline[1].strip())
else:
	print("Errore: nessun valore assegnato a FRH\n")

RRHline = lines[3].split("=")
if len(RRHline) > 1:
	RRH = float(RRHline[1].strip())
else:
	print("Errore: nessun valore assegnato a RRH\n")

rollline = lines[4].split("=")
if len(rollline) > 1:
	roll = float(rollline[1].strip())
else:
	print("Errore: nessun valore assegnato a roll\n")

yawline = lines[5].split("=")
if len(yawline) > 1:
	yaw = float(yawline[1].strip())
else:
	print("Errore: nessun valore assegnato a yaw\n")

steerline = lines[6].split("=")
if len(steerline) > 1:
	steer = float(steerline[1].strip())
else:
	print("Errore: nessun valore assegnato a steer\n")

regZline = lines[7].split("=")
if len(regZline) > 1:
	regZ = float(regZline[1].strip())
else:
	print("Errore: nessun valore assegnato a dZ\n")

AoAline = lines[8].split("=")
if len(AoAline) > 1:
	AoAFW = float(AoAline[1].strip())
else:
	print("Errore: nessun valore assegnato a AoA\n")

attfrontline = lines[9]
start_index = attfrontline.find('(')
end_index = attfrontline.find(')')+1
if start_index >= 0:
	attfront = np.array([float(v) for v in attfrontline[start_index:end_index].strip('()').split()])
else:
	print("Errore: nessun valore assegnato a AttFront\n")

attrearline = lines[10]
start_index = attrearline.find('(')
end_index = attrearline.find(')')+1
if start_index >= 0:
	attrear = np.array([float(v) for v in attrearline[start_index:end_index].strip('()').split()])
else:
	print("Errore: nessun valore assegnato a AttRear\n")

file.close()

# Front suspension hardpoints (upper/lower ball joints), used only to build the
# kingpin axis for the front-wheel steer rotation below and for the diagnostic
# axis/center recompute at the bottom of this file. Not used by the pitch/FW
# transforms above, which is why this is a separate file from setup.txt.
file_path = os.path.join(setupDir, "setup_UBJ_LBJ.txt")
with open(file_path, "r") as file:
		lines = file.readlines()

def _readPoint(line):
	start_index = line.find('(')
	end_index = line.find(')')+1
	return np.array([float(v) for v in line[start_index:end_index].strip('()').split()])

UBJ_R = _readPoint(lines[7])
LBJ_R = _readPoint(lines[8])
UBJ_L = _readPoint(lines[9])
LBJ_L = _readPoint(lines[10])

L_axis = UBJ_L - LBJ_L				# left kingpin axis
L_norm = np.linalg.norm(L_axis)
L_versor = L_axis / L_norm

R_axis = UBJ_R - LBJ_R				# right kingpin axis
R_norm = np.linalg.norm(R_axis)
R_versor = R_axis / R_norm

zpitchBase = 0.203			# Wheel radius (m)
xpitch = 1.53				# Wheelbase (m)
pitch=-degrees(atan((RRH-FRH)/(xpitch*1000)))	# pitch angle (deg)
zpitch=RRH/1000-0.035			# dZ imposed on RRH (m)

L = attrear[0] - attfront[0]			# x distance between the mounting brackets [m]
dz = regZ/1000							# z height variation [m]
L1 = 0.19481							# x distance between the DP14 mounting brackets
AoAFWvero = degrees(atan(math.tan(np.radians(AoAFW))*L1/L))	# true adjustment AoA


# ====================================
os.system("rm -f constant/triSurface/*")

os.system("cp -a constant/triSurface_0deg/. constant/triSurface/")

os.chdir("constant/triSurface")


# ============================================================= obj utilities
# The OBJ files are the bulk of the run time and none of it is arithmetic:
# it is ASCII parsing and ASCII writing. Two consequences drive the code below.
#   1. every file is read once and written once, with all the rigid transforms
#      composed into a single 4x4 beforehand;
#   2. for the bodies that are not split, the face block is never parsed nor
#      reformatted - it is copied through as raw bytes, since the connectivity
#      does not change when the points move.
CHUNK = 1 << 16


def _blockOf(buf, tag, start=0):
	"""Byte range of the contiguous run of lines starting with `tag ` (b'v' or
	b'f'). Returns (begin, end) or (-1, -1). Leading and trailing header or
	comment lines are trimmed off, not parsed."""
	pre = tag + b" "
	if buf.startswith(pre, start):
		b = start
	else:
		b = buf.find(b"\n" + pre, start)
		if b < 0:
			return -1, -1
		b += 1
	e = buf.rfind(b"\n" + pre)
	if e < b:						# single line
		e = buf.find(b"\n", b)
		return b, (len(buf) if e < 0 else e + 1)
	e = buf.find(b"\n", e + 1)
	return b, (len(buf) if e < 0 else e + 1)


def _isPlain(seg, tag):
	"""True when `seg` is nothing but lines of the form `tag a b c`."""
	return seg.count(b"\n") == seg.count(tag + b" ")


def readObjSlow(path):
	"""Generic fallback: handles quads, v/vt/vn indices, interleaved comments."""
	verts, faces = [], []
	with open(path, "r") as fp:
		for line in fp:
			p = line.split()
			if not p:
				continue
			if p[0] == "v":
				verts.append((float(p[1]), float(p[2]), float(p[3])))
			elif p[0] == "f":
				idx = [int(tok.split("/")[0]) for tok in p[1:]]
				faces.append([i-1 if i > 0 else len(verts)+i for i in idx])
	return np.array(verts, dtype=float), faces


def readObjPoints(path):
	"""Vertices as an (n,3) array, plus everything around them as raw bytes, so
	the file can be rewritten byte for byte except for the point coordinates."""
	with open(path, "rb") as fp:
		buf = fp.read()
	b, e = _blockOf(buf, b"v")
	if b < 0:
		raise SystemExit("!! %s: nessun vertice" % path)
	seg = buf[b:e]
	if not _isPlain(seg, b"v"):
		return None, buf, b""				# caller falls back
	V = np.fromstring(seg.replace(b"v ", b""), sep=" ")
	if V.size % 3:
		return None, buf, b""
	return V.reshape(-1, 3), buf[:b], buf[e:]


def readObj(path):
	"""Vertices and faces as arrays. Only needed for the assemblies we split."""
	with open(path, "rb") as fp:
		buf = fp.read()
	vb, ve = _blockOf(buf, b"v")
	fb, fe = _blockOf(buf, b"f", ve if ve > 0 else 0)
	vseg, fseg = buf[vb:ve], buf[fb:fe]
	if (vb < 0 or fb < 0 or not _isPlain(vseg, b"v")
			or not _isPlain(fseg, b"f") or b"/" in fseg):
		V, faces = readObjSlow(path)
		return V, np.array(faces, dtype=np.int64)
	V = np.fromstring(vseg.replace(b"v ", b""), sep=" ")
	F = np.fromstring(fseg.replace(b"f ", b""), sep=" ")
	if V.size % 3 or F.size % 3:
		V, faces = readObjSlow(path)
		return V, np.array(faces, dtype=np.int64)
	return V.reshape(-1, 3), F.astype(np.int64).reshape(-1, 3) - 1


def _writePoints(fp, V):
	for k in range(0, len(V), CHUNK):
		c = V[k:k+CHUNK]
		fp.write((("v %.6f %.6f %.6f\n" * len(c)) % tuple(c.ravel())).encode())


def writeObjPoints(path, V, head, tail):
	"""Rewrite the points, splice header and face block back verbatim."""
	with open(path, "wb") as fp:
		fp.write(head)
		_writePoints(fp, V)
		fp.write(tail)


def writeObj(path, V, F):
	"""Writes only the vertices actually used by the given faces, renumbered."""
	used = np.unique(F)
	remap = np.zeros(len(V), dtype=np.int64)
	remap[used] = np.arange(1, len(used)+1)
	Fr = remap[F]
	with open(path, "wb") as fp:
		fp.write(b"# Wavefront OBJ file written by preProcessor_pietro.py\n")
		fp.write(b"o %s\n\n" % os.path.basename(path)[:-4].encode())
		fp.write(b"# points : %d\n" % len(used))
		_writePoints(fp, V[used])
		fp.write(b"\n# faces  : %d\n" % len(Fr))
		for k in range(0, len(Fr), CHUNK):
			c = Fr[k:k+CHUNK]
			fp.write((("f %d %d %d\n" * len(c)) % tuple(c.ravel())).encode())


def centroids(V, F):
	"""Face centroids, (m,3)."""
	return V[F].mean(axis=1)


# ============================================================= transforms
# surfaceTransformPoints -rollPitchYaw '(r p y)' rotates about the origin with
# Rz(y) . Ry(p) . Rx(r); -translate adds a vector afterwards. Everything the
# preProcessor does is one of those two, so every step is an affine 4x4 and the
# whole chain collapses into a single matrix product.
def T(t):
	M = np.eye(4)
	M[:3, 3] = t
	return M


def Ry(deg):
	c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
	M = np.eye(4)
	M[0, 0], M[0, 2] = c, s
	M[2, 0], M[2, 2] = -s, c
	return M


def Rz(deg):
	c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
	M = np.eye(4)
	M[0, 0], M[0, 1] = c, -s
	M[1, 0], M[1, 1] = s, c
	return M


def Raxis(versor, deg):
	"""Rodrigues rotation of `deg` about an arbitrary unit axis `versor`,
	through the origin. Used for the kingpin (steer) rotation, whose axis is
	not aligned with x/y/z."""
	theta = math.radians(deg)
	x, y, z = versor
	c, s = math.cos(theta), math.sin(theta)
	R = np.array([
		[x*x + (1-x*x)*c,     x*y*(1-c) - z*s,   x*z*(1-c) + y*s],
		[x*y*(1-c) + z*s,     y*y + (1-y*y)*c,   y*z*(1-c) - x*s],
		[x*z*(1-c) - y*s,     y*z*(1-c) + x*s,   z*z + (1-z*z)*c],
	])
	M = np.eye(4)
	M[:3, :3] = R
	return M


def applyT(M, V):
	if M is None:
		return V
	return V @ M[:3, :3].T + M[:3, 3]


# CAD has its origin on the rear axle, x towards the nose, y towards the left.
# OpenFOAM has its origin on the front axle, x towards the rear, y towards the
# right: 180 degree rotation about z plus a wheelbase translation in x, i.e.
#     x_aero = -x_cad + 1.53      y_aero = -y_cad      z_aero = z_cad
# The y flip is not optional: dropping it mirrors the car onto the discarded
# half of the symmetric domain.
M_CAD_TO_AERO = T([xpitch, 0.0, 0.0]) @ Rz(180.0)


def pitchMatrix():
	"""Car pitch about the rear axle contact point, plus the RRH lift.
	Identity at the reference ride height (FRH = RRH = 35 mm)."""
	if abs(pitch) < TOL_ANGLE and abs(zpitch) < TOL_LENGTH:
		return None
	return (T([xpitch, 0.0, zpitchBase + zpitch])
			@ Ry(pitch)
			@ T([-xpitch, 0.0, -zpitchBase]))


def fwMatrix():
	"""Front wing incidence about the rear mounting bracket, plus dZ.
	Identity at AoA = 0 and dZ = 0."""
	if abs(AoAFWvero) < TOL_ANGLE and abs(dz) < TOL_LENGTH:
		return None
	return (T([attrear[0], 0.0, attrear[2] + dz])
			@ Ry(AoAFWvero)
			@ T([-attrear[0], 0.0, -attrear[2]]))


def steerMatrix():
	"""Front wheel steer rotation about the right kingpin axis (UBJ_R-LBJ_R),
	pivoting at LBJ_R. Identity at steer = 0.

	Only the right-side hardpoints are used: the case is a y > 0 half-car, and
	Wheels_Front (the only surface this is applied to, see run()) is what
	survives the symmetry cut. Wheels_Rear does not steer. Tyre_Plinths keeps
	a plain slip BC with no rotational dependence (see orig0/U), and it is a
	single file merging all four corners in this geometry, so it is left
	un-rotated here too - splitting it front/rear to carry the steer rotation
	would add a patch DP18_Case_Personal does not have."""
	if abs(steer) < TOL_ANGLE:
		return None
	return T(LBJ_R) @ Raxis(R_versor, steer) @ T(-LBJ_R)


def bodyMatrix(name, M_pitch, M_fw, M_steer):
	"""The single matrix that takes `name` from the CAD frame to its final
	position: CAD->aero, then the front wing setting / steer, then the car
	pitch."""
	M = M_CAD_TO_AERO
	if name == "FW" and M_fw is not None:
		M = M_fw @ M
	if name == "Wheels_Front" and M_steer is not None:
		M = M_steer @ M
	if M_pitch is not None and (name in BODY
								or any(name.startswith(p) for p in BODY_PREFIX)):
		M = M_pitch @ M
	return M


# ============================================================= split
# The CAD hands over one file per assembly, but two of them hold parts that
# the case has to treat separately: the wheels need one rotation centre each,
# the battery fans one fanMomentumSource each.
def splitByPlane(V, F, x_cut, nameLow, nameHigh):
	"""Splits an OBJ in two along a plane normal to x, by face centroid."""
	cx = centroids(V, F)[:, 0]
	low, high = F[cx < x_cut], F[cx >= x_cut]
	print("\t%s -> %s (%d facce) + %s (%d facce)"
		  % ("Wheels", nameLow, len(low), nameHigh, len(high)))
	if not len(low) or not len(high):
		raise SystemExit("\t!! Wheels non si divide sul piano x = %.3f" % x_cut)
	return {nameLow: low, nameHigh: high}


def connectedComponents(V, F):
	"""Faces grouped by connected component, union-find on shared vertices."""
	parent = np.arange(len(V))

	def find(a):
		while parent[a] != a:
			parent[a] = parent[parent[a]]
			a = parent[a]
		return a

	for f in F:
		r0 = find(f[0])
		for i in f[1:]:
			ri = find(i)
			if ri != r0:
				parent[ri] = r0
	roots = np.array([find(f[0]) for f in F])
	return [F[roots == r] for r in np.unique(roots)]


def splitByComponents(V, F, name, nExpected):
	"""One output file per connected component, keeping only the components
	that sit inside the meshed half of the car.

	The battery pack has four fans and the CAD exports all of them, but the
	straight line domain starts at y = 0 and two of them fall outside it. One
	disc is one cellZone with its own fan curve, so the components are NOT
	merged: a cellZone holding two discs would read the fan curve at twice the
	flow rate, and a cellZone entirely outside the domain would come out empty
	with its fvOptions source silently doing nothing.

	Naming is by centroid y so it is reproducible from the geometry alone. The
	order in which union-find happens to visit the faces is not: with the wrong
	order the cellZones of fvOptions would land on different discs at every
	re-export of the CAD."""
	parts = connectedComponents(V, F)
	print("\t%s -> %d componenti connesse" % (name, len(parts)))

	# centroid of each component, y first, then x and z to break ties
	key = []
	for k, g in enumerate(parts):
		c = centroids(V, g).mean(axis=0)
		key.append((c[1], c[0], c[2], k))
	key.sort()

	keep = [t for t in key if t[0] > SYMMETRY_Y]
	drop = [t for t in key if t[0] <= SYMMETRY_Y]

	if drop:
		print("\t\t%s oltre il piano di simmetria y = %.3f, "
			  "nella meta' non simulata, scartat%s:"
			  % ("%d componenti" % len(drop) if len(drop) > 1 else "1 componente",
				 SYMMETRY_Y, "e" if len(drop) > 1 else "a"))
		for cy, cx, cz, k in drop:
			print("\t\t\ty = %+.4f (%d facce)" % (cy, len(parts[k])))

	if len(keep) != nExpected:
		raise SystemExit(
			"\t!! %s: attese %d componenti dentro il dominio, trovate %d "
			"(%d componenti in tutto). Controlla l'export del CAD."
			% (name, nExpected, len(keep), len(parts)))

	out = {}
	for gi, (cy, cx, cz, k) in enumerate(keep, 1):
		out["%s_%d" % (name, gi)] = parts[k]
		print("\t\t%s_%d.obj (%d facce, y = %+.4f)"
			  % (name, gi, len(parts[k]), cy))
	return out


def splitAssemblies(V, F, name, M_pitch, M_fw):
	if name in SPLIT_BY_PLANE:
		lo, hi = SPLIT_BY_PLANE[name]
		return splitByPlane(V, F, xpitch/2.0, lo, hi)
	return splitByComponents(V, F, name, SPLIT_BY_COMPONENTS[name])


# ============================================================= run
def run():
	M_pitch = pitchMatrix()
	M_fw = fwMatrix()
	M_steer = steerMatrix()

	print("\nFRH: %.1f mm" % FRH)
	print("RRH: %.1f mm" % RRH)
	print("Angolo di pitch: %.3f gradi%s"
		  % (pitch, "" if M_pitch is not None else "   (identita', saltato)"))
	print("Angolo di pitch FW (regolazione): %.3f gradi (rot asse -y)" % -AoAFW)
	print("Angolo di pitch FW (vero): %.3f gradi (rot asse -y)%s"
		  % (-AoAFWvero, "" if M_fw is not None else "   (identita', saltato)"))
	print("Variazione di altezza FW: %d mm" % regZ)
	print("Angolo di steer: %.3f gradi (Wheels_Front, asse kingpin destro)%s"
		  % (steer, "" if M_steer is not None else "   (identita', saltato)"))

	names = sorted(f[:-4] for f in os.listdir(".") if f.endswith(".obj"))
	toSplit = set(SPLIT_BY_PLANE) | set(SPLIT_BY_COMPONENTS)

	print("\nDa riferimento macchina a riferimento aero, assetto e split")
	written = 0
	for name in names:
		src = name + ".obj"

		if name not in toSplit:
			# one read, one composed matrix, one write; header and face block
			# are copied through untouched
			print("\t%s" % name)
			V, head, tail = readObjPoints(src)
			if V is None:					# unusual OBJ, generic path
				V, faces = readObjSlow(src)
				writeObj(src, applyT(bodyMatrix(name, M_pitch, M_fw, M_steer), V),
						 np.array(faces, dtype=np.int64))
			else:
				writeObjPoints(src, applyT(bodyMatrix(name, M_pitch, M_fw, M_steer), V),
							   head, tail)
			written += 1
			continue

		# split assemblies: the parts can take different transforms, so the
		# CAD->aero step is applied first and the rest per output file
		V, F = readObj(src)
		V = applyT(M_CAD_TO_AERO, V)
		print("\t%s" % name)
		for part, faces in splitAssemblies(V, F, name, M_pitch, M_fw).items():
			Mrest = None
			if part == "Wheels_Front" and M_steer is not None:
				Mrest = M_steer
			if M_pitch is not None and (part in BODY
										or any(part.startswith(p) for p in BODY_PREFIX)):
				Mrest = M_pitch if Mrest is None else M_pitch @ Mrest
			writeObj(part + ".obj", applyT(Mrest, V), faces)
			written += 1
		os.remove(src)

	print("\nconstant/triSurface: %d file" % written)
	if written != N_SURFACES_EXPECTED:
		raise SystemExit(
			"!! attesi %d file in constant/triSurface, scritti %d: "
			"lo split non e' andato a buon fine"
			% (N_SURFACES_EXPECTED, written))


# ============================================================= wheel axis diagnostic
# Informational only: does not write anything back to initialConditions. Ported
# from DP17_Case_Pietro/preProcessor.py. FLCenter/FLaxis/... in initialConditions
# are static reference values (at steer = yaw = 0); this recomputes what they
# would become for the current steer/yaw in setup.txt so the new numbers can be
# pasted in by hand if a sweep needs them. Only FRCenter/FRaxis/FRomega and
# RRCenter/RRaxis/RRomega actually feed orig0/U (see steerMatrix() above for why
# only the right side matters), but all four corners are recomputed here to
# mirror DP17's original output.
def _split_and_get_array(lines, index_line):
	line = lines[index_line]
	start_index = line.find('(')
	end_index = line.find(')')+1
	return np.array([float(v) for v in line[start_index:end_index].strip('()').split()])


def _rotation_tensor_z(alpha_z):
	c, s = np.cos(alpha_z), np.sin(alpha_z)
	return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rotation_tensor_steer(theta, versor):
	c, s = np.cos(theta), np.sin(theta)
	x, y, z = versor
	return np.array([
		[x**2 + (1-x**2)*c, x*y*(1-c) - z*s, x*z*(1-c) + y*s],
		[x*y*(1-c) + z*s, y**2 + (1-y**2)*c, y*z*(1-c) - x*s],
		[x*z*(1-c) - y*s, y*z*(1-c) + x*s, z**2 + (1-z**2)*c],
	])


def printWheelAxisDiagnostic():
	with open(os.path.join(caseDir, "initialConditions"), "r") as f:
		icLines = f.readlines()

	FLCenter = FRCenter = RLCenter = RRCenter = None
	FLaxis = FRaxis = RLaxis = RRaxis = None
	for i, line in enumerate(icLines):
		s = line.strip()
		if s.startswith("FLCenter"):
			FLCenter = _split_and_get_array(icLines, i)
		elif s.startswith("FLaxis"):
			FLaxis = _split_and_get_array(icLines, i)
		elif s.startswith("FRCenter"):
			FRCenter = _split_and_get_array(icLines, i)
		elif s.startswith("FRaxis"):
			FRaxis = _split_and_get_array(icLines, i)
		elif s.startswith("RLCenter"):
			RLCenter = _split_and_get_array(icLines, i)
		elif s.startswith("RLaxis"):
			RLaxis = _split_and_get_array(icLines, i)
		elif s.startswith("RRCenter"):
			RRCenter = _split_and_get_array(icLines, i)
		elif s.startswith("RRaxis"):
			RRaxis = _split_and_get_array(icLines, i)

	if steer == 0 and yaw == 0:
		return		# identity: initialConditions already holds the right numbers

	yaw_rad = math.radians(yaw)
	steer_rad = math.radians(steer)

	steer_tensor_right = _rotation_tensor_steer(steer_rad, R_versor)
	steer_tensor_left = _rotation_tensor_steer(steer_rad, L_versor)
	FLCenter = np.dot(steer_tensor_left, FLCenter - LBJ_L) + LBJ_L
	FLaxis = np.dot(steer_tensor_left, FLaxis)
	FRCenter = np.dot(steer_tensor_right, FRCenter - LBJ_R) + LBJ_R
	FRaxis = np.dot(steer_tensor_right, FRaxis)

	yaw_tensor = _rotation_tensor_z(yaw_rad)
	FLCenter, FLaxis, FRCenter, FRaxis, RLCenter, RLaxis, RRCenter, RRaxis = (
		np.dot(yaw_tensor, FLCenter), np.dot(yaw_tensor, FLaxis),
		np.dot(yaw_tensor, FRCenter), np.dot(yaw_tensor, FRaxis),
		np.dot(yaw_tensor, RLCenter), np.dot(yaw_tensor, RLaxis),
		np.dot(yaw_tensor, RRCenter), np.dot(yaw_tensor, RRaxis),
	)

	print("\nNUOVI CENTRI E ASSI RUOTA (steer/yaw != 0, da incollare a mano in "
		  "initialConditions se serve):\n")
	for label, val in [("FLCenter", FLCenter), ("FLaxis", FLaxis),
						("FRCenter", FRCenter), ("FRaxis", FRaxis),
						("RLCenter", RLCenter), ("RLaxis", RLaxis),
						("RRCenter", RRCenter), ("RRaxis", RRaxis)]:
		print("\t%s: (%.7f %.7f %.7f)" % (label, val[0], val[1], val[2]))


if __name__ == "__main__":
	run()
	os.chdir(caseDir)
	printWheelAxisDiagnostic()
