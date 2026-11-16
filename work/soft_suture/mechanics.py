"""Reduced-order elastic tissue and sliding continuous cable (SI units).

Implicit elastic lattice with a volume penalty, mass/inertia, damping, unilateral tray/wound
contact, force-triggered tip puncture and a compliant tension-only cable.
This is NOT a calibrated constitutive tissue model or a cutting/FEM solver.
The cable slides frictionlessly through material eyelets; it has one global
length constraint, rather than independent stitches or prescribed closure.
"""
from dataclasses import dataclass
import itertools
import numpy as np
from scipy.optimize import minimize


@dataclass
class Parameters:
    spring_stiffness: float = 35.0  # N/m per lattice edge, not Young's modulus
    cable_stiffness: float = 1500.0  # N/m for the whole strand
    damping: float = 18.0          # 1/s velocity damping
    puncture_force: float = 0.04   # N, illustrative threshold
    node_mass: float = 0.0003     # kg, lumped tissue mass
    dt: float = 0.01
    tip_radius: float = 0.0015
    gap_contact: float = 0.0003
    gravity: float = 9.81
    bulk_modulus: float = 5000.0  # Pa, volumetric penalty (illustrative)
    strain_stiffening: float = 0.0
    edge_viscosity: float = 0.0  # N s/m, axial Kelvin-Voigt damping
    distributed_contact: bool = False

    def __post_init__(self):
        for name in ['spring_stiffness','cable_stiffness','puncture_force',
                     'node_mass','dt','tip_radius','gap_contact','bulk_modulus']:
            if not np.isfinite(getattr(self,name)) or getattr(self,name)<=0:
                raise ValueError(name+' must be finite and positive')
        if self.damping<0 or self.gravity<0 or self.strain_stiffening<0 or self.edge_viscosity<0:
            raise ValueError('Damping and gravity must be non-negative')


class SoftSuture:
    def __init__(self, parameters=None):
        self.p = parameters or Parameters()
        self.origin = np.array([-.03, .345, .5755])
        self.xs = np.array([.002, .005, .008, .014, .026, .05])
        self.ys = np.array([-.035, -.020, -.009, -.006, -.003, 0, .003,
                            .006, .009, .012, .015, .018, .021, .026, .035])
        self.zs = np.array([.004, .009, .014])
        nx, ny, nz = len(self.xs), len(self.ys), len(self.zs)
        self.ids = np.arange(2*nx*ny*nz).reshape(2, nx, ny, nz)
        points = []
        for side in [-1, 1]:
            points.extend([self.origin+[side*x, y, z]
                           for x in self.xs for y in self.ys for z in self.zs])
        self.x = np.array(points); self.rest = self.x.copy()
        self.n_tissue = len(points)
        self.v = np.zeros_like(self.x)
        self.w = np.full(len(self.x), 1/self.p.node_mass)
        self.fixed = np.unique(np.concatenate([self.ids[:, -1].ravel(),
                        self.ids[:, :, 0].ravel(), self.ids[:, :, -1].ravel()]))
        self.w[self.fixed] = 0
        edges = set()
        # Axial, face diagonal, and body diagonal springs resist shear.
        directions = [d for d in itertools.product([-1, 0, 1], repeat=3)
                      if d > (0, 0, 0)]
        for s, i, j, k in np.ndindex(self.ids.shape):
            for di, dj, dk in directions:
                b = (i+di, j+dj, k+dk)
                if 0 <= b[0] < nx and 0 <= b[1] < ny and 0 <= b[2] < nz:
                    edges.add(tuple(sorted((self.ids[s,i,j,k], self.ids[(s,)+b]))))
        self.edges = np.array(sorted(edges), dtype=int)
        self.lengths = np.linalg.norm(self.x[self.edges[:,1]]-self.x[self.edges[:,0]], axis=1)
        self.faces = self._faces()
        tets=[]
        for side in range(2):
            for i,j,k in np.ndindex(nx-1,ny-1,nz-1):
                for order in itertools.permutations(range(3)):
                    corner=np.array([i,j,k]);v=[self.ids[(side,)+tuple(corner)]]
                    for axis in order:
                        corner=corner.copy();corner[axis]+=1
                        v.append(self.ids[(side,)+tuple(corner)])
                    tets.append(v)
        self.tets=np.array(tets,dtype=int)
        z=self.rest[self.tets]
        V=np.einsum('ij,ij->i',z[:,1]-z[:,0],np.cross(z[:,2]-z[:,0],z[:,3]-z[:,0]))/6
        self.volume0=V
        self.top = self.ids[:,:,:, -1].ravel()
        self.inner = self.ids[:,0,:,:]
        self.route = []
        self.cable_rest = 0.
        self.cable_end = None
        self.punctured = set()
        self.puncture_events = []
        self.time = 0.
        self.tension = 0.
        self.tip_force = 0.

    def _faces(self):
        faces=[]
        for s in range(2):
            g=self.ids[s]
            for axis in range(3):
                for end in [0,-1]:
                    plane=np.take(g,end,axis=axis)
                    for i in range(plane.shape[0]-1):
                        for j in range(plane.shape[1]-1):
                            a,b,c,d=plane[i,j],plane[i+1,j],plane[i+1,j+1],plane[i,j+1]
                            faces.extend([[a,b,c],[a,c,d]])
        return np.array(faces,dtype=int)

    def add_particle(self, point, fixed=False):
        n=len(self.x)
        self.x=np.vstack([self.x,point]);self.rest=np.vstack([self.rest,point])
        self.v=np.vstack([self.v,[0.,0.,0.]])
        self.w=np.r_[self.w,0 if fixed else 1/0.00002]
        return n

    def deploy(self, stitch, tail, slack=0.002):
        """Feed new thread through one completed tunnel; don't move tissue.

        Topology is a material-eyelet approximation. Feed length is explicitly
        added by a virtual spool. It stops feeding during the tightening phase.
        """
        j=int(np.argmin(abs(self.ys-(stitch-1)*.003)))
        if not self.route:
            anchor=self.origin+np.array([.015,0,.018])
            self.route=[self.add_particle(anchor,True)]
        else:
            self.route.pop()  # replace previous needle endpoint by new tunnel
            self.w[self.cable_end]=0
        # Entry / inner wall / inner wall / exit: one path through both halves.
        guides=[self.ids[1,2,j,2],self.ids[1,0,j,2],
                self.ids[0,0,j,2],self.ids[0,2,j,2]]
        # Quasi-static massless cable: material eyelets are its degrees of
        # freedom. Avoid inventing independent per-stitch shortening forces.
        self.route.extend(int(guide) for guide in guides)
        self.cable_end=self.add_particle(np.array(tail),True)
        self.route.append(self.cable_end)
        self.cable_rest=self.cable_length()+slack

    def start_strand(self, tail):
        """The same strand exists before the first puncture and stays attached."""
        if self.route:
            raise RuntimeError('The continuous strand has already been initialized')
        anchor=self.origin+np.array([.015,0,.018])
        self.route=[self.add_particle(anchor,True)]
        self.cable_end=self.add_particle(np.asarray(tail),True)
        self.route.append(self.cable_end)
        self.cable_rest=self.cable_length()+.002

    def display_cable(self, stitch, theta, phase, completed):
        """Trace the fed portion continuously while the needle eye passes through.

        During slack feeding, guide points grow from the last existing point.
        On tunnel completion this becomes exactly the material-eyelet route.
        A 0.15 mm surface offset avoids depth fighting, not a force offset.
        """
        if not self.route:raise RuntimeError('Initialize the strand before rendering')
        points=[p.copy() for p in self.x[self.route[:-1]]]
        progress=0.
        if stitch>completed and phase=='left_extract':
            sweep=2*np.arcsin(.008/.011375)
            progress=float(np.clip((theta-np.pi)/sweep,0,1))
        if progress>0:
            j=int(np.argmin(abs(self.ys-(stitch-1)*.003)))
            guides=[self.ids[1,2,j,2],self.ids[1,0,j,2],self.ids[0,0,j,2],self.ids[0,2,j,2]]
            for k,guide in enumerate(guides):
                fraction=float(np.clip(progress*4-k,0,1))
                if fraction<=0:break
                points.append(points[-1]*(1-fraction)+self.x[guide]*fraction)
        for p in points[1:]:p[2]+=.00015
        points.append(self.x[self.cable_end].copy())
        return np.array(points)

    def cable_length(self):
        if len(self.route)<2:return 0.
        return float(np.linalg.norm(np.diff(self.x[self.route],axis=0),axis=1).sum())

    def gap_mm(self):
        # Evaluate exactly the 5 sutured material stations at the top edge.
        js=[int(np.argmin(abs(self.ys-j*.003))) for j in range(5)]
        left=self.x[self.ids[0,0,js,2],0];right=self.x[self.ids[1,0,js,2],0]
        return (right-left)*1000

    def step(self, tip=None, puncture_key=None, cable_end=None, feed=False):
        return self._implicit_step(tip,puncture_key,cable_end,feed)

    def _implicit_step(self,tip,puncture_key,cable_end,feed):
        p=self.p;dt=p.dt;old=self.x.copy()
        predicted=old+self.v*np.exp(-p.damping*dt)*dt
        predicted[self.w>0,2]-=p.gravity*dt*dt
        if self.cable_end is not None and cable_end is not None:
            self.x[self.cable_end]=cable_end
            if feed:self.cable_rest=self.cable_length()+.002
        free=np.flatnonzero(self.w>0);mass=1/self.w[free]
        a,b=self.edges.T
        ids=np.array(self.route,dtype=int)
        active=tip is not None and puncture_key not in self.punctured
        contact_ids=np.array([],dtype=int);surface=np.array([])
        if active:
            r2=np.sum((self.rest[self.top,:2]-np.asarray(tip)[:2])**2,axis=1)
            mask=r2<p.tip_radius**2
            contact_ids=self.top[mask]
            surface=np.asarray(tip)[2]-np.sqrt(p.tip_radius**2-r2[mask])
        weights=None
        if active and p.distributed_contact:
            # Smooth pressure footprint avoids a force jump when crossing nodes.
            r2=np.sum((self.rest[self.top,:2]-np.asarray(tip)[:2])**2,axis=1)
            mask=r2<(.0045**2)
            contact_ids=self.top[mask]
            weights=np.exp(-r2[mask]/(2*.0018**2))
            if len(weights):weights/=weights.sum()
            surface=np.asarray(tip)[2]-p.tip_radius
        contact_k=1000.
        old_lengths=np.linalg.norm(old[b]-old[a],axis=1)
        def energy(flat):
            pos=self.x.copy();pos[free]=flat.reshape(-1,3)
            inertia=pos[free]-predicted[free]
            E=.5*np.sum(mass[:,None]*inertia*inertia)/dt**2
            grad=np.zeros_like(pos);grad[free]=mass[:,None]*inertia/dt**2
            delta=pos[b]-pos[a];length=np.linalg.norm(delta,axis=1)
            C=length-self.lengths
            E+=.5*p.spring_stiffness*np.dot(C,C)
            force=(p.spring_stiffness*C/np.maximum(length,1e-12))[:,None]*delta
            if p.strain_stiffening or p.edge_viscosity:
                # Cubic force response resists excessive stretching; viscosity
                # dissipates relative edge motion without prescribing deformation.
                E+=p.spring_stiffness*p.strain_stiffening*np.sum(C**4/self.lengths**2)/4
                E+=p.edge_viscosity*np.sum((length-old_lengths)**2)/(2*dt)
                extra=p.spring_stiffness*p.strain_stiffening*C**3/self.lengths**2+p.edge_viscosity*(length-old_lengths)/dt
                force+=extra[:,None]*delta/np.maximum(length[:,None],1e-12)
            np.add.at(grad,a,-force);np.add.at(grad,b,force)
            z=pos[self.tets];u=z[:,1]-z[:,0];v=z[:,2]-z[:,0];w=z[:,3]-z[:,0]
            volume=np.einsum('ij,ij->i',u,np.cross(v,w))/6
            J=volume/self.volume0
            # Strong compression penalty also resists element inversion.
            barrier=np.minimum(J-.25,0)
            E+=.5*p.bulk_modulus*np.sum(abs(self.volume0)*((J-1)**2+100*barrier**2))
            scale=p.bulk_modulus*np.sign(self.volume0)*((J-1)+100*barrier)/6
            g1=np.cross(v,w)*scale[:,None];g2=np.cross(w,u)*scale[:,None];g3=np.cross(u,v)*scale[:,None]
            np.add.at(grad,self.tets[:,0],-g1-g2-g3)
            np.add.at(grad,self.tets[:,1],g1);np.add.at(grad,self.tets[:,2],g2);np.add.at(grad,self.tets[:,3],g3)
            if len(ids):
                delta=np.diff(pos[ids],axis=0);length=np.linalg.norm(delta,axis=1)
                extension=max(0.,length.sum()-self.cable_rest)
                E+=.5*p.cable_stiffness*extension**2
                force=p.cable_stiffness*extension*delta/np.maximum(length[:,None],1e-12)
                np.add.at(grad,ids[:-1],-force);np.add.at(grad,ids[1:],force)
            if len(contact_ids):
                depth=np.maximum(0,pos[contact_ids,2]-surface) if weights is None else max(0.,float(weights@pos[contact_ids,2]-surface))
                E+=.5*contact_k*np.dot(depth,depth)
                np.add.at(grad[:,2],contact_ids,contact_k*depth if weights is None else contact_k*depth*weights)
            # Scale energy to improve absolute optimizer tolerances at mm scale.
            return float(E*1e4),(grad[free]*1e4).ravel()
        lo=np.full((len(free),3),-np.inf);hi=-lo.copy()
        tissue=free<self.n_tissue
        lo[tissue,2]=self.origin[2]+.004
        left=tissue & (self.rest[free,0]<self.origin[0])
        right=tissue & ~left
        hi[left,0]=self.origin[0]-p.gap_contact/2
        lo[right,0]=self.origin[0]+p.gap_contact/2
        result=minimize(energy,self.x[free].ravel(),jac=True,method='L-BFGS-B',
            bounds=list(zip(lo.ravel(),hi.ravel())),
            options=dict(maxiter=500,ftol=1e-9,gtol=1e-4,maxls=30,maxcor=12))
        self.x[free]=result.x.reshape(-1,3)
        self.last_solver=dict(success=bool(result.success),iterations=int(result.nit),message=str(result.message))
        if not result.success:raise RuntimeError('Implicit mechanics failed: '+str(result.message))
        self.tip_force=float(contact_k*np.maximum(0,self.x[contact_ids,2]-surface).sum()) if weights is None else float(contact_k*max(0.,weights@self.x[contact_ids,2]-surface)) if len(weights) else 0.
        if active and puncture_key is not None and self.tip_force>=p.puncture_force:
            self.punctured.add(puncture_key)
            self.puncture_events.append(dict(time=self.time,key=int(puncture_key),force_N=self.tip_force,
                indentation_mm=float(np.max(self.rest[self.top,2]-self.x[self.top,2])*1000)))
        self.tension=p.cable_stiffness*max(0,self.cable_length()-self.cable_rest)
        self.v=(self.x-old)/dt;self.v[self.w==0]=0
        self.time+=dt
        if not np.isfinite(self.x).all():raise FloatingPointError('non-finite mechanics state')
        return dict(time=self.time,tension_N=self.tension,tip_force_N=self.tip_force,
            gap_mm=self.gap_mm().tolist(),punctures=len(self.punctured),
            max_displacement_mm=float(np.max(np.linalg.norm(self.x[:self.n_tissue]-self.rest[:self.n_tissue],axis=1))*1000))

