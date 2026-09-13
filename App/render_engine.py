import os, math, time, threading
import numpy as np
from PIL import Image
import moderngl, glfw, trimesh

_VERT = '''
#version 330 core
in vec3 in_position;
in vec3 in_normal;
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;
out vec3 v_pos;
out vec3 v_norm;
void main() {
    vec4 world  = u_model * vec4(in_position, 1.0);
    v_pos       = world.xyz;
    v_norm      = normalize(mat3(u_model) * in_normal);
    gl_Position = u_proj * u_view * world;
}
'''

_FRAG = '''
#version 330 core
in vec3 v_pos;
in vec3 v_norm;
uniform vec3  u_color;
uniform vec3  u_light_pos;
uniform vec3  u_view_pos;
uniform float u_ambient;
out vec4 out_color;
void main() {
    vec3 N = normalize(v_norm);
    vec3 L = normalize(u_light_pos - v_pos);
    vec3 V = normalize(u_view_pos  - v_pos);
    vec3 H = normalize(L + V);
    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), 64.0);
    vec3 col = (u_ambient + diff * 0.72 + spec * 0.28) * u_color;
    out_color = vec4(col, 1.0);
}
'''

_GRID_VERT = '''
#version 330 core
in vec3 in_pos;
uniform mat4 u_view;
uniform mat4 u_proj;
void main() { gl_Position = u_proj * u_view * vec4(in_pos, 1.0); }
'''

_GRID_FRAG = '''
#version 330 core
out vec4 out_color;
void main() { out_color = vec4(0.32, 0.32, 0.32, 0.85); }
'''

_AXIS_VERT = '''
#version 330 core
in vec3 in_pos;
in vec3 in_col;
uniform mat4 u_view;
uniform mat4 u_proj;
out vec3 v_col;
void main() { gl_Position = u_proj * u_view * vec4(in_pos, 1.0); v_col = in_col; }
'''

_AXIS_FRAG = '''
#version 330 core
in vec3 v_col;
out vec4 out_color;
void main() { out_color = vec4(v_col, 1.0); }
'''

def quat_to_mat4(pos, q):
    x,y,z,w = q
    M = np.eye(4, dtype=np.float32)
    M[0,0]=1-2*(y*y+z*z); M[0,1]=2*(x*y-z*w); M[0,2]=2*(x*z+y*w)
    M[1,0]=2*(x*y+z*w);   M[1,1]=1-2*(x*x+z*z); M[1,2]=2*(y*z-x*w)
    M[2,0]=2*(x*z-y*w);   M[2,1]=2*(y*z+x*w); M[2,2]=1-2*(x*x+y*y)
    M[0,3]=pos[0]; M[1,3]=pos[1]; M[2,3]=pos[2]
    return M

def _look_at(eye, target, up=np.array([0.,0.,1.])):
    f=target-eye; f/=np.linalg.norm(f)
    r=np.cross(f,up); r/=np.linalg.norm(r)
    u=np.cross(r,f)
    M=np.eye(4,dtype=np.float32)
    M[0,:3]=r;  M[0,3]=-float(np.dot(r,eye))
    M[1,:3]=u;  M[1,3]=-float(np.dot(u,eye))
    M[2,:3]=-f; M[2,3]= float(np.dot(f,eye))
    return M

def _perspective(fov_deg, aspect, near, far):
    f=1.0/math.tan(math.radians(fov_deg)/2.0)
    M=np.zeros((4,4),dtype=np.float32)
    M[0,0]=f/aspect; M[1,1]=f
    M[2,2]=(far+near)/(near-far); M[2,3]=2.0*far*near/(near-far)
    M[3,2]=-1.0
    return M

def _eye_pos(target, yaw_deg, pitch_deg, dist):
    y=math.radians(yaw_deg); p=math.radians(pitch_deg)
    t=np.asarray(target,dtype=np.float64)
    return t+dist*np.array([math.cos(p)*math.sin(y), math.cos(p)*(-math.cos(y)), math.sin(-p)])

def _grid_verts(size=1.5,step=0.2):
    lines=[]
    for c in np.arange(-size,size+step*0.5,step):
        lines+=[[-size,c,0.001],[size,c,0.001]]
        lines+=[[c,-size,0.001],[c,size,0.001]]
    return np.array(lines,dtype=np.float32)

def _axis_verts():
    L=0.25
    return np.array([0,0,0.005,1,0,0, L,0,0.005,1,0,0,
                     0,0,0.005,0,0.8,0, 0,L,0.005,0,0.8,0,
                     0,0,0.005,0,0.4,1, 0,0,L,0,0.4,1],dtype=np.float32)

def _box_verts(hx,hy,hz):
    data=[]
    faces=[([[ hx,-hy,-hz],[ hx, hy,-hz],[ hx, hy, hz],[ hx,-hy, hz]],[1,0,0]),
           ([[-hx, hy,-hz],[-hx,-hy,-hz],[-hx,-hy, hz],[-hx, hy, hz]],[-1,0,0]),
           ([[-hx, hy,-hz],[ hx, hy,-hz],[ hx, hy, hz],[-hx, hy, hz]],[0,1,0]),
           ([[ hx,-hy,-hz],[-hx,-hy,-hz],[-hx,-hy, hz],[ hx,-hy, hz]],[0,-1,0]),
           ([[-hx,-hy, hz],[ hx,-hy, hz],[ hx, hy, hz],[-hx, hy, hz]],[0,0,1]),
           ([[-hx, hy,-hz],[ hx, hy,-hz],[ hx,-hy,-hz],[-hx,-hy,-hz]],[0,0,-1])]
    for verts,n in faces:
        v0,v1,v2,v3=verts
        for v in [v0,v1,v2,v0,v2,v3]: data+=v+n
    return np.array(data,dtype=np.float32)

LINK_NAMES=['base_link','link_1','link_2','link_3','link_4','link_5','link_6']
LINK_COLORS={'base_link':(0.22,0.22,0.22),'link_1':(0.96,0.76,0.13),'link_2':(0.96,0.76,0.13),
             'link_3':(0.96,0.76,0.13),'link_4':(0.96,0.76,0.13),'link_5':(0.15,0.15,0.15),
             'link_6':(0.22,0.22,0.22)}
PRESETS={'standard':{'render':(640,800),'display':(600,800)},
         'hd':{'render':(960,1200),'display':(600,800)},
         'fhd':{'render':(1280,1600),'display':(600,800)}}

class RobotGPURenderer:
    def __init__(self,mesh_dir,q_img,shared_state,state_lock):
        self.mesh_dir=mesh_dir; self.q_img=q_img
        self.shared=shared_state; self.lock=state_lock
        self._thread=threading.Thread(target=self._run,daemon=True,name='GPURender')
    def start(self): self._thread.start()

    @staticmethod
    def _make_fbo(ctx,w,h):
        return ctx.framebuffer(color_attachments=[ctx.texture((w,h),4)],
                               depth_attachment=ctx.depth_texture((w,h)))

    @staticmethod
    def _load_vao(ctx,prog,path):
        mesh=trimesh.load(path,process=True)
        v=np.asarray(mesh.vertices,dtype=np.float32)
        n=np.asarray(mesh.vertex_normals,dtype=np.float32)
        f=np.asarray(mesh.faces,dtype=np.uint32)
        data=np.hstack([v,n]).astype(np.float32)
        vbo=ctx.buffer(data.tobytes()); ibo=ctx.buffer(f.tobytes())
        return ctx.vertex_array(prog,[(vbo,'3f 3f','in_position','in_normal')],ibo)

    def _run(self):
        if not glfw.init(): print('[GPURender] glfw.init failed'); return
        glfw.window_hint(glfw.VISIBLE,glfw.FALSE)
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR,3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR,3)
        glfw.window_hint(glfw.OPENGL_PROFILE,glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.DOUBLEBUFFER,glfw.FALSE)
        win=glfw.create_window(640,800,'offscreen',None,None)
        if not win: glfw.terminate(); print('[GPURender] window failed'); return
        glfw.make_context_current(win)
        ctx=moderngl.create_context()
        ctx.enable(moderngl.DEPTH_TEST); ctx.enable(moderngl.CULL_FACE)

        prog=ctx.program(vertex_shader=_VERT,fragment_shader=_FRAG)
        grid_prog=ctx.program(vertex_shader=_GRID_VERT,fragment_shader=_GRID_FRAG)
        axis_prog=ctx.program(vertex_shader=_AXIS_VERT,fragment_shader=_AXIS_FRAG)

        robot_vaos = {}
        current_link_names = list(LINK_NAMES)
        for name in LINK_NAMES:
            p = os.path.join(self.mesh_dir, f'{name}.stl')
            try:
                robot_vaos[name] = (self._load_vao(ctx, prog, p), LINK_COLORS.get(name, (0.8, 0.8, 0.8)))
            except Exception as e:
                print(f'[GPURender] {name}: {e}')

        cur_grid_size = 1.5
        grid_vao = ctx.vertex_array(grid_prog, [(ctx.buffer(_grid_verts(size=cur_grid_size, step=0.2).tobytes()), '3f', 'in_pos')])
        axis_vao = ctx.vertex_array(axis_prog, [(ctx.buffer(_axis_verts().tobytes()), '3f 3f', 'in_pos', 'in_col')])

        box_vao = None; box_vbo = None; box_key = None
        cur_rw, cur_rh = 640, 800
        fbo = self._make_fbo(ctx, cur_rw, cur_rh)
        cam_yaw, cam_pitch, cam_dist = 45.0, -25.0, 1.8
        cam_target = np.array([0., 0., 0.45])
        last_t = time.perf_counter()

        while True:
            new_mesh_data = None
            with self.lock:
                if not self.shared.get('running', True): break
                transforms = self.shared.get('transforms', [None] * len(current_link_names))
                obstacle = self.shared.get('obstacle', None)
                preset = self.shared.get('preset', 'standard')

                cam_dist = float(self.shared.get('cam_dist', cam_dist))
                raw_tgt = self.shared.get('cam_target', cam_target)
                if raw_tgt is not None:
                    cam_target = np.asarray(raw_tgt, dtype=np.float64)
                cam_yaw = float(self.shared.get('cam_yaw', cam_yaw))
                cam_pitch = float(self.shared.get('cam_pitch', cam_pitch))
                new_grid_size = float(self.shared.get('grid_size', cur_grid_size))

                if self.shared.get('reload_meshes', False):
                    new_mesh_data = self.shared.get('mesh_data', [])
                    self.shared['reload_meshes'] = False

            # Izgara boyutunu robot ölçeğine göre güncelle
            if abs(new_grid_size - cur_grid_size) > 0.05:
                cur_grid_size = new_grid_size
                step = max(0.1, cur_grid_size / 8.0)
                grid_vao.release()
                grid_vao = ctx.vertex_array(grid_prog, [(ctx.buffer(_grid_verts(size=cur_grid_size, step=step).tobytes()), '3f', 'in_pos')])

            # Dinamik model yükleme talebi geldiğinde VAO'ları güncelle
            if new_mesh_data is not None:
                for vao_item in robot_vaos.values():
                    try:
                        if isinstance(vao_item, tuple): vao_item[0].release()
                        elif hasattr(vao_item, 'release'): vao_item.release()
                    except Exception:
                        pass
                robot_vaos.clear()
                current_link_names = []

                for item in new_mesh_data:
                    if isinstance(item, dict):
                        name = item.get('name', 'link')
                        mesh_src = item.get('mesh_path')
                        col = item.get('color', (0.85, 0.85, 0.85))
                        geom_type = item.get('geom_type', 5)
                        dims = item.get('dims', (0.1, 0.1, 0.1))
                    else:
                        name = item[0]
                        mesh_src = item[1]
                        col = item[2] if len(item) > 2 and item[2] else (0.85, 0.85, 0.85)
                        geom_type = item[3] if len(item) > 3 else 5
                        dims = item[4] if len(item) > 4 else (0.1, 0.1, 0.1)

                    current_link_names.append(name)
                    vao = None

                    if mesh_src and isinstance(mesh_src, str) and os.path.isfile(mesh_src):
                        try:
                            vao = self._load_vao(ctx, prog, mesh_src)
                        except Exception as e:
                            print(f'[GPURender] Mesh okunamadı ({name}): {e}')

                    # Eğer mesh dosyası yoksa veya geometrik ilkel ise prosedürel geometri oluştur
                    if vao is None:
                        try:
                            if geom_type == 3:  # GEOM_BOX
                                hx, hy, hz = dims[0], dims[1], dims[2]
                                mesh_obj = trimesh.creation.box(extents=[max(0.01, hx * 2), max(0.01, hy * 2), max(0.01, hz * 2)])
                            elif geom_type == 4:  # GEOM_CYLINDER
                                length, radius = dims[0], dims[1]
                                mesh_obj = trimesh.creation.cylinder(radius=max(0.005, radius), height=max(0.01, length))
                            elif geom_type == 2:  # GEOM_SPHERE
                                mesh_obj = trimesh.creation.icosphere(radius=max(0.005, dims[0]))
                            else:
                                mesh_obj = trimesh.creation.cylinder(radius=0.035, height=0.09)

                            v = np.asarray(mesh_obj.vertices, dtype=np.float32)
                            n = np.asarray(mesh_obj.vertex_normals, dtype=np.float32)
                            f = np.asarray(mesh_obj.faces, dtype=np.uint32)
                            data = np.hstack([v, n]).astype(np.float32)
                            vbo = ctx.buffer(data.tobytes())
                            ibo = ctx.buffer(f.tobytes())
                            vao = ctx.vertex_array(prog, [(vbo, '3f 3f', 'in_position', 'in_normal')], ibo)
                        except Exception as e:
                            print(f'[GPURender] Prosedürel VAO hatası ({name}): {e}')

                    if vao is not None:
                        robot_vaos[name] = (vao, col)

            cfg = PRESETS.get(preset, PRESETS['standard'])
            rw, rh = cfg['render']; dw, dh = cfg['display']
            if rw != cur_rw or rh != cur_rh:
                fbo.release(); cur_rw, cur_rh = rw, rh; fbo = self._make_fbo(ctx, cur_rw, cur_rh)
            eye = _eye_pos(cam_target, cam_yaw, cam_pitch, cam_dist)
            view = _look_at(eye, cam_target)
            proj = _perspective(60.0, cur_rw / cur_rh, 0.01, max(100.0, cam_dist * 10.0))
            view_b = view.T.astype(np.float32).tobytes()
            proj_b = proj.T.astype(np.float32).tobytes()
            fbo.use(); ctx.viewport = (0, 0, cur_rw, cur_rh); ctx.clear(0.098, 0.098, 0.098, 1.0)
            prog['u_view'].write(view_b); prog['u_proj'].write(proj_b)
            prog['u_light_pos'].value = (cam_dist * 0.8, cam_dist * 0.6, cam_dist * 1.5)
            prog['u_view_pos'].value = tuple(float(e) for e in eye)
            prog['u_ambient'].value = 0.35
            for i, name in enumerate(current_link_names):
                T = (transforms[i] if (transforms and i < len(transforms) and transforms[i] is not None) else np.eye(4, dtype=np.float32))
                prog['u_model'].write(T.T.astype(np.float32).tobytes())
                if name in robot_vaos:
                    vao_entry = robot_vaos[name]
                    if isinstance(vao_entry, tuple):
                        vao, col = vao_entry
                        prog['u_color'].value = col
                        vao.render()
                    else:
                        prog['u_color'].value = LINK_COLORS.get(name, (0.8, 0.8, 0.8))
                        vao_entry.render()
            if obstacle is not None:
                obs_pos,half=obstacle
                key=(round(half[0],4),round(half[1],4),round(half[2],4))
                if key!=box_key:
                    if box_vao is not None: box_vao.release(); box_vbo.release()
                    bv=_box_verts(*half); box_vbo=ctx.buffer(bv.tobytes())
                    box_vao=ctx.vertex_array(prog,[(box_vbo,'3f 3f','in_position','in_normal')]); box_key=key
                T_box=np.eye(4,dtype=np.float32); T_box[0,3]=obs_pos[0]; T_box[1,3]=obs_pos[1]; T_box[2,3]=obs_pos[2]
                prog['u_model'].write(T_box.T.astype(np.float32).tobytes())
                prog['u_color'].value=(0.85,0.14,0.04); box_vao.render()
            ctx.enable(moderngl.BLEND); ctx.blend_func=moderngl.SRC_ALPHA,moderngl.ONE_MINUS_SRC_ALPHA
            grid_prog['u_view'].write(view_b); grid_prog['u_proj'].write(proj_b)
            grid_vao.render(moderngl.LINES)
            axis_prog['u_view'].write(view_b); axis_prog['u_proj'].write(proj_b)
            axis_vao.render(moderngl.LINES)
            ctx.disable(moderngl.BLEND)
            raw=fbo.read(components=3)
            img=Image.frombytes('RGB',(cur_rw,cur_rh),raw).transpose(Image.FLIP_TOP_BOTTOM)
            if (cur_rw,cur_rh)!=(dw,dh):
                try: img=img.resize((dw,dh),Image.Resampling.LANCZOS)
                except: img=img.resize((dw,dh),Image.LANCZOS)
            if not self.q_img.full(): self.q_img.put(img)
            glfw.poll_events()
            now=time.perf_counter()
            s=(1.0/60.0)-(now-last_t)
            if s>0.001: time.sleep(s)
            last_t=time.perf_counter()

        fbo.release()
        if box_vao: box_vao.release(); box_vbo.release()
        glfw.destroy_window(win); glfw.terminate()
        print('[GPURender] Stopped.')
