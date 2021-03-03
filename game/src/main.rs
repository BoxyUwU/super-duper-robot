use std::collections::{HashMap, HashSet};

use glam::{quat, Mat4, Vec3, Vec3A, Vec4};
use gltf::buffer::Data;
use rapier3d::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};
use rapier3d::na::{Isometry3, Point3, Vector3};
use rapier3d::pipeline::PhysicsPipeline;
use rapier3d::{
    dynamics::{IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodyHandle, RigidBodySet},
    na::Quaternion,
};
use rbot::App;
use rbot::State;
use rend3::{
    datatypes::{
        CameraProjection, DirectionalLightHandle, MaterialHandle, MeshHandle, ObjectHandle,
        RendererTextureFormat, Texture,
    },
    Renderer,
};
use winit::event::VirtualKeyCode;

mod platform;

fn load_gltf(
    renderer: &rend3::Renderer,
    path: &'static str,
) -> (
    rend3::datatypes::MeshHandle,
    rend3::datatypes::MaterialHandle,
) {
    let (doc, datas, _) = gltf::import(path).unwrap();
    let mesh_data = doc.meshes().next().expect("no meshes in data.glb");
    load_mesh(mesh_data, &datas, renderer)
}

fn load_mesh(
    mesh: gltf::Mesh,
    data: &[Data],
    renderer: &rend3::Renderer,
) -> (
    rend3::datatypes::MeshHandle,
    rend3::datatypes::MaterialHandle,
) {
    let primitive = mesh.primitives().next().expect("no primitives in data.glb");
    let reader = primitive.reader(|b| Some(&data.get(b.index())?.0[..b.length()]));

    let vertex_positions: Vec<_> = reader.read_positions().unwrap().map(Vec3::from).collect();
    let vertex_normals: Vec<_> = reader.read_normals().unwrap().map(Vec3::from).collect();
    let vertex_tangents: Vec<_> = reader
        .read_tangents()
        .unwrap()
        .map(glam::Vec4::from)
        .map(From::from)
        .collect();
    let vertex_uvs: Vec<_> = reader
        .read_tex_coords(0)
        .unwrap()
        .into_f32()
        .map(glam::Vec2::from)
        .collect();
    let indices = reader.read_indices().unwrap().into_u32().collect();

    let mesh = rend3::datatypes::MeshBuilder::new(vertex_positions.to_vec())
        .with_indices(indices)
        .with_vertex_normals(vertex_normals)
        .with_vertex_tangents(vertex_tangents)
        .with_vertex_uvs(vertex_uvs)
        .with_right_handed()
        .build();

    // Add mesh to renderer's world
    let mesh_handle = renderer.add_mesh(mesh);

    // Add basic material with all defaults except a single color.
    let material = primitive.material();
    let metallic_roughness = material.pbr_metallic_roughness();
    let material_handle = renderer.add_material(rend3::datatypes::Material {
        albedo: rend3::datatypes::AlbedoComponent::Value(
            metallic_roughness.base_color_factor().into(),
        ),
        ..Default::default()
    });

    (mesh_handle, material_handle)
}

fn load_skybox(renderer: &Renderer) {
    let handle = renderer.add_texture_cube(Texture {
        format: RendererTextureFormat::Rgba8Linear,
        width: 1,
        height: 1,
        data: vec![
            20, 20, 20, 255, 20, 20, 20, 255, 20, 20, 20, 255, 20, 20, 20, 255, 20, 20, 20, 255,
            20, 20, 20, 255,
        ],
        label: None,
        mip_levels: 1,
    });
    renderer.set_background_texture(handle);
}

fn main() {
    App::run(init);
}

struct Game1 {
    camera: rend3::datatypes::Camera,

    navmesh: Navmesh,

    human_mesh: MeshHandle,
    human_material: MaterialHandle,
    human_handle: ObjectHandle,
    human_position: Vec3,

    path: Vec<usize>,

    light_handle: DirectionalLightHandle,

    level_mesh: MeshHandle,
    level_material: MaterialHandle,
    level_handle: ObjectHandle,

    // physics
    physics_pipeline: PhysicsPipeline,
    gravity: Vector3<f32>,
    physics_integration_parameters: IntegrationParameters,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    rigidbodies: RigidBodySet,
    colliders: ColliderSet,
    joints: JointSet,

    player_body_handle: RigidBodyHandle,
    level_body_handle: RigidBodyHandle,
}

#[derive(Debug)]
struct Triangle {
    verts: [usize; 3],
    neighbours: Vec<usize>,
}

struct Navmesh {
    verts: Box<[[f32; 3]]>,
    tris: Box<[Triangle]>,
    vert_to_tris: HashMap<usize, Box<[usize]>>,
}

impl Navmesh {
    fn path_from_vert_to_vert(&self, from: usize, to: usize) -> Vec<usize> {
        #[derive(Eq, PartialEq, Hash, Copy, Clone)]
        struct Node {
            vert: usize,
        }

        let calculate_vert_distance = |v1: usize, v2: usize| -> u32 {
            let from: Vec3 = self.verts[v1].into();
            let to: Vec3 = self.verts[v2].into();
            (to - from).abs().length() as u32
        };

        let calculate_f_score =
            |g_score: u32, vert: usize| -> u32 { g_score + calculate_vert_distance(vert, to) };

        let start_node = Node { vert: from };

        let mut open_set = Vec::<Node>::new();
        open_set.push(start_node);

        let mut came_from = HashMap::<Node, Node>::new();

        let mut g_scores = HashMap::<Node, u32>::new();
        g_scores.insert(start_node, 0);

        let mut f_scores = HashMap::<Node, u32>::new();
        f_scores.insert(start_node, calculate_f_score(0, start_node.vert));

        loop {
            open_set.sort_by(|n1, n2| std::cmp::Ord::cmp(&f_scores[&n1], &f_scores[&n2]));
            let cur_node = open_set.remove(0); // this will panic if there's no path
            if cur_node.vert == to {
                // reached goal
                let mut path = Vec::new();
                path.push(cur_node.vert);

                let mut cur_node = cur_node;
                while let Some(prev_node) = came_from.get(&cur_node) {
                    path.insert(0, prev_node.vert);
                    cur_node = *prev_node;
                }

                return path;
            }

            for neighbour in self.vert_to_tris[&cur_node.vert]
                .iter()
                .flat_map(|&tri| &self.tris[tri].verts)
                .map(|&vert| Node { vert })
            {
                let tentative_g =
                    g_scores[&cur_node] + calculate_vert_distance(cur_node.vert, neighbour.vert);

                let &g_score = g_scores.get(&neighbour).unwrap_or(&u32::MAX);
                if tentative_g < g_score {
                    came_from.insert(neighbour, cur_node);
                    g_scores.insert(neighbour, tentative_g);
                    f_scores.insert(neighbour, calculate_f_score(tentative_g, neighbour.vert));
                    if open_set.contains(&neighbour) == false {
                        open_set.push(neighbour);
                    }
                }
            }
        }
    }
}

fn init(app: &mut App) -> Game1 {
    let navmesh = {
        let navmesh =
            obj::Obj::load(concat!(env!("CARGO_MANIFEST_DIR"), "/data/navmesh.obj")).unwrap();

        let obj::ObjData {
            position, objects, ..
        } = navmesh.data;
        let verts: Box<[[f32; 3]]> = position.into_iter().map(|[x, y, z]| [x, y, -z]).collect();

        let mut tris = objects[0].groups[0]
            .polys
            .iter()
            .map(|poly| &poly.0)
            .map(|poly| [poly[0].0, poly[1].0, poly[2].0])
            .map(|verts| Triangle {
                verts,
                neighbours: Vec::new(),
            })
            .collect::<Box<[Triangle]>>();

        let mut vert_to_tris: HashMap<usize, Vec<usize>> = HashMap::new();
        for (n, tri) in tris.iter().enumerate() {
            for vert in tri.verts.iter().copied() {
                vert_to_tris.entry(vert).or_default().push(n);
            }
        }

        for (n, tri) in tris.iter_mut().enumerate() {
            for vert in tri.verts.iter().copied() {
                for &neighbour in vert_to_tris[&vert].iter().filter(|&&tri_idx| tri_idx != n) {
                    tri.neighbours.push(neighbour);
                }
            }
        }

        let vert_to_tris = vert_to_tris
            .into_iter()
            .map(|(vert_idx, tris)| (vert_idx, tris.into_boxed_slice()))
            .collect::<HashMap<usize, Box<[usize]>>>();

        Navmesh {
            verts,
            tris,
            vert_to_tris,
        }
    };

    let (human_mesh, human_material) = load_gltf(
        &app.renderer,
        concat!(env!("CARGO_MANIFEST_DIR"), "/data/human.glb"),
    );

    let path_start_vert = 39;
    let path_end_vert = 56;
    let human_position = navmesh.verts[path_start_vert].into();
    // Combine the mesh and the material with a location to give an object.
    let object = rend3::datatypes::Object {
        mesh: human_mesh,
        material: human_material,
        transform: rend3::datatypes::AffineTransform {
            // Need to flip gltf's coords and winding order
            transform: Mat4::from_scale_rotation_translation(
                Vec3::new(1.0, 1.0, -1.0),
                glam::Quat::default(),
                human_position,
            ),
        },
    };
    let human_handle = app.renderer.add_object(object);

    let mut path = navmesh.path_from_vert_to_vert(path_start_vert, path_end_vert);
    path.remove(0);

    // Rapier stuff
    //
    let level_collider = obj::Obj::load(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/data/level_collider.obj"
    ))
    .unwrap();

    let obj::ObjData {
        position, objects, ..
    } = level_collider.data;

    let verts: Vec<Point3<f32>> = position
        .into_iter()
        .map(|[x, y, z]| Point3::new(x, y, -z))
        .collect();

    let indices: Vec<[u32; 3]> = objects[0].groups[0]
        .polys
        .iter()
        .map(|poly| &poly.0)
        .map(|poly| [poly[0].0 as u32, poly[1].0 as u32, poly[2].0 as u32])
        .collect();

    let mut body_set = RigidBodySet::new();

    let level_body = RigidBodyBuilder::new_static().build();
    let level_body = body_set.insert(level_body);
    let level_collider = ColliderBuilder::trimesh(verts, indices).build();

    let player_body = RigidBodyBuilder::new_dynamic()
        .position(Isometry3::translation(0.0, 5.0, 0.0))
        .build();
    let player_body = body_set.insert(player_body);
    let player_collider = ColliderBuilder::cuboid(0.3, 0.5, 0.3).build();

    let mut collider_set = ColliderSet::new();
    collider_set.insert(level_collider, level_body, &mut body_set);
    collider_set.insert(player_collider, player_body, &mut body_set);

    let physics_pipeline = PhysicsPipeline::new();
    let gravity = Vector3::new(0.0, -2., 0.0);
    let physics_integration_parameters = IntegrationParameters::default();
    let broad_phase = BroadPhase::new();
    let narrow_phase = NarrowPhase::new();
    let joints = JointSet::new();

    // Load level
    let (level_mesh, level_material) = load_gltf(
        &app.renderer,
        concat!(env!("CARGO_MANIFEST_DIR"), "/data/level.glb"),
    );
    let object = rend3::datatypes::Object {
        mesh: level_mesh,
        material: level_material,
        transform: rend3::datatypes::AffineTransform {
            // Need to flip gltf's coords and winding order
            transform: Mat4::from_scale(Vec3::new(1.0, 1.0, -1.0)),
        },
    };
    let level_handle = app.renderer.add_object(object);

    // Set camera's location
    let camera = rend3::datatypes::Camera {
        projection: rend3::datatypes::CameraProjection::Projection {
            vfov: 60.0,
            near: 0.1,
            pitch: 0.5,
            yaw: -0.55,
        },
        location: glam::Vec3A::new(3.0, 3.0, -5.0),
    };

    // Create a single directional light
    let light_handle = app
        .renderer
        .add_directional_light(rend3::datatypes::DirectionalLight {
            color: Vec3::one(),
            intensity: 10.0,
            // Direction will be normalized
            direction: Vec3::new(-3.0, -3.0, 2.0),
        });

    load_skybox(&app.renderer);

    Game1 {
        camera,

        navmesh,

        human_mesh,
        human_material,
        human_handle,
        human_position,
        path,

        light_handle,

        level_material,
        level_mesh,
        level_handle,

        //physics
        physics_pipeline,
        gravity,
        physics_integration_parameters,
        broad_phase,
        narrow_phase,
        rigidbodies: body_set,
        colliders: collider_set,
        joints,

        player_body_handle: player_body,
        level_body_handle: level_body,
    }
}

impl State for Game1 {
    fn update(&mut self, app: &mut App) {
        use std::f32::consts::TAU;

        for (dx, dy) in app.input.mouse_deltas() {
            match &mut self.camera.projection {
                CameraProjection::Projection {
                    ref mut yaw,
                    ref mut pitch,
                    ..
                } => {
                    *yaw += (dx / 1000.0) as f32;
                    *pitch += (dy / 1000.0) as f32;
                    if *yaw < 0.0 {
                        *yaw += TAU;
                    } else if *yaw >= TAU {
                        *yaw -= TAU;
                    }
                    *pitch = pitch
                        .max(-std::f32::consts::FRAC_PI_2 + 0.0001)
                        .min(std::f32::consts::FRAC_PI_2 - 0.0001);
                }
                _ => unreachable!(),
            }
        }

        let forward = {
            if let CameraProjection::Projection { yaw, pitch, .. } = &mut self.camera.projection {
                Vector3::new(yaw.sin() * pitch.cos(), 0., yaw.cos() * pitch.cos()).normalize()
            } else {
                unreachable!()
            }
        };
        let side = forward.cross(&Vector3::new(0., 1., 0.)).normalize();

        let velocity = match app.input.is_scancode_pressed(platform::Scancodes::SHIFT) {
            true => 4.0,
            false => 1.0,
        };

        let mut movement = Vector3::new(0., 0., 0.);

        let jumped = app.input.is_keycode_just_pressed(VirtualKeyCode::Space);

        if app.input.is_scancode_pressed(platform::Scancodes::W) {
            movement += forward * velocity * app.delta_time;
        }
        if app.input.is_scancode_pressed(platform::Scancodes::S) {
            movement -= forward * velocity * app.delta_time;
        }
        if app.input.is_scancode_pressed(platform::Scancodes::A) {
            movement += side * velocity * app.delta_time;
        }
        if app.input.is_scancode_pressed(platform::Scancodes::D) {
            movement -= side * velocity * app.delta_time;
        }

        let camera_body = self.rigidbodies.get_mut(self.player_body_handle).unwrap();
        let new_iso = Isometry3::from_parts(
            (camera_body.position().translation.vector + movement).into(),
            camera_body.position().rotation,
        );
        camera_body.set_position(new_iso, true);
        if jumped {
            camera_body.apply_force(Vector3::new(0., 30., 0.), true);
        }

        let linvel = *camera_body.linvel();
        camera_body.set_linvel(Vector3::new(0., linvel[1], 0.), true);

        self.physics_pipeline.step(
            &self.gravity,
            &self.physics_integration_parameters,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigidbodies,
            &mut self.colliders,
            &mut self.joints,
            None,
            None,
            &(),
        );

        let camera_position = self
            .rigidbodies
            .get(self.player_body_handle)
            .unwrap()
            .position()
            .translation
            .vector;
        let camera_position = [camera_position[0], camera_position[1], camera_position[2]];
        self.camera.location = camera_position.into();
        app.renderer.set_camera_data(self.camera);

        if self.path.len() > 0 {
            let human_speed = 1.0 * app.delta_time;
            let target_pos: Vec3 = self.navmesh.verts[self.path[0]].into();
            let length = (self.human_position - target_pos).length();

            if human_speed > length {
                let overshoot = human_speed - length;
                self.human_position = target_pos;
                self.path.remove(0);
                if self.path.len() > 0 {
                    let target_pos: Vec3 = self.navmesh.verts[self.path[0]].into();
                    self.human_position +=
                        (target_pos - self.human_position).normalize() * overshoot;
                }
            } else {
                self.human_position += (target_pos - self.human_position).normalize() * human_speed;
            }
        }

        app.renderer.set_object_transform(
            self.human_handle,
            rend3::datatypes::AffineTransform {
                transform: Mat4::from_scale_rotation_translation(
                    Vec3::new(1.0, 1.0, -1.0),
                    glam::Quat::default(),
                    self.human_position,
                ),
            },
        )
    }
}
