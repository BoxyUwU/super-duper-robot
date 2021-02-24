use glam::Vec3A;
use rbot::App;
use rbot::State;
use rend3::{
    datatypes::{
        CameraProjection, DirectionalLightHandle, MaterialHandle, MeshHandle, ObjectHandle,
        RendererTextureFormat, Texture,
    },
    Renderer,
};

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

    let primitive = mesh_data
        .primitives()
        .next()
        .expect("no primitives in data.glb");
    let reader = primitive.reader(|b| Some(&datas.get(b.index())?.0[..b.length()]));

    let vertex_positions: Vec<_> = reader
        .read_positions()
        .unwrap()
        .map(glam::Vec3::from)
        .collect();
    let vertex_normals: Vec<_> = reader
        .read_normals()
        .unwrap()
        .map(glam::Vec3::from)
        .collect();
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
        data: vec![0_u8; 6 * 4],
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
    human_mesh: MeshHandle,
    human_material: MaterialHandle,
    humans: Vec<ObjectHandle>,
    light_handle: DirectionalLightHandle,
}

fn init(app: &mut App) -> Game1 {
    let (mesh, material) = load_gltf(
        &app.renderer,
        concat!(env!("CARGO_MANIFEST_DIR"), "/data/human.glb"),
    );

    let mut humans = vec![];

    for x in 0..10 {
        for z in 0..10 {
            // Combine the mesh and the material with a location to give an object.
            let object = rend3::datatypes::Object {
                mesh,
                material,
                transform: rend3::datatypes::AffineTransform {
                    // Need to flip gltf's coords and winding order
                    transform: glam::Mat4::from_scale_rotation_translation(
                        glam::Vec3::new(1.0, 1.0, -1.0),
                        glam::Quat::default(),
                        glam::Vec3::new(x as f32 * 5.0, 0.0, z as f32 * 5.0),
                    ),
                },
            };
            let object_handle = app.renderer.add_object(object);
            humans.push(object_handle);
        }
    }

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
            color: glam::Vec3::one(),
            intensity: 10.0,
            // Direction will be normalized
            direction: glam::Vec3::new(-1.0, -4.0, 2.0),
        });

    load_skybox(&app.renderer);

    Game1 {
        camera,
        human_mesh: mesh,
        human_material: material,
        humans,
        light_handle,
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
                Vec3A::new(
                    yaw.sin() * pitch.cos(),
                    -pitch.sin(),
                    yaw.cos() * pitch.cos(),
                )
            } else {
                unreachable!()
            }
        };
        let up = Vec3A::unit_y();
        let side: Vec3A = forward.cross(up).normalize();
        let velocity = match app.input.is_scancode_pressed(platform::Scancodes::SHIFT) {
            true => 2.0,
            false => 1.0,
        };

        if app.input.is_scancode_pressed(platform::Scancodes::W) {
            self.camera.location += forward * velocity * app.delta_time;
        }
        if app.input.is_scancode_pressed(platform::Scancodes::S) {
            self.camera.location -= forward * velocity * app.delta_time;
        }
        if app.input.is_scancode_pressed(platform::Scancodes::A) {
            self.camera.location += side * velocity * app.delta_time;
        }
        if app.input.is_scancode_pressed(platform::Scancodes::D) {
            self.camera.location -= side * velocity * app.delta_time;
        }
        if app.input.is_scancode_pressed(platform::Scancodes::Q) {
            self.camera.location += up * velocity * app.delta_time;
        }
        if app.input.is_scancode_pressed(platform::Scancodes::Z) {
            self.camera.location -= up * velocity * app.delta_time;
        }

        app.renderer.set_camera_data(self.camera);
    }
}
