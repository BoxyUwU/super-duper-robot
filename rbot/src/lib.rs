use rend3::Renderer;
use std::{sync::Arc, time::Instant};
use winit::event::*;

pub mod input;
pub use input::InputCtx;

pub trait State: Sized {
    fn update(&mut self, app: &mut App);
}

pub struct App {
    pub renderer: Arc<Renderer>,
    pub input: InputCtx,
    pub delta_time: f32,
}

impl App {
    pub fn run<T: State + 'static>(state_init: impl FnOnce(&mut App) -> T) {
        // Setup logging
        wgpu_subscriber::initialize_default_subscriber(None);

        // Create event loop and window
        let event_loop = winit::event_loop::EventLoop::new();
        let window = {
            let mut builder = winit::window::WindowBuilder::new();
            builder = builder.with_title("rbot engine");
            builder.build(&event_loop).expect("Could not build window")
        };

        let window_size = window.inner_size();

        let mut options = rend3::RendererOptions {
            vsync: rend3::VSyncMode::Off,
            size: [window_size.width, window_size.height],
        };

        let renderer = pollster::block_on(
            rend3::RendererBuilder::new(options.clone())
                .window(&window)
                .build(),
        )
        .unwrap();

        // Create the default set of shaders and pipelines
        let pipelines = pollster::block_on(async {
            let shaders = rend3_list::DefaultShaders::new(&renderer).await;
            rend3_list::DefaultPipelines::new(&renderer, &shaders).await
        });

        let mut app = App {
            renderer,
            input: InputCtx::new_empty(),
            delta_time: 0.0f32,
        };
        let mut state = state_init(&mut app);
        let mut timestamp_last_frame = Instant::now();

        event_loop.run(move |event, _, control| match event {
            // Close button was clicked, we should close.
            Event::WindowEvent {
                event: WindowEvent::CloseRequested,
                ..
            } => {
                *control = winit::event_loop::ControlFlow::Exit;
            }

            // Input handling
            Event::WindowEvent {
                event: WindowEvent::KeyboardInput { input, .. },
                ..
            } => {
                app.input.update_from_keyboard_event(input);
            }
            Event::WindowEvent {
                event: WindowEvent::MouseInput { state, button, .. },
                ..
            } => {
                app.input.update_from_mouse_event(state, button);
            }
            Event::DeviceEvent {
                event: event @ DeviceEvent::MouseMotion { .. },
                ..
            } => {
                app.input.update_from_mouse_motion_event(event);
            }

            // Window was resized, need to resize renderer.
            Event::WindowEvent {
                event: WindowEvent::Resized(size),
                ..
            } => {
                options.size = [size.width, size.height];
            }

            // Render!
            Event::MainEventsCleared => {
                window.request_redraw();
            }
            Event::RedrawRequested(..) => {
                let now = Instant::now();
                app.delta_time = (now - timestamp_last_frame).as_secs_f32();
                timestamp_last_frame = now;

                state.update(&mut app);

                app.renderer.set_options(options.clone());

                // Size of the internal buffers used for rendering.
                //
                // This can be different from the size of the swapchain,
                // it will be scaled to the swapchain size when being
                // rendered onto the swapchain.
                let internal_renderbuffer_size = options.size;

                // Default set of rendering commands using the default shaders.
                let render_list = rend3_list::default_render_list(
                    app.renderer.mode(),
                    internal_renderbuffer_size,
                    &pipelines,
                );

                // Dispatch a render!
                pollster::block_on(
                    app.renderer
                        .render(render_list, rend3::RendererOutput::InternalSwapchain),
                );

                app.input.next_frame();
            }
            // Other events we don't care about
            _ => {}
        });
    }
}
