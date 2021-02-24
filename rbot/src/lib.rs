use rend3::Renderer;
use rend3_list::DefaultPipelines;
use std::{sync::Arc, time::Instant};
use winit::{event::*, window::Window};

pub mod input;
pub use input::InputCtx;

pub struct App<T: 'static> {
    options: rend3::RendererOptions,
    event_loop: winit::event_loop::EventLoop<()>,
    window: Window,
    renderer: Arc<Renderer>,
    pipelines: DefaultPipelines,
    state: T,
}

impl<T: 'static> App<T> {
    pub fn new(init: impl FnOnce(&Renderer) -> T) -> Self {
        // Create event loop and window
        let event_loop = winit::event_loop::EventLoop::new();
        let window = {
            let mut builder = winit::window::WindowBuilder::new();
            builder = builder.with_title("rend3 cube");
            builder.build(&event_loop).expect("Could not build window")
        };

        let window_size = window.inner_size();

        let options = rend3::RendererOptions {
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

        Self {
            options,
            event_loop,
            window,
            state: init(&renderer),
            renderer,
            pipelines,
        }
    }

    pub fn run(self, mut update: impl FnMut(&mut T, InputCtx, &Renderer, f32) + 'static) {
        let Self {
            mut options,
            event_loop,
            window,
            renderer,
            pipelines,
            mut state,
        } = self;

        // Setup logging
        wgpu_subscriber::initialize_default_subscriber(None);

        let mut timestamp_last_frame = Instant::now();
        let mut frame_times = histogram::Histogram::new();

        let mut input_ctx = InputCtx::new_empty();

        event_loop.run(move |event, _, control| match event {
            Event::WindowEvent {
                event: WindowEvent::KeyboardInput { input, .. },
                ..
            } => {
                input_ctx.update_from_keyboard_event(input);
            }
            Event::WindowEvent {
                event: WindowEvent::MouseInput { state, button, .. },
                ..
            } => {
                input_ctx.update_from_mouse_event(state, button);
            }
            Event::WindowEvent {
                event: WindowEvent::Focused(_),
                ..
            } => {
                //cursor_grab = b;
            }
            // Close button was clicked, we should close.
            Event::WindowEvent {
                event: WindowEvent::CloseRequested,
                ..
            } => {
                *control = winit::event_loop::ControlFlow::Exit;
            }
            // Window was resized, need to resize renderer.
            Event::WindowEvent {
                event: WindowEvent::Resized(size),
                ..
            } => {
                options.size = [size.width, size.height];
            }
            Event::DeviceEvent {
                event: event @ DeviceEvent::MouseMotion { .. },
                ..
            } => {
                input_ctx.update_from_mouse_motion_event(event);
            }
            // Render!
            Event::MainEventsCleared => {
                window.request_redraw();
            }
            Event::RedrawRequested(..) => {
                let now = Instant::now();
                let delta_time = now - timestamp_last_frame;
                frame_times
                    .increment(delta_time.as_micros() as u64)
                    .unwrap();
                timestamp_last_frame = now;

                update(
                    &mut state,
                    input_ctx.clone(),
                    &renderer,
                    delta_time.as_secs_f32(),
                );
                renderer.set_options(options.clone());

                // Size of the internal buffers used for rendering.
                //
                // This can be different from the size of the swapchain,
                // it will be scaled to the swapchain size when being
                // rendered onto the swapchain.
                let internal_renderbuffer_size = options.size;

                // Default set of rendering commands using the default shaders.
                let render_list = rend3_list::default_render_list(
                    renderer.mode(),
                    internal_renderbuffer_size,
                    &pipelines,
                );

                // Dispatch a render!
                let handle = renderer.render(render_list, rend3::RendererOutput::InternalSwapchain);

                // Wait until it's done
                pollster::block_on(handle);

                input_ctx.next_frame();
            }
            // Other events we don't care about
            _ => {}
        });
    }
}
