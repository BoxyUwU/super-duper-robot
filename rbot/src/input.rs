use std::collections::HashSet;

use winit::event::{
    DeviceEvent, ElementState, KeyboardInput, MouseButton, ScanCode, VirtualKeyCode,
};

#[derive(Clone)]
pub struct InputCtx {
    prev_ctx: Option<Box<InputCtx>>,
    pressed_scancodes: HashSet<ScanCode>,
    pressed_keycodes: HashSet<VirtualKeyCode>,
    pressed_mouse_buttons: HashSet<MouseButton>,
    mouse_deltas: Vec<(f64, f64)>,
}

impl InputCtx {
    pub(crate) fn new_empty() -> Self {
        InputCtx {
            prev_ctx: Some(Box::new(InputCtx {
                prev_ctx: None,
                pressed_scancodes: HashSet::new(),
                pressed_keycodes: HashSet::new(),
                pressed_mouse_buttons: HashSet::new(),
                mouse_deltas: Vec::new(),
            })),
            pressed_scancodes: HashSet::new(),
            pressed_keycodes: HashSet::new(),
            pressed_mouse_buttons: HashSet::new(),
            mouse_deltas: Vec::new(),
        }
    }

    pub(crate) fn next_frame(&mut self) {
        self.prev_ctx = None;
        self.prev_ctx = Some(Box::new(self.clone()));
        self.mouse_deltas.clear();
    }

    pub(crate) fn update_from_mouse_motion_event(&mut self, event: DeviceEvent) {
        if let DeviceEvent::MouseMotion { delta, .. } = event {
            self.mouse_deltas.push(delta);
        }
    }

    pub(crate) fn update_from_keyboard_event(&mut self, event: KeyboardInput) {
        let KeyboardInput {
            scancode,
            virtual_keycode,
            state,
            ..
        } = event;

        match state {
            ElementState::Pressed => {
                self.pressed_scancodes.insert(scancode);
                if let Some(keycode) = virtual_keycode {
                    self.pressed_keycodes.insert(keycode);
                }
            }
            ElementState::Released => {
                self.pressed_scancodes.remove(&scancode);
                if let Some(keycode) = virtual_keycode {
                    self.pressed_keycodes.remove(&keycode);
                }
            }
        }
    }

    pub(crate) fn update_from_mouse_event(&mut self, state: ElementState, button: MouseButton) {
        match state {
            ElementState::Pressed => {
                self.pressed_mouse_buttons.insert(button);
            }
            ElementState::Released => {
                self.pressed_mouse_buttons.remove(&button);
            }
        }
    }

    pub fn is_scancode_pressed(&self, scancode: ScanCode) -> bool {
        self.pressed_scancodes.get(&scancode).is_some()
    }

    pub fn is_keycode_pressed(&self, keycode: VirtualKeyCode) -> bool {
        self.pressed_keycodes.get(&keycode).is_some()
    }

    pub fn is_keycode_just_pressed(&self, keycode: VirtualKeyCode) -> bool {
        self.is_keycode_pressed(keycode)
            && self.prev_ctx.as_ref().unwrap().is_keycode_pressed(keycode) == false
    }

    pub fn is_keycode_just_released(&self, keycode: VirtualKeyCode) -> bool {
        self.is_keycode_pressed(keycode) == false
            && self.prev_ctx.as_ref().unwrap().is_keycode_pressed(keycode) == true
    }

    pub fn is_mouse_button_pressed(&self, button: MouseButton) -> bool {
        self.pressed_mouse_buttons.get(&button).is_some()
    }

    pub fn mouse_deltas(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.mouse_deltas.iter().copied()
    }
}
