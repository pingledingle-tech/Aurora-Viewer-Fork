//! Build › Pingmator link: turning the loopback link on and off, and laying
//! the poses it receives on our own avatar through the local poser.
use super::*;

impl App {
    pub(super) fn toggle_pingmator_link(&mut self) {
        if self.pingmator_link.enabled() {
            self.pingmator_link.stop();
            self.panels.pingmator_link = false;
            self.world.system_message(aurora_i18n::tr!("pingmator-link-off"));
            return;
        }
        let port = crate::pingmator_link::DEFAULT_PORT;
        match self.pingmator_link.start(port, self.world.avatar_lib.skeleton.as_ref()) {
            Ok(()) => {
                self.panels.pingmator_link = true;
                self.world
                    .system_message(aurora_i18n::tr!("pingmator-link-on", port = i64::from(port)));
            }
            Err(error) => {
                self.panels.pingmator_link = false;
                self.world
                    .system_message(aurora_i18n::tr!("pingmator-link-failed", error = error.as_str()));
            }
        }
    }

    pub(super) fn tick_pingmator_link(&mut self) {
        if !self.pingmator_link.enabled() {
            return;
        }
        if let Some(client) = self.pingmator_link.take_connected() {
            self.world
                .system_message(aurora_i18n::tr!("pingmator-link-connected", client = client.as_str()));
        }
        if let Some(error) = self.pingmator_link.take_error() {
            self.pingmator_link.stop();
            self.panels.pingmator_link = false;
            self.world
                .system_message(aurora_i18n::tr!("pingmator-link-failed", error = error.as_str()));
            return;
        }
        if let Some(animation) = self.pingmator_link.take_pose()
            && let Err(animation) = self.ik_editor.apply_link_pose(&self.world, animation)
        {
            self.pingmator_link.keep(animation);
        }
    }
}
